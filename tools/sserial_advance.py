#!/usr/bin/env python3
# sserial0.pos_advance from LinuxCNC's own following error, the last
# commissioning step of a smart serial axis (after id_pid and the loop
# settings, and again after any firmware or loop change).
#
# The drive answers each frame with pos_fb + vel_fb * (pos_advance + its own
# snapshot age). Whatever else delays the position LinuxCNC compares against
# its command (host read/write timing in the servo thread, encoder transport,
# the drive's loop following its observer instead of the raw encoder) shows
# up as a following error proportional to velocity. This tool fits
#
#   ferror = a * vel + b * acc + c
#
# over a run, and prints sserial0.pos_advance = <advance + a>.
#
# Run on the machine with LinuxCNC on and homed, the joint free to move:
#   sserial_advance.py --joint 1 --advance 0.0003 --program /abs/path/move.ngc
# or without --program, log for --time seconds while you jog or run a program
# that moves the joint back and forth at a steady feed (cruise segments of a
# few hundred ms at 50-100 % of the joint's speed give the clearest fit).
# --advance is the drive's current sserial0.pos_advance (template 0.0003).
# Repeat with the new value: a should come out near 0.
import argparse
import sys
import time

import linuxcnc
import numpy as np

ap = argparse.ArgumentParser(description="fit sserial0.pos_advance from LinuxCNC's following error")
ap.add_argument("--joint", type=int, default=0)
ap.add_argument("--advance", type=float, required=True, help="the drive's current sserial0.pos_advance [s]")
ap.add_argument("--time", type=float, default=60.0, help="logging time [s] when no --program is given")
ap.add_argument("--program", help="absolute path of a program to run in auto mode while logging")
ap.add_argument("--min-vel", type=float, default=0.0, help="fit only above this speed [units/s] (default 2 %% of the peak)")
args = ap.parse_args()

st = linuxcnc.stat()
cmd = linuxcnc.command()
st.poll()
if st.task_state != linuxcnc.STATE_ON:
    sys.exit("machine is not on")

rows, last = [], None


def sample():
    global last
    st.poll()
    j = st.joint[args.joint]
    v = (j["ferror_current"], j["velocity"])
    if v != last:  # stat updates at the task rate, keep only new values
        rows.append((time.time(),) + v)
        last = v


if args.program:
    if not all(st.homed[: st.joints]):
        sys.exit("not homed")
    cmd.mode(linuxcnc.MODE_AUTO)
    cmd.wait_complete()
    cmd.program_open(args.program)
    cmd.wait_complete()
    cmd.auto(linuxcnc.AUTO_RUN, 0)
    t0 = time.time()
    while time.time() - t0 < 0.5 or st.interp_state != linuxcnc.INTERP_IDLE:
        sample()
        time.sleep(0.0003)
        if time.time() - t0 > 600:
            sys.exit("program did not finish in 600 s")
else:
    t0 = time.time()
    while time.time() - t0 < args.time:
        sample()
        time.sleep(0.0003)

if len(rows) < 100:
    sys.exit(f"only {len(rows)} samples: did the joint move?")
a = np.array(rows)
t, fe, ve = a[:, 0], a[:, 1], a[:, 2]
acc = np.gradient(ve, t)
vmin = args.min_vel or 0.02 * np.abs(ve).max()
m = np.abs(ve) > vmin
if m.sum() < 50:
    sys.exit("too few moving samples")
X = np.c_[ve[m], acc[m], np.ones(m.sum())]
co, *_ = np.linalg.lstsq(X, fe[m], rcond=None)
res = fe[m] - X @ co
k, b, c = co

print(f"joint {args.joint}: {len(a)} samples ({len(a) / (t[-1] - t[0]):.0f}/s), {m.sum()} moving, peak vel {np.abs(ve).max():.2f}")
print(f"fit: ferror = {k * 1e6:+.1f} us * vel {b * 1e6:+.2f} us^2 * acc {c:+.6f}, residual rms {res.std():.6f}")
cr = m & (np.abs(acc) < 0.05 * np.abs(acc).max())
for sgn in (1, -1):
    s = cr & (np.sign(ve) == sgn)
    if s.sum():
        print(f"cruise {'+' if sgn > 0 else '-'}: vel {ve[s].mean():+.2f}, ferror mean {fe[s].mean():+.6f} sd {fe[s].std():.6f}")
print(f"ferror moving: rms {np.sqrt((fe[m] ** 2).mean()):.6f}, max {np.abs(fe).max():.6f}")
new = args.advance + k
print(f"sserial0.pos_advance = {new:.5f}  # was {args.advance:.5f}")
if abs(k) < 20e-6:
    print("change under 20 us: inside the run to run scatter, keep the current value")
if new < 0 or new > 0.002:
    print("warning: outside 0..2 ms, check the joint's scale and the feedback before using it")

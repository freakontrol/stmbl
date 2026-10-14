# Spindle bench configs, 13 Oct 2026 (Fanuc a2, IM06B50GC1 board)

From a clean config (hardware, plate, safety only) plus tool output, no hand tuning:
idacim (+ pole/direction test) -> id_sys (encoder) -> conf0.max_force = acim_foc0.t_max
-> id_pid speed mode. Firmware: idacim_rot_tr df22b814.

- spindle_clean_2026-10-13.txt: the start point
- spindle_auto_2026-10-13.txt: encoder FOC (vel_bw 79.66 from id_pid speed mode), saved on the drive
- spindle_foc_sl_auto_2026-10-13.txt: sensorless FOC (encoder linked for monitoring only), vel_bw 59.85
- spindle_vf_auto_2026-10-13.txt: V/f open loop (vf0.k_vel 1 with vel_src 1)
- spindle_vf_enc_auto_2026-10-13.txt: V/f with encoder slip control

All ran 0 -> +-838 rad/s both directions without a fault. The ID chain and its
commands are in the bench handover (Kira, captures/handover_kira_2026-10-13.md).

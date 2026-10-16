#!/usr/bin/env python3
"""Plot a stmbl_sim trace: one panel per signal, shared time axis.

usage: plot.py trace.csv [-o out.png] [-s signal,signal,...] [--from t0] [--to t1]
Needs matplotlib. Without -o it opens a window.
"""
import argparse
import csv

import matplotlib

ap = argparse.ArgumentParser()
ap.add_argument('csv')
ap.add_argument('-o', '--out')
ap.add_argument('-s', '--signals', help='comma separated, default all')
ap.add_argument('--from', dest='t0', type=float, default=None)
ap.add_argument('--to', dest='t1', type=float, default=None)
a = ap.parse_args()
if a.out:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt

with open(a.csv) as f:
    rows = list(csv.reader(f))
head, data = rows[0], [[float(x) for x in r] for r in rows[1:]]
data = [r for r in data if (a.t0 is None or r[0] >= a.t0) and (a.t1 is None or r[0] <= a.t1)]
names = a.signals.split(',') if a.signals else head[1:]
cols = [head.index(n) for n in names]
t = [r[0] for r in data]

fig, axs = plt.subplots(len(cols), 1, sharex=True, figsize=(10, 1.6 * len(cols) + 1), squeeze=False)
for ax, c in zip(axs[:, 0], cols):
    ax.plot(t, [r[c] for r in data], lw=0.8)
    ax.set_ylabel(head[c], rotation=0, ha='right', va='center', fontsize=8)
    ax.grid(True, lw=0.3)
axs[-1, 0].set_xlabel('t [s]')
fig.suptitle(a.csv, fontsize=9)
fig.tight_layout()
if a.out:
    fig.savefig(a.out, dpi=110)
else:
    plt.show()

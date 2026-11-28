"""Empirical survival curves for 1 kHz PID inter-start jitter."""
import pathlib
import gzip
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter
import numpy as np

here=pathlib.Path(__file__).resolve().parent
fig, axes=plt.subplots(1,2,figsize=(10.8,4.3),sharey=True,constrained_layout=True)
palette={"base":"#292b45","pr1":"#007b68","pr2":"#b93c65"}
for ax,mode in zip(axes,["inline","async"]):
    for rev,label in [("base","Base"),("pr1","PR 1"),("pr2","PR 2")]:
        jitters=[]
        folder=here/"pid-results"
        for trial in sorted(folder.glob(f"{rev}-{mode}-*.txt")) or sorted(folder.glob(f"{rev}-{mode}-*.txt.gz")):
            data=gzip.decompress(trial.read_bytes()).decode() if trial.suffix==".gz" else trial.read_text()
            if "PARTITION " not in data: continue
            starts=np.array([int(s) for s in re.findall(r"^TICK \d+ (\d+) \d+$",data,re.M)])
            jitters.extend(abs(np.diff(starts)-1_000_000)/1000)
        assert len(jitters)==89_997
        vals=np.sort(np.asarray(jitters))
        # Limit SVG size while retaining more points at the sparse tail.
        ranks=np.unique(np.geomspace(1,len(vals),1500).astype(int))
        ax.step(vals[-ranks],ranks/len(vals),where="post",color=palette[rev],label=label,lw=1.7)
    ax.set(xscale="log",yscale="log",xlim=(.1,300000),ylim=(.00001,1),title=f"{mode.capitalize()} output",xlabel="Absolute start-to-start jitter (µs)")
    ax.set_xticks([.1,1,10,100,1000,10000,100000],["0.1","1","10","100","1k","10k","100k"])
    ax.set_yticks([1,.1,.01,.001,.0001,.00001])
    ax.yaxis.set_major_formatter(PercentFormatter(1,decimals=3))
    ax.axvline(1000,color="#666666",ls=":",lw=.9)
    ax.grid(which="major",alpha=.16)
    ax.legend(frameon=False,fontsize=9)
axes[0].set_ylabel("Fraction of intervals at or above jitter")
fig.suptitle("1 kHz PID on SD  ·  3 × 30,000 ticks per configuration",fontsize=14)
fig.savefig(here/"pid-jitter.svg",metadata={"Date":None})

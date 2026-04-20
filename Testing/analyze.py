"""
ESP32 Channel Hopper — Statistical Analysis
============================================
Reads the CSV files produced by experiment.py and computes
paper-quality statistics comparing Round-Robin vs Adaptive hopping
across all tested cycle times.
 
Outputs:
  analysis/summary_stats.json      — machine-readable summary
  analysis/paper_table.txt         — LaTeX-ready table
  analysis/channel_bias_report.txt — per-channel dwell distribution analysis
  analysis/figures/                — matplotlib charts (PNG + SVG)
 
Metrics computed
----------------
1.  Packets per second (PPS) — primary throughput metric
2.  Capture efficiency ratio — adaptive / round-robin PPS
3.  Gini coefficient of channel dwell time — measures how aggressively
    the algorithm concentrates dwell on productive channels
4.  Shannon entropy of dwell allocation — theoretical upper bound on
    information gain per second
5.  Coefficient of variation (CV) of per-channel PPS — convergence
    indicator; lower CV means the algorithm is smoothing out traffic
6.  z-score correlation analysis — validates that high z-score channels
    receive proportionally more dwell time
7.  Per-channel capture contribution — fraction of total packets
    attributable to each channel
8.  Warm-up convergence — cycles until dwell allocation stabilises
    (Euclidean distance to final allocation < ε)
9.  Dwell standard deviation over time — stability measure
10. Bootstrap 95% CI on all key metrics (1000 resamples)
"""
 
import csv
import json
import math
import random
import statistics
import os
import sys
from collections import defaultdict
from typing import List, Dict, Tuple
 
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False
    print("matplotlib not found — skipping figure generation. "
          "Install with: pip install matplotlib")
 
NUM_CHANNELS = 13
 
 
# ─── I/O helpers ─────────────────────────────────────────────────────────────
 
def load_cycles(path: str) -> List[Dict]:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            r["ts"]              = float(r["ts"])
            r["cycle_time_ms"]   = int(r["cycle_time_ms"])
            r["cycle_num"]       = int(r["cycle_num"])
            r["cycle_total"]     = int(r["cycle_total"])
            r["all_time_total"]  = int(r["all_time_total"])
            r["packets_per_sec_cycle"] = float(r["packets_per_sec_cycle"])
            if r["global_mean"]:
                r["global_mean"]   = float(r["global_mean"])
                r["global_std_dev"] = float(r["global_std_dev"])
            rows.append(r)
    return rows
 
def load_channels(path: str) -> List[Dict]:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            r["ts"]            = float(r["ts"])
            r["cycle_time_ms"] = int(r["cycle_time_ms"])
            r["cycle_num"]     = int(r["cycle_num"])
            r["ch"]            = int(r["ch"])
            if r["packets"]:   r["packets"]  = int(r["packets"])
            if r["pps"]:       r["pps"]      = float(r["pps"])
            if r["avg_pps"]:   r["avg_pps"]  = float(r["avg_pps"])
            if r["dwell_ms"]:  r["dwell_ms"] = int(r["dwell_ms"])
            if r["z_score"]:   r["z_score"]  = float(r["z_score"])
            rows.append(r)
    return rows
 
 
# ─── Statistical primitives ───────────────────────────────────────────────────
 
def mean(xs): return sum(xs) / len(xs) if xs else 0.0
def variance(xs):
    if len(xs) < 2: return 0.0
    m = mean(xs); return sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
def stdev(xs): return math.sqrt(variance(xs))
def median(xs): return statistics.median(xs) if xs else 0.0
def cv(xs): m = mean(xs); return stdev(xs) / m if m else 0.0
 
def gini(values):
    """Gini coefficient of a list of non-negative values."""
    v = sorted(max(x, 0) for x in values)
    n = len(v); s = sum(v)
    if s == 0 or n == 0: return 0.0
    rank_sum = sum((2*i - n - 1) * vi for i, vi in enumerate(v, 1))
    return rank_sum / (n * s)
 
def shannon_entropy(values):
    """Shannon entropy (bits) of a discrete distribution."""
    s = sum(values)
    if s == 0: return 0.0
    probs = [v / s for v in values if v > 0]
    return -sum(p * math.log2(p) for p in probs)
 
def bootstrap_ci(data, stat_fn, n_boot=1000, alpha=0.05):
    """Bootstrap confidence interval for stat_fn applied to data."""
    if len(data) < 2: return (stat_fn(data), stat_fn(data))
    boot = []
    for _ in range(n_boot):
        sample = [random.choice(data) for _ in data]
        boot.append(stat_fn(sample))
    boot.sort()
    lo = boot[int(alpha/2 * n_boot)]
    hi = boot[int((1 - alpha/2) * n_boot)]
    return lo, hi
 
def pearson(xs, ys):
    """Pearson correlation coefficient."""
    n = len(xs)
    if n < 2: return 0.0
    mx, my = mean(xs), mean(ys)
    num = sum((x - mx)*(y - my) for x, y in zip(xs, ys))
    den = math.sqrt(sum((x-mx)**2 for x in xs) * sum((y-my)**2 for y in ys))
    return num / den if den else 0.0
 
 
# ─── Core analysis ───────────────────────────────────────────────────────────
 
def group_by_phase(cycles: List[Dict]) -> Dict[str, List[Dict]]:
    g = defaultdict(list)
    for r in cycles: g[r["phase"]].append(r)
    return dict(g)
 
def phase_key(phase: str):
    """Sort key: RR first, then AD by cycle time."""
    if "rr" in phase: return (0, 0)
    ms = int(phase.split("_")[-1].replace("ms", ""))
    return (1, ms)
 
def compute_phase_stats(cycles: List[Dict]) -> Dict:
    pps_vals = [r["packets_per_sec_cycle"] for r in cycles]
    total_pkts = sum(r["cycle_total"] for r in cycles)
    n_cycles   = len(cycles)
    mode       = cycles[0]["mode"]
    cycle_ms   = cycles[0]["cycle_time_ms"]
 
    ci_lo, ci_hi = bootstrap_ci(pps_vals, mean)
 
    stats = {
        "mode":           mode,
        "cycle_time_ms":  cycle_ms,
        "n_cycles":       n_cycles,
        "total_packets":  total_pkts,
        "mean_pps":       round(mean(pps_vals), 4),
        "median_pps":     round(median(pps_vals), 4),
        "std_pps":        round(stdev(pps_vals), 4),
        "cv_pps":         round(cv(pps_vals), 4),
        "ci95_lo":        round(ci_lo, 4),
        "ci95_hi":        round(ci_hi, 4),
    }
 
    if mode == "ad":
        if cycles[0].get("global_mean"):
            gm_vals = [r["global_mean"] for r in cycles if r.get("global_mean")]
            stats["mean_global_mean"]   = round(mean(gm_vals), 4)
            stats["mean_global_stddev"] = round(
                mean([r["global_std_dev"] for r in cycles if r.get("global_std_dev")]), 4)
 
    return stats
 
def compute_channel_stats(chan_rows: List[Dict], phase: str) -> List[Dict]:
    by_ch = defaultdict(list)
    for r in chan_rows:
        if r["phase"] == phase:
            by_ch[r["ch"]].append(r)
    result = []
    for ch in range(1, NUM_CHANNELS + 1):
        rows = by_ch.get(ch, [])
        mode = rows[0]["mode"] if rows else ""
        if mode == "rr":
            pps_vals   = [r["pps"] for r in rows if r.get("pps")]
            dwell_vals = [r["dwell_ms"] for r in rows if r.get("dwell_ms")]
        else:
            pps_vals   = [r["avg_pps"] for r in rows if r.get("avg_pps")]
            dwell_vals = [r["dwell_ms"] for r in rows if r.get("dwell_ms")]
        result.append({
            "ch":           ch,
            "mean_pps":     round(mean(pps_vals), 4),
            "mean_dwell_ms": round(mean(dwell_vals), 1),
            "total_packets": sum(r["packets"] for r in rows if r.get("packets")),
        })
    return result
 
def convergence_analysis(chan_rows: List[Dict], phase: str) -> Dict:
    """
    Measure how many cycles it takes for dwell allocation to stabilise.
    Metric: Euclidean distance between current dwell vector and final
    dwell vector (mean of last 10 cycles). Returns cycle index when
    distance first drops below 5% of max possible distance.
    """
    if "rr" in phase:
        return {"convergence_cycle": 0, "note": "RR is static — always converged"}
 
    by_cycle = defaultdict(lambda: [0]*NUM_CHANNELS)
    for r in chan_rows:
        if r["phase"] == phase and r.get("dwell_ms"):
            by_cycle[r["cycle_num"]][r["ch"]-1] = r["dwell_ms"]
 
    cycles_sorted = sorted(by_cycle.keys())
    if len(cycles_sorted) < 10:
        return {"convergence_cycle": None, "note": "Insufficient data"}
 
    # Final allocation = mean of last 10 cycles
    final_vec = [0.0] * NUM_CHANNELS
    for cyc in cycles_sorted[-10:]:
        for i, v in enumerate(by_cycle[cyc]):
            final_vec[i] += v / 10.0
 
    cycle_ms = 0
    for r in chan_rows:
        if r["phase"] == phase and r.get("cycle_time_ms"):
            cycle_ms = r["cycle_time_ms"]; break
 
    max_dist = math.sqrt(NUM_CHANNELS * (cycle_ms ** 2))
    threshold = 0.05 * max_dist
 
    for cyc in cycles_sorted:
        vec = by_cycle[cyc]
        dist = math.sqrt(sum((v - f)**2 for v, f in zip(vec, final_vec)))
        if dist < threshold:
            return {"convergence_cycle": cyc,
                    "threshold_pct": 5,
                    "note": f"Dwell allocation within 5% of final by cycle {cyc}"}
 
    return {"convergence_cycle": None,
            "note": "Did not converge within experiment window"}
 
def zscore_dwell_correlation(chan_rows: List[Dict], phase: str) -> float:
    """Pearson r between z-score and dwell_ms across all cycles."""
    zs, ds = [], []
    for r in chan_rows:
        if r["phase"] == phase and r.get("z_score") and r.get("dwell_ms"):
            zs.append(r["z_score"])
            ds.append(r["dwell_ms"])
    return round(pearson(zs, ds), 4) if zs else 0.0
 
def gini_per_phase(chan_rows: List[Dict], phase: str) -> Dict:
    """Gini coefficient of dwell allocation, averaged across cycles."""
    by_cycle = defaultdict(list)
    for r in chan_rows:
        if r["phase"] == phase and r.get("dwell_ms"):
            by_cycle[r["cycle_num"]].append(r["dwell_ms"])
    ginis = [gini(v) for v in by_cycle.values() if v]
    entropies = [shannon_entropy(v) for v in by_cycle.values() if v]
    return {
        "mean_gini":    round(mean(ginis), 4),
        "std_gini":     round(stdev(ginis), 4),
        "mean_entropy": round(mean(entropies), 4),
        "max_entropy":  round(math.log2(NUM_CHANNELS), 4),
    }
 
 
# ─── Report generators ────────────────────────────────────────────────────────
 
def build_latex_table(phase_order, phase_stats, rr_mean_pps):
    """Generate a LaTeX tabular suitable for direct inclusion in a paper."""
    lines = [
        r"\begin{table}[htbp]",
        r"\centering",
        r"\caption{Packet capture throughput: round-robin vs.\ adaptive channel hopping}",
        r"\label{tab:hopper-results}",
        r"\begin{tabular}{lrrrrrr}",
        r"\toprule",
        r"Mode & $T$ (s) & $N$ & Mean PPS & Median PPS & Std PPS & 95\% CI \\",
        r"\midrule",
    ]
    for phase in phase_order:
        s = phase_stats[phase]
        mode_str = "Round-Robin" if s["mode"] == "rr" else "Adaptive"
        T = s["cycle_time_ms"] / 1000
        ci = f"[{s['ci95_lo']:.2f}, {s['ci95_hi']:.2f}]"
        line = (f"{mode_str} & {T:.0f} & {s['n_cycles']} & "
                f"{s['mean_pps']:.2f} & {s['median_pps']:.2f} & "
                f"{s['std_pps']:.2f} & {ci} \\\\")
        lines.append(line)
    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)
 
def build_text_report(phase_order, phase_stats, chan_gini,
                      convergence, corr, rr_phase):
    lines = ["=" * 70,
             "  ESP32 ADAPTIVE CHANNEL HOPPER — ANALYSIS REPORT",
             "=" * 70, ""]
 
    rr_pps = phase_stats[rr_phase]["mean_pps"] if rr_phase else None
 
    lines.append("THROUGHPUT SUMMARY")
    lines.append("-" * 40)
    for phase in phase_order:
        s = phase_stats[phase]
        mode = "Round-Robin" if s["mode"] == "rr" else "Adaptive"
        T    = s["cycle_time_ms"] / 1000
        ratio = (s["mean_pps"] / rr_pps) if rr_pps else 1.0
        lines.append(
            f"  {mode:<14} T={T:.0f}s | "
            f"Mean PPS: {s['mean_pps']:>8.2f}  "
            f"CI95: [{s['ci95_lo']:.2f}, {s['ci95_hi']:.2f}]  "
            f"Ratio vs RR: {ratio:.3f}x"
        )
    lines.append("")
 
    lines.append("DWELL DISTRIBUTION (Gini / Shannon Entropy)")
    lines.append("-" * 40)
    for phase in phase_order:
        g = chan_gini.get(phase, {})
        s = phase_stats[phase]
        mode = "Round-Robin" if s["mode"] == "rr" else "Adaptive"
        T    = s["cycle_time_ms"] / 1000
        lines.append(
            f"  {mode:<14} T={T:.0f}s | "
            f"Gini: {g.get('mean_gini', 0):.4f}  "
            f"Entropy: {g.get('mean_entropy', 0):.4f} bits "
            f"(max {g.get('max_entropy', math.log2(NUM_CHANNELS)):.4f})"
        )
    lines.append("")
 
    lines.append("CONVERGENCE ANALYSIS (Adaptive phases only)")
    lines.append("-" * 40)
    for phase, result in convergence.items():
        s = phase_stats[phase]
        T = s["cycle_time_ms"] / 1000
        cyc = result.get("convergence_cycle")
        lines.append(
            f"  T={T:.0f}s | Converged at cycle: "
            f"{'N/A' if cyc is None else cyc}  "
            f"({result.get('note','')})"
        )
    lines.append("")
 
    lines.append("z-SCORE ↔ DWELL CORRELATION (Adaptive phases)")
    lines.append("-" * 40)
    for phase, r_val in corr.items():
        s = phase_stats[phase]
        T = s["cycle_time_ms"] / 1000
        lines.append(f"  T={T:.0f}s | Pearson r = {r_val:.4f}")
    lines.append("")
 
    return "\n".join(lines)
 
 
# ─── Figures ──────────────────────────────────────────────────────────────────
 
def plot_pps_boxplot(phase_order, cycles_by_phase, phase_stats, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    labels, data = [], []
    for phase in phase_order:
        s = phase_stats[phase]
        mode = "RR" if s["mode"] == "rr" else f"AD-{s['cycle_time_ms']//1000}s"
        labels.append(mode)
        data.append([r["packets_per_sec_cycle"] for r in cycles_by_phase[phase]])
    bp = ax.boxplot(data, patch_artist=True, notch=True)
    colors = ["#4C72B0"] + ["#DD8452","#55A868","#C44E52"][: len(labels)-1]
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c); patch.set_alpha(0.7)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylabel("Packets per second", fontsize=11)
    ax.set_title("Throughput distribution: Round-Robin vs Adaptive (all cycle times)")
    ax.grid(axis="y", alpha=0.4)
    plt.tight_layout()
    plt.savefig(f"{out_dir}/pps_boxplot.png", dpi=150)
    plt.savefig(f"{out_dir}/pps_boxplot.svg")
    plt.close()
    print("  Saved: pps_boxplot.png / .svg")
 
def plot_dwell_heatmap(chan_rows, phase_order, phase_stats, out_dir):
    fig, axes = plt.subplots(1, len(phase_order), figsize=(4*len(phase_order), 4),
                              sharey=True)
    if len(phase_order) == 1: axes = [axes]
    for ax, phase in zip(axes, phase_order):
        s = phase_stats[phase]
        mode = "RR" if s["mode"] == "rr" else f"AD-{s['cycle_time_ms']//1000}s"
        by_ch = defaultdict(list)
        for r in chan_rows:
            if r["phase"] == phase and r.get("dwell_ms"):
                by_ch[r["ch"]].append(r["dwell_ms"])
        means = [mean(by_ch.get(ch, [0])) for ch in range(1, NUM_CHANNELS+1)]
        ax.barh(range(1, NUM_CHANNELS+1), means, color="#4C72B0", alpha=0.75)
        ax.set_xlabel("Mean dwell (ms)")
        ax.set_title(mode, fontsize=10)
        ax.set_yticks(range(1, NUM_CHANNELS+1))
        ax.set_yticklabels([str(c) for c in range(1, NUM_CHANNELS+1)], fontsize=8)
        ax.grid(axis="x", alpha=0.3)
    axes[0].set_ylabel("Channel")
    fig.suptitle("Mean dwell allocation per channel")
    plt.tight_layout()
    plt.savefig(f"{out_dir}/dwell_heatmap.png", dpi=150)
    plt.savefig(f"{out_dir}/dwell_heatmap.svg")
    plt.close()
    print("  Saved: dwell_heatmap.png / .svg")
 
def plot_convergence(chan_rows, ad_phases, phase_stats, out_dir):
    fig, axes = plt.subplots(1, len(ad_phases), figsize=(5*len(ad_phases), 4),
                              sharey=True)
    if len(ad_phases) == 1: axes = [axes]
    for ax, phase in zip(axes, ad_phases):
        s = phase_stats[phase]
        T = s["cycle_time_ms"] // 1000
        by_cycle_ch = defaultdict(lambda: [0]*NUM_CHANNELS)
        for r in chan_rows:
            if r["phase"] == phase and r.get("dwell_ms"):
                by_cycle_ch[r["cycle_num"]][r["ch"]-1] = r["dwell_ms"]
        cycles_sorted = sorted(by_cycle_ch.keys())
        if not cycles_sorted: continue
        final = [mean([by_cycle_ch[c][i] for c in cycles_sorted[-10:]])
                 for i in range(NUM_CHANNELS)]
        dists = [math.sqrt(sum((by_cycle_ch[c][i]-final[i])**2
                               for i in range(NUM_CHANNELS)))
                 for c in cycles_sorted]
        ax.plot(cycles_sorted, dists, color="#DD8452")
        ax.axhline(y=0.05 * math.sqrt(NUM_CHANNELS * (s["cycle_time_ms"]**2)),
                   color="red", linestyle="--", alpha=0.6, label="5% threshold")
        ax.set_xlabel("Cycle number"); ax.set_title(f"AD T={T}s convergence")
        ax.set_ylabel("Euclidean dist. to final allocation")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"{out_dir}/convergence.png", dpi=150)
    plt.savefig(f"{out_dir}/convergence.svg")
    plt.close()
    print("  Saved: convergence.png / .svg")
 
def plot_efficiency_ratio(phase_stats, rr_phase, ad_phases, out_dir):
    rr_pps = phase_stats[rr_phase]["mean_pps"]
    labels  = [f"AD-{phase_stats[p]['cycle_time_ms']//1000}s" for p in ad_phases]
    ratios  = [phase_stats[p]["mean_pps"] / rr_pps for p in ad_phases]
    ci_lo   = [phase_stats[p]["ci95_lo"] / rr_pps for p in ad_phases]
    ci_hi   = [phase_stats[p]["ci95_hi"] / rr_pps for p in ad_phases]
    yerr    = [[r - lo for r, lo in zip(ratios, ci_lo)],
               [hi - r for r, hi in zip(ratios, ci_hi)]]
    fig, ax = plt.subplots(figsize=(6, 4))
    x = range(len(labels))
    ax.bar(x, ratios, color=["#DD8452","#55A868","#C44E52"][:len(labels)],
           alpha=0.75, yerr=yerr, capsize=5, error_kw={"elinewidth":1.5})
    ax.axhline(y=1.0, color="black", linestyle="--", alpha=0.5, label="RR baseline")
    ax.set_xticks(list(x)); ax.set_xticklabels(labels)
    ax.set_ylabel("Capture efficiency ratio (vs RR)")
    ax.set_title("Adaptive capture efficiency relative to Round-Robin")
    ax.legend(); ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"{out_dir}/efficiency_ratio.png", dpi=150)
    plt.savefig(f"{out_dir}/efficiency_ratio.svg")
    plt.close()
    print("  Saved: efficiency_ratio.png / .svg")
 
 
# ─── Main ─────────────────────────────────────────────────────────────────────
 
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Analyze hopper experiment data")
    parser.add_argument("--data",  default="data",     help="Input data directory")
    parser.add_argument("--out",   default="analysis", help="Output directory")
    parser.add_argument("--seed",  type=int, default=42, help="Random seed for bootstrap")
    args = parser.parse_args()
 
    random.seed(args.seed)
    os.makedirs(args.out, exist_ok=True)
    fig_dir = f"{args.out}/figures"
    if HAVE_MPL: os.makedirs(fig_dir, exist_ok=True)
 
    print("Loading data...")
    cycles   = load_cycles(f"{args.data}/cycles.csv")
    chan_rows = load_channels(f"{args.data}/channels.csv")
 
    cycles_by_phase = group_by_phase(cycles)
    phase_order = sorted(cycles_by_phase.keys(), key=phase_key)
    print(f"  Phases found: {phase_order}")
 
    print("\nComputing per-phase statistics...")
    phase_stats = {}
    for phase in phase_order:
        phase_stats[phase] = compute_phase_stats(cycles_by_phase[phase])
 
    rr_phases = [p for p in phase_order if phase_stats[p]["mode"] == "rr"]
    ad_phases  = [p for p in phase_order if phase_stats[p]["mode"] == "ad"]
    rr_phase   = rr_phases[0] if rr_phases else None
    rr_pps     = phase_stats[rr_phase]["mean_pps"] if rr_phase else None
 
    print("Computing Gini / entropy of dwell allocation...")
    chan_gini = {phase: gini_per_phase(chan_rows, phase) for phase in phase_order}
 
    print("Computing convergence analysis...")
    convergence = {phase: convergence_analysis(chan_rows, phase)
                   for phase in ad_phases}
 
    print("Computing z-score ↔ dwell correlation...")
    corr = {phase: zscore_dwell_correlation(chan_rows, phase)
            for phase in ad_phases}
 
    # Build full summary
    summary = {
        "phase_stats": phase_stats,
        "dwell_distribution": chan_gini,
        "convergence": convergence,
        "zscore_dwell_correlation": corr,
    }
    if rr_pps:
        summary["efficiency_ratios"] = {
            phase: round(phase_stats[phase]["mean_pps"] / rr_pps, 4)
            for phase in ad_phases
        }
 
    # Write JSON
    with open(f"{args.out}/summary_stats.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nWrote: {args.out}/summary_stats.json")
 
    # Write LaTeX table
    with open(f"{args.out}/paper_table.tex", "w", encoding="utf-8") as f:
        f.write(build_latex_table(phase_order, phase_stats, rr_pps))
    print(f"Wrote: {args.out}/paper_table.tex")
 
    # Write text report
    report = build_text_report(phase_order, phase_stats, chan_gini,
                                convergence, corr, rr_phase)
    with open(f"{args.out}/analysis_report.txt", "w", encoding="utf-8") as f:
        f.write(report)
    print(f"Wrote: {args.out}/analysis_report.txt")
    print("\n" + report)
 
    # Generate figures
    if HAVE_MPL:
        print("\nGenerating figures...")
        plot_pps_boxplot(phase_order, cycles_by_phase, phase_stats, fig_dir)
        plot_dwell_heatmap(chan_rows, phase_order, phase_stats, fig_dir)
        if ad_phases:
            plot_convergence(chan_rows, ad_phases, phase_stats, fig_dir)
        if rr_phase and ad_phases:
            plot_efficiency_ratio(phase_stats, rr_phase, ad_phases, fig_dir)
        print(f"Figures saved to: {fig_dir}/")
    else:
        print("Skipping figures (matplotlib not installed)")
 
    print("\nAnalysis complete.")
 
 
if __name__ == "__main__":
    main()
 

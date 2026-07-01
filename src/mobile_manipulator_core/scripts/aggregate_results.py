#!/usr/bin/env python3
"""
aggregate_results.py — Phase 6 Part 2: reads experiment_runner's CSV output
and produces a summary table + figures for the paper.

Usage:
    ~/aset_ws/venv/bin/python3 aggregate_results.py \
        --csv ~/aset_ws/experiment_logs/experiment_results.csv \
        --out ~/aset_ws/experiment_logs/figures

Outputs:
    summary_table.csv      — success rate + mean time per (scenario, system)
    success_rate.png/.pdf  — bar chart, LLM vs baseline, per scenario
    disturbance.png/.pdf   — normal (S1) vs disturbance (S4) comparison
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')  # headless -- this runs from a terminal, no display needed
import matplotlib.pyplot as plt
import pandas as pd


def load(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df['success'] = df['success'].astype(bool)
    return df


def summary_table(df: pd.DataFrame) -> pd.DataFrame:
    grouped = df.groupby(['scenario', 'system']).agg(
        n_trials=('trial', 'count'),
        success_rate=('success', 'mean'),
        mean_time_sec=('total_time_sec', 'mean'),
        mean_path_length_m=('path_length_m', 'mean'),
        mean_num_llm_calls=('num_llm_calls', 'mean'),
        mean_num_replans=('num_replans', 'mean'),
    ).reset_index()
    grouped['success_rate'] = (grouped['success_rate'] * 100).round(1)
    for col in ('mean_time_sec', 'mean_path_length_m', 'mean_num_llm_calls', 'mean_num_replans'):
        grouped[col] = grouped[col].round(2)
    return grouped


def plot_success_rate(summary: pd.DataFrame, out_dir: str) -> None:
    pivot = summary.pivot(index='scenario', columns='system', values='success_rate').fillna(0)
    ax = pivot.plot(kind='bar', figsize=(8, 5), rot=0)
    ax.set_ylabel('Success rate (%)')
    ax.set_xlabel('Scenario')
    ax.set_title('Success rate: LLM planner vs baseline controller')
    ax.set_ylim(0, 105)
    ax.legend(title='System')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'success_rate.png'), dpi=150)
    plt.savefig(os.path.join(out_dir, 'success_rate.pdf'))
    plt.close()


def plot_disturbance_comparison(df: pd.DataFrame, summary: pd.DataFrame, out_dir: str) -> None:
    subset = summary[summary['scenario'].isin(['S1', 'S4'])]
    if subset.empty:
        print('No S1/S4 data found -- skipping disturbance comparison plot.')
        return
    pivot = subset.pivot(index='scenario', columns='system', values='success_rate').fillna(0)
    ax = pivot.plot(kind='bar', figsize=(7, 5), rot=0)
    ax.set_ylabel('Success rate (%)')
    ax.set_xlabel('Scenario (S1 = normal, S4 = mid-task disturbance)')
    ax.set_title('Effect of mid-task disturbance')
    ax.set_ylim(0, 105)
    ax.legend(title='System')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'disturbance.png'), dpi=150)
    plt.savefig(os.path.join(out_dir, 'disturbance.pdf'))
    plt.close()

    s4 = df[(df['scenario'] == 'S4') & (df['system'] == 'llm') & df['success']]
    if not s4.empty and s4['time_to_recover_sec'].notna().any():
        mean_recover = s4['time_to_recover_sec'].mean()
        print(f"Mean time_to_recover for successful LLM S4 trials: {mean_recover:.1f}s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default=os.path.expanduser('~/aset_ws/experiment_logs/experiment_results.csv'))
    parser.add_argument('--out', default=os.path.expanduser('~/aset_ws/experiment_logs/figures'))
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    df = load(args.csv)
    print(f"Loaded {len(df)} trial rows from {args.csv}")

    summary = summary_table(df)
    summary_path = os.path.join(args.out, 'summary_table.csv')
    summary.to_csv(summary_path, index=False)
    print(f"\nSummary table:\n{summary.to_string(index=False)}")
    print(f"\nSaved: {summary_path}")

    plot_success_rate(summary, args.out)
    plot_disturbance_comparison(df, summary, args.out)
    print(f"Saved figures to {args.out}")


if __name__ == '__main__':
    main()

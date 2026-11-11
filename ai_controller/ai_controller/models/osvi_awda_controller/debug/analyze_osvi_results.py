#!/usr/bin/env python3
"""Analizza i risultati OSVI-AWDA in task_00..task_15 / traj_000..traj_011.

Esempio:
 python3 analyze_osvi_results.py --root /percorso/pick_place --out /percorso/risultati

Impostare FIELD_ALIASES dopo aver confrontato i nomi con un JSON reale.
"""
import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

TRAIN = {1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14}
TEST = {0, 5, 10, 15}
METRICS = ('pick', 'total_success', 'wrong_object_correct_bin', 'wrong_bin')
# Le varianti qui sotto sono proposte; lo script rifiuta silenziose equivalenze non certe.
# Se i JSON usano altri nomi, aggiungerli alla metrica corrispondente.
FIELD_ALIASES = {
    'pick': ('object_picked',),
    'wrong_object_correct_bin': ('place_wrong_correct_bin',),
    'wrong_bin': ('place_wrong_wrong_bin',),
}
REQUIRED_TOTAL_FIELDS = ('object_picked', 'object_placed', 'picked_wrong',
                         'place_wrong_correct_bin', 'place_wrong_wrong_bin')



def normalize_key(key):
    return re.sub(r'[^a-z0-9]+', '_', str(key).lower()).strip('_')


def flatten(obj, prefix=''):
    if not isinstance(obj, dict):
        return []
    out = []
    for key, val in obj.items():
        name = (prefix + '.' if prefix else '') + str(key)
        if isinstance(val, dict):
            out.extend(flatten(val, name))
        else:
            out.append((name, val))
    return out


def binary(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (int, float)) and value in (0, 1):
        return int(value)
    if isinstance(value, str):
        return {'true': 1, 'false': 0, 'yes': 1, 'no': 0,
                'success': 1, 'failure': 0, '0': 0, '1': 1}.get(value.strip().lower())
    return None


def extract(obj, metric):
    if metric == 'total_success':
        missing = [k for k in REQUIRED_TOTAL_FIELDS if k not in obj]
        if missing:
            return None, 'MISSING for total_success: ' + ', '.join(missing)
        values = {k: binary(obj[k]) for k in REQUIRED_TOTAL_FIELDS}
        invalid = [k for k, v in values.items() if v is None]
        if invalid:
            return None, 'INVALID for total_success: ' + ', '.join(invalid)
        # Successo pieno: presa corretta, oggetto posizionato e nessun evento wrong.
        success = (values['object_picked'] == 1 and
                   values['object_placed'] == 1 and
                   values['picked_wrong'] == 0 and
                   values['place_wrong_correct_bin'] == 0 and
                   values['place_wrong_wrong_bin'] == 0)
        return int(success), 'derived from object_picked/object_placed and wrong flags'
    flat = flatten(obj)
    matches = []
    aliases = set(FIELD_ALIASES[metric])
    for key, value in flat:
        if normalize_key(key.split('.')[-1]) in aliases:
            result = binary(value)
            if result is not None:
                matches.append((key, result))
    if not matches:
        return None, 'MISSING'
    distinct = {v for _, v in matches}
    if len(distinct) > 1:
        return None, 'CONFLICT: ' + repr(matches)
    return matches[0][1], matches[0][0]



def stats(values):
    n = len(values)
    if not n:
        return {'n': 0, 'sum': '', 'mean': '', 'std': ''}
    mu = sum(values) / n
    sigma = math.sqrt(sum((v - mu)**2 for v in values) / n)  # ddof=0
    return {'n': n, 'sum': sum(values), 'mean': mu, 'std': sigma}


def fmt(x):
    return 'N/A' if x == '' else f'{x:.2f}'


def write_csv(path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=Path('/home/asus-mivia/Desktop/UR-Control/UR5e-2f-85/ai_controller/saved_rollouts/osvi_awda_controller/pick_place'))
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--traj-start', type=int, default=0,
                help='Prima traiettoria da analizzare (default: 0)')
    ap.add_argument('--traj-end', type=int, default=3,
                    help='Ultima traiettoria da analizzare inclusa (default: 3)')
    args = ap.parse_args()
    if args.traj_start < 0 or args.traj_end < args.traj_start:
        ap.error('Intervallo delle traiettorie non valido')
    root = args.root.expanduser()
    if not root.is_dir():
        ap.error(f'Directory inesistente: {root}')
    out = (args.out or root / 'results_analysis').expanduser()
    out.mkdir(parents=True, exist_ok=True)

    records, issues = [], []
    seen = set()
    for path in sorted(root.rglob('traj_*.json')):
        rel = path.relative_to(root)
        # Seleziona file SOLO immediatamente dentro task_XX, scartando altro in subdir.
        if len(rel.parts) != 2:
            continue
        task_match = re.fullmatch(r'task_(\d{2})', rel.parts[0])
        traj_match = re.fullmatch(r'traj_(\d{3})\.json', rel.parts[1])
        if not (task_match and traj_match):
            continue
        t, j = int(task_match.group(1)), int(traj_match.group(1))
        if not (0 <= t <= 15 and args.traj_start <= j <= args.traj_end):
            continue
        seen.add((t, j))
        try:
            with path.open(encoding='utf-8') as f:
                obj = json.load(f)
        except (OSError, ValueError) as e:
            issues.append({'file': str(path), 'metric': '*', 'problem': f'JSON ERROR: {e}'})
            continue
        record = {'split': 'train' if t in TRAIN else 'test', 'task': f'task_{t:02d}',
                  'trajectory': f'traj_{j:03d}', 'file': str(path)}
        for metric in METRICS:
            value, source = extract(obj, metric)
            record[metric] = '' if value is None else value
            if value is None:
                available = ', '.join(k for k, _ in flatten(obj))
                issues.append({'file': str(path), 'metric': metric,
                               'problem': f'{source}; available keys: {available}'})
        records.append(record)

    for t in range(16):
        for j in range(args.traj_start, args.traj_end + 1):
            if (t, j) not in seen:
                issues.append({
                    'file': str(root / f'task_{t:02d}' / f'traj_{j:03d}.json'),
                    'metric': '*',
                    'problem': 'FILE MISSING'
                })
    write_csv(out / 'trajectories.csv', records, ['split', 'task', 'trajectory', *METRICS, 'file'])
    task_rows, split_rows = [], []
    for task in range(16):
        subset = [r for r in records if r['task'] == f'task_{task:02d}']
        row = {'split': 'train' if task in TRAIN else 'test', 'task': f'task_{task:02d}',
               'n_files': len(subset)}
        for metric in METRICS:
            s = stats([r[metric] for r in subset if r[metric] != ''])
            for k, v in s.items():
                row[f'{metric}_{k}'] = v
        task_rows.append(row)

    for split in ('train', 'test', 'all'):
        selected_tasks = [r for r in task_rows if split == 'all' or r['split'] == split]
        selected_records = [r for r in records if split == 'all' or r['split'] == split]
        for metric in METRICS:
            # Macro: media dei task; std delle medie dei task, non somma delle std dei task.
            task_means = [r[f'{metric}_mean'] for r in selected_tasks if r[f'{metric}_mean'] != '']
            macro = stats(task_means)
            # Micro: risultati di tutte le traiettorie aggregati (pesati dal numero di rollout).
            micro = stats([r[metric] for r in selected_records if r[metric] != ''])
            split_rows.append({'split': split, 'metric': metric, 'n_tasks': macro['n'],
                               'n_trajectories': micro['n'], 'total_successes': micro['sum'],
                               'macro_mean': macro['mean'], 'macro_std_between_tasks': macro['std'],
                               'micro_mean': micro['mean'], 'micro_std_trajectories': micro['std']})

    task_fields = ['split', 'task', 'n_files'] + [f'{metric}_{k}' for metric in METRICS for k in ('n','sum','mean','std')]
    write_csv(out / 'per_task.csv', task_rows, task_fields)
    write_csv(out / 'train_test_summary.csv', split_rows, list(split_rows[0]))
    write_csv(out / 'warnings.csv', issues, ['file', 'metric', 'problem'])
    expected_trajectories = 16 * (args.traj_end - args.traj_start + 1)

    lines = ['OSVI-AWDA — analisi risultati',
             f'Root: {root}', f'Traiettorie JSON caricate: {len(records)}/{expected_trajectories}',
             'Deviazione standard: popolazione (ddof=0)',
             'Valori in [0,1]. Moltiplicare le medie e std per 100 per ottenere percentuali.',
             'Macro = media delle medie task, std = deviazione tra le medie task.',
             'Micro = media e std di tutti i rollout nello split.',
             'Le metriche mancanti non vengono interpretate come fallimenti (0).', '']
    for split in ('train', 'test', 'all'):
        lines.append(f'===== {split.upper()} =====')
        for r in split_rows:
            if r['split'] != split:
                continue
            lines.append(f"{r['metric']}: successi={r['total_successes']}, n={r['n_trajectories']}, "
                         f"macro={fmt(r['macro_mean'])} ± {fmt(r['macro_std_between_tasks'])}, "
                         f"micro={fmt(r['micro_mean'])} ± {fmt(r['micro_std_trajectories'])}")
        lines.append('')
    lines.append('===== PER TASK (media ± std) =====')
    for r in task_rows:
        parts = [f"{m}={fmt(r[f'{m}_mean'])} ± {fmt(r[f'{m}_std'])} "
                 f"(n={r[f'{m}_n']}, successi={r[f'{m}_sum']})" for m in METRICS]
        lines.append(f"{r['task']} [{r['split']}]: " + '; '.join(parts))
    lines.extend(['', f'Problemi / campi non riconosciuti / file mancanti: {len(issues)}',
                  'Controlla warnings.csv prima di usare i risultati.'])
    report = '\n'.join(lines) + '\n'
    (out / 'report.txt').write_text(report, encoding='utf-8')
    print(report)
    print(f'CSV e report salvati in: {out}')

if __name__ == '__main__':
    main()

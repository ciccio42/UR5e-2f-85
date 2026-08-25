# Action Head Tests

Questo test valuta la pipeline Mimic Video completa sul dataset UR5e:

1. prende una traiettoria `processed_data/ur5e_pick_place_action/ep_*.safetensors`;
2. usa 5 frame RGB consecutivi come contesto;
3. usa lo stato robotico dell'ultimo frame come osservazione lowdim;
4. esegue Cosmos V2W fine-tuned + action head W2A;
5. ripete la predizione ogni 15 frame, cioe ogni 1.5 secondi a 10 fps;
6. di default si ferma al primo rilascio GT (`chiuso -> aperto`);
7. plotta la ground truth completa e tutti i chunk predetti generati fino al rilascio;
8. salva overlay PNG, chunk `.npz`, `metrics.csv` e `summary.json`.

Il default dell'action head e:

```bash
checkpoints/posttraining/world2action/w2a_ur5e_videmb_v2w_ur5e_finetuned_lr1.000e-04_layer20_bsz1_accum12_train/checkpoints/model/iter_000010000.pt
```

Esempio dentro l'ambiente/container Mimic Video gia configurato:

```bash
python /workspace/mimic_video_workspace/test/action/test_action_head.py \
  --episode ep_000000 \
  --plane xy
```

Per testare un checkpoint diverso:

```bash
python /workspace/mimic_video_workspace/test/action/test_action_head.py \
  --episode ep_000000 \
  --action-model-path /workspace/mimic_video_workspace/checkpoints/posttraining/world2action/.../iter_000010000.pt
```

Gli output finiscono in:

```bash
mimic_video_workspace/test/action/outputs/<checkpoint_stem>/<episode_stem>/
```

Il file principale e:

```bash
ep_XXXXXX_trajectory_xy.png
```

Nell'overlay:

- GT open: linea azzurra
- GT closed: linea verde
- Pred open: linea arancione
- Pred closed: linea magenta

I marker etichettati `GT open`, `GT closed`, `Pred open`, `Pred closed` indicano i punti in cui il gripper cambia stato.

Nota: l'overlay e un grafico diagnostico in coordinate robotiche (`xy`, `xz` o `yz`) disegnato sopra un frame della traiettoria. Non e una proiezione camera calibrata.

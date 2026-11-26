# Patch minimale — Timing di generazione action chunk VLA-JEPA

Questa patch e' costruita ESATTAMENTE sui file inviati il 26 novembre 2026.
Funziona sia con `action_head` (Qwen + DiT) sia con `wm_conditioned` (Qwen + V-JEPA2 + Predictor + DiT).

## File toccati

- `vla_jepa.py` — cronometro installato su `policy.predict_action_chunk()` dopo il load e prima delle inferenze.
- `vla_jepa_wm.py` — inoltro del nuovo parametro al runtime base (necessario alla sottoclasse WM).
- `vla_jepa_controller.py` — lettura YAML e inoltro al runtime scelto.
- `vla_jepa_config.yaml` e `vla_jepa_config_wm_conditioned.yaml` — nuova chiave `log_chunk_inference_time: true`.

Gli altri file **non vanno modificati**. Le correzioni di grasp, fixed orientation, gripper, conversione delta, processor LeRobot e coda sono preservati.

## Che cosa misura

La misura racchiude la chiamata effettiva a `predict_action_chunk()` con `time.perf_counter()` e `torch.cuda.synchronize(device)` all'inizio e alla fine. Quindi cronometra il tempo reale GPU necessario alla produzione di un nuovo chunk e il relativo overhead interno alla funzione. Le richieste che leggono azioni gia' accodate non fanno comparire un messaggio. Un chunk standard contiene sette azioni.

Non include: preprocessing delle immagini nel controller o nel preprocessor LeRobot, POSTPROCESSING, codifica e trasporto HTTP, MoveIt, acquisizione delle camere o tempo di esecuzione del robot.

Esempio di log sul terminale che avvia `./run_server.sh`:

```text
[VLAJEPA Chunk Timing] chunk=1 actions=7 model_forward=2.348s (2348.0ms) device=cuda
```

Il valore e' un esempio inventato del formato, non una misura reale. Il contatore riparte da 1 dopo `reset()` o `load_command()`.

Per disattivarlo: `log_chunk_inference_time: false` (o rimuovi la chiave). Riavvia il server dopo cambiamenti YAML.

## Installazione Spark (host, server fermo)

Lo ZIP contiene il percorso relativo `ai_controller/ai_controller/models/vla_jepa_controller/`: estrarlo nella radice del repository, **non** nella directory del controller. Da host:

```bash
REPO="/home/asus-mivia/Desktop/Alex/UR5e-2f-85"
CTRL="$REPO/ai_controller/ai_controller/models/vla_jepa_controller"
ZIP="$CTRL/vla_jepa_chunk_timing_patch.zip"
mkdir -p "$CTRL/backup_chunk_timing"
for f in vla_jepa.py vla_jepa_wm.py vla_jepa_controller.py vla_jepa_config.yaml vla_jepa_config_wm_conditioned.yaml; do
  cp -a "$CTRL/$f" "$CTRL/backup_chunk_timing/$f"
done
unzip -t "$ZIP" && unzip -o "$ZIP" -d "$REPO"
```

Riavvia `./run_server.sh --config vla_jepa_config_wm_conditioned.yaml` o con lo YAML baseline. Nessuna modifica al nodo ROS e nessuna reinstallazione LeRobot.

## Note

- La prima inferenza puo' includere warm-up/JIT/cache: confronta anche le misure dei chunk successivi.
- Su CUDA la sincronizzazione e' fondamentale per non misurare solo il lancio asincrono dei kernel.
- Ogni log appare a generazione del chunk, non per le 6 letture successive dalla coda.
- Il timer e' installato dinamicamente sull'istanza di policy e **non** modifica i parametri del modello o lo state_dict.

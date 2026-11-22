# Patch minimale VLA-JEPA WM Conditioned per UR5e

Questa e' una versione **delta** del package completo: contiene SOLO i file originali modificati (6, incluso lo YAML baseline con il nuovo blocco commentato), i nuovi moduli per WM Conditioned (5) e questo README. Non contiene file originali invariati (`server.py`, `vla_jepa_utils.py`, `ai_controller_node.py`).

## Compatibilita' baseline

Se nel YAML esistente `inference_mode` e' assente, il default e' `action_head`. Il flusso standard Qwen -> DiT, processor LeRobot, conversione scala 0.05, camera crop e gripper restano invariati. La compatibilita' con un checkpoint richiede gli stessi formati di input e i processor previsti dalla vecchia codebase. I checkpoint di training esportati in pretrained_model devono essere montati al path dello YAML.

## Due modi per selezionare il runtime

Opzione A: mantenere lo YAML baseline invariato e usare il nuovo file `vla_jepa_config_wm_conditioned.yaml` per WM.

Opzione B: utilizzare **un solo file YAML** (`vla_jepa_config.yaml`) e modificare esclusivamente:

```yaml
inference_mode: action_head      # baseline
# oppure
inference_mode: wm_conditioned   # nuovo esperimento
inference_timeout_sec: 180.0     # consigliato per WM, opzionale per baseline
```

`run_server.sh` e `vla_jepa_controller.py` leggono entrambi lo stesso `inference_mode`; il client legge `inference_timeout_sec`. Nel passaggio tra le modalita' occorre anche montare il checkpoint corrispondente in `/opt/vla_jepa/checkpoint`. Riavviare il server dopo le modifiche. Non avviare contemporaneamente due server sulla stessa porta.

## Installazione

Arrestare server e nodo ROS; fare un backup della cartella del controller. Quindi, **dalla radice** del repository `UR5e-2f-85`:

```bash
REPO=/home/asus-mivia/Desktop/UR-Control/UR5e-2f-85
CTRL="$REPO/ai_controller/ai_controller/models/vla_jepa_controller"
cp -a "$CTRL" "${CTRL}_backup_before_wm_$(date +%Y%m%d_%H%M%S)"
unzip -o /percorso/vla_jepa_wm_controller_minimal_patch.zip -d "$REPO"
cd "$CTRL"
./run_server.sh --config vla_jepa_config.yaml --preflight-only
```

Per WM, selezionare `wm_conditioned` nel YAML e montare il checkpoint corretto; poi eseguire il preflight con `--preflight-only` prima del deployment.

## Limiti

Il pacchetto e' stato testato localmente tramite test CPU. Il forward con i pesi WM reali e CUDA sulla Spark deve ancora essere verificato. Non eseguire movimenti robotici prima del preflight e di un dry-run con `move_robot=false`.

Per le shape, i dettagli architetturali, e la procedura completa di validazione consultare il README dello ZIP integrale `vla_jepa_wm_controller_extension.zip`.

## Orientamento fisso opzionale per entrambi i runtime

In **entrambi** `vla_jepa_config.yaml` e `vla_jepa_config_wm_conditioned.yaml`,
alla fine del file, sotto la sezione `tasks`, e' incluso il blocco commentato:

```yaml
# fixed_orientation_xyzw:
#   - 0.9994452044624775
#   - 0.03161651380119412
#   - 0.0021438049655468088
#   - -0.010251021036213035
```

- Default: commentato => il controller usa il quaternione calcolato dalla delta-action predetta.
- Attivazione: decommentare **tutte e cinque le righe** e riavviare il server.
- Il quaternione e' nell'ordine ROS/SciPy `[qx, qy, qz, qw]`, nel frame `base_link`; non e' RPY.
- Il controller valida 4 numeri finiti e norma ~1 all'avvio. Valori non validi bloccano l'avvio.
- In `post_process()`, la sostituzione e' eseguita **per ogni singola azione** subito prima di costruire `[x,y,z,qx,qy,qz,qw,gripper]` restituito al nodo ROS.
- XYZ, offset Y/Z sul grasp, gripper, queue, preprocessor e checkpoint non cambiano.
- I preflight baseline/WM controllano anche il quaternione del target quando il flag e' attivo.

**Nota:** la modifica riguarda il target di orientamento; non sostituisce la validazione di collisioni, limiti cinematica e sicurezza del robot. Prima di muovere il robot controllare un target in dry-run con `move_robot=false`.

Il vecchio archivio `vla_jepa_wm_controller_minimal_patch.zip` e' **superato** dalla versione `vla_jepa_wm_controller_minimal_patch_fixed_orientation.zip`.

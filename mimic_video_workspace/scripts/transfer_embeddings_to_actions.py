#!/usr/bin/env python3
"""Add precomputed text and video embeddings to UR5e action safetensors."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import safetensors.numpy as st_np
import safetensors.torch as st_torch
import torch


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
PROCESSED_DIR = WORKSPACE_DIR / "processed_data"
DEFAULT_ACTION_DIR = PROCESSED_DIR / "ur5e_pick_place_action"
DEFAULT_LANGUAGE_EMBEDDINGS_DIR = PROCESSED_DIR / "ur5e_pick_place_video" / "language_embeddings"
DEFAULT_VIDEO_EMBEDDINGS_DIR = PROCESSED_DIR / "ur5e_pick_place_video" / "video_embeddings"
DEFAULT_VIDEO_SUFFIX = "0"
NS_PER_SEC = 1_000_000_000


def language_embedding_path(action_path: Path, language_embeddings_dir: Path) -> Path:
    # ep_000000.safetensors usa l'embedding testuale ep_000000.safetensors.
    return language_embeddings_dir / action_path.name


def video_embedding_path(action_path: Path, video_embeddings_dir: Path, video_suffix: str) -> Path:
    # Per l'action head usiamo la front camera: ep_000000.safetensors -> ep_0000000.safetensors.
    return video_embeddings_dir / f"{action_path.stem}{video_suffix}.safetensors"


def read_language_embedding(path: Path) -> np.ndarray:
    # Gli autori salvano encoded_text come tensore torch; lo porto a float32 numpy e aggiungo la dimensione episodio.
    encoded_text = st_torch.load_file(path)["encoded_text"]
    return np.ascontiguousarray(encoded_text.to(torch.float32).cpu().numpy()[None])


def video_embedding_timestamps(action_data: dict[str, np.ndarray], video_data: dict[str, np.ndarray]) -> np.ndarray:
    # Gli embedding video sono associati agli indici ancora usati dallo script di precompute.
    # Li riallineo ai timestamp di workspace_rgb, gestendo anche gli indici oltre la fine del video.
    
    video_len = int(video_data["video_len"]) #numero di frame della traiettoria
    fps = float(video_data["fps"])
    idxs = video_data["video_embeddings_idxs"].astype(np.int64) #indici dei frame utilizzati come anchor nel calcolo degli embeddings
    # questi indici possono anche eccedere i frame reali del video.
    # questo può accadere perchè per ogni anchor frame, 4 frame precedenti vengono utilizzati come 
    # osservazione, e su una finestra di 56 frame successivi all'anchor viene calcolato l'embedding.
    # Al massimo l'indice quindi può superare di 4 la lunghezza totale della traiettoria, l'ultimo frame è visto come contesto e l'embedding viene calcolato sulla ripetizione del frame terminale.
    # Cosmos dovrà produrre predizioni latenti quanto più simili agli embedding calcolati sulla traiettoria vera.

    # ogni step ha un timestamp sintetico calcolato dalla pipeline di preprocess. 
    # Il calcolo considera che l'intera sequenza è a 10 fps, quindi i frame si sussegiono ogni 0.1 secondi.
    # Lo step inziale ha timestamp 0, il primo dopo 0.1 secondi ha timestamp 0.1, il secondo 0.2 e così via. 
    workspace_timestamps = action_data["workspace_rgb_timestamps"]
    # in realtà i timestamps sono riportati in nanosecondi, quindi:
    # [0, 200_000_000, 400_000_000, 500_000_000, 700_000_000]
    
    # la lunghezza del video calcolata dallo script video embedding, deve corrispondere con la lunghezza della traiettoria memorizzata nel safetensor dell'action
    # L'embedding è stato calcolato su quel video. Le lunghezze devono corrispondere.
    if len(action_data["workspace_rgb"]) != video_len:
        raise ValueError(
            f"Video/action length mismatch: workspace_rgb={len(action_data['workspace_rgb'])}, video_len={video_len}"
        )

    # fase di allineamento tra i timestamp degli action safetensor e video safetensor.
    # Ex: 
    #frame:      0    1    2    3    4
    #timestamp: 0.0  0.1  0.2  0.3  0.4 secondi = workspace_timestamps
    #idxs = [0, 2, 4, 5, 7] anchor scelti dal video encoder
    # video_len - 1 = 4 = ultimo indice valido
    padding_idx = idxs - (video_len - 1) 
    #padding_idx: -4  -2   0   1   3
    #padding_idx < 0  -> anchor interno al video
    #padding_idx = 0  -> ultimo frame reale
    #padding_idx = 1  -> un frame oltre la fine
    #padding_idx = 3  -> tre frame oltre la fine

    # np.clip pone a 0 i padding_idx negativi
    # workspace_timestamps[-1] è 0.4
    extrapolated = (
        workspace_timestamps[-1] + NS_PER_SEC * np.clip(padding_idx, 0, None) / fps
    ).astype(np.uint64)
    # extrapolated: 0.4 0.4 0.4 0.5 0.7

    # ora si lavora sui timestamps della traiettoria
    # si selezionano solo i timestamps indicati dagli idxs, clippando al valore massimo quelli che lo superano.
    # idxs:             0   2   4   5   7
    # indici limitati:  0   2   4   4   4 questo accade con clip
    aligned = workspace_timestamps[np.clip(idxs, 0, video_len - 1)]
    #aligned: 0.0  0.2  0.4  0.4  0.4 secondi

    # si restituisce extrapolated per gli anchor oltre il video
    # timestamp reale altrimenti
    return np.where(padding_idx > 0, extrapolated, aligned).astype(np.uint64)
    # return 0.0 0.2 0.4 0.5 0.7 (in nanosecondi)
    # nel caso di timestamps sintetici calcolati in base agli fps tutto questo poteva essere
    # return indxs / fps * NS_PER_SEC


def add_embeddings(
    action_path: Path,
    language_embeddings_dir: Path,
    video_embeddings_dir: Path,
    video_suffix: str,
    overwrite: bool,
) -> str:
    # Carico il safetensor action, aggiungo gli embedding mancanti e risalvo lo stesso file.
    action_data = st_np.load_file(action_path)
    has_language = "language_embedding" in action_data
    has_video = "workspace_rgb_embedding" in action_data

    if has_language and has_video and not overwrite:
        return f"skip: {action_path.name}"

    if overwrite or not has_language:
        lang_path = language_embedding_path(action_path, language_embeddings_dir)
        if not lang_path.exists():
            raise FileNotFoundError(lang_path)
        action_data["language_embedding"] = read_language_embedding(lang_path)
        action_data["language_embedding_timestamps"] = np.array([0], dtype=np.uint64)

    if overwrite or not has_video:
        vid_path = video_embedding_path(action_path, video_embeddings_dir, video_suffix)
        if not vid_path.exists():
            raise FileNotFoundError(vid_path)
        video_data = st_np.load_file(vid_path)
        action_data["workspace_rgb_embedding"] = np.ascontiguousarray(video_data["video_embeddings"])
        action_data["workspace_rgb_embedding_timestamps"] = video_embedding_timestamps(action_data, video_data)
        
        # num_conditional_frames indica il numero di frame latenti che condizionano la predizione.
        # sono 5 frame RGB incluso l'anchor frame, ma che risultano in 2 frame latenti a causa della compressione eseguita dal tokenizer VAE
        action_data["num_conditional_frames"] = np.array([int(video_data["num_conditional_frames"])])
        action_data["num_conditional_frames_timestamps"] = np.array([0], dtype=np.uint64)
        #Video RGB:
        #    [5 frame osservati | 56 frame futuri]

        #Latente:
        #    [2 condizionanti | 14 da predire]

    st_np.save_file(action_data, action_path)
    return f"ok: {action_path.name}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-dir", type=Path, default=DEFAULT_ACTION_DIR)
    parser.add_argument("--language-embeddings-dir", type=Path, default=DEFAULT_LANGUAGE_EMBEDDINGS_DIR)
    parser.add_argument("--video-embeddings-dir", type=Path, default=DEFAULT_VIDEO_EMBEDDINGS_DIR)
    parser.add_argument("--video-suffix", default=DEFAULT_VIDEO_SUFFIX)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    action_paths = sorted(args.action_dir.glob("ep_*.safetensors"))
    if not action_paths:
        raise FileNotFoundError(f"No action safetensors found in {args.action_dir}")

    print(f"action_dir={args.action_dir.resolve()}")
    print(f"language_embeddings_dir={args.language_embeddings_dir.resolve()}")
    print(f"video_embeddings_dir={args.video_embeddings_dir.resolve()}")
    print(f"video_suffix={args.video_suffix}")

    for action_path in action_paths:
        print(add_embeddings(
            action_path,
            args.language_embeddings_dir,
            args.video_embeddings_dir,
            args.video_suffix,
            args.overwrite,
        ))

    print(f"done: {len(action_paths)} action safetensors")


if __name__ == "__main__":
    main()




# Verifica dell'aggiornamento degli action safetensor da terminale dopo l'esecuzione.
'''
python - <<'PY'
from pathlib import Path

import numpy as np
from safetensors.numpy import load_file

path = Path(
    "mimic_video_workspace/processed_data/"
    "ur5e_pick_place_action/ep_000000.safetensors"
)

data = load_file(path)

required = [
    "language_embedding",
    "language_embedding_timestamps",
    "workspace_rgb_embedding",
    "workspace_rgb_embedding_timestamps",
    "num_conditional_frames",
    "num_conditional_frames_timestamps",
]

print(f"\nFile: {path.resolve()}")
print(f"Dimensione: {path.stat().st_size / 1024**2:.2f} MiB")

print("\n========== CHIAVI EMBEDDING ==========")
for key in required:
    if key not in data:
        print(f"MISSING  {key}")
        continue

    value = data[key]
    print(f"OK       {key:42s} shape={value.shape}, dtype={value.dtype}")

missing = [key for key in required if key not in data]
if missing:
    raise RuntimeError(f"Chiavi mancanti: {missing}")

video_embeddings = data["workspace_rgb_embedding"]
video_timestamps = data["workspace_rgb_embedding_timestamps"]

assert len(video_embeddings) == len(video_timestamps), (
    f"Embedding/timestamp non allineati: "
    f"{len(video_embeddings)} != {len(video_timestamps)}"
)

assert len(data["language_embedding"]) == len(
    data["language_embedding_timestamps"]
)

assert len(data["num_conditional_frames"]) == len(
    data["num_conditional_frames_timestamps"]
)

assert np.all(np.diff(video_timestamps.astype(np.int64)) >= 0), (
    "I timestamp degli embedding video non sono ordinati"
)

print("\n========== VALORI ==========")
print(f"Numero embedding video:    {len(video_embeddings)}")
print(f"Timestamp embedding:       {video_timestamps.tolist()}")
print(
    "Num conditional frames:  "
    f"{data['num_conditional_frames'].tolist()}"
)
print(
    "Timestamp testo:         "
    f"{data['language_embedding_timestamps'].tolist()}"
)
print(
    "Timestamp metadata:      "
    f"{data['num_conditional_frames_timestamps'].tolist()}"
)

print("\nControllo completato correttamente.")
PY
'''
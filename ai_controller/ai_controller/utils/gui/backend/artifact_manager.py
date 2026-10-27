from __future__ import annotations

import json
import mimetypes

from pathlib import Path
from threading import Lock

import numpy as np


IMAGE_SUFFIXES = {
    ".png",
    ".jpg",
    ".jpeg",
    ".webp",
    ".gif",
}

VIDEO_SUFFIXES = {
    ".mp4",
    ".webm",
    ".mov",
}

TEXT_SUFFIXES = {
    ".txt",
    ".log",
    ".yaml",
    ".yml",
    ".md",
    ".csv",
    ".sh",
    ".py",
}

ARRAY_SUFFIXES = {
    ".npy",
    ".npz",
}


class ArtifactManager:

    def __init__(self):
        self._lock = Lock()
        self._root: Path | None = None


    def clear(self) -> None:
        with self._lock:
            self._root = None


    def set_root(
        self,
        path: str | Path,
    ) -> None:
        root = (
            Path(path)
            .expanduser()
            .resolve()
        )

        if not root.is_dir():
            raise ValueError(
                "Artifact root does not exist "
                f"or is not a directory: {root}"
            )

        with self._lock:
            self._root = root


    def get_root(self) -> Path | None:
        with self._lock:
            return self._root


    @staticmethod
    def _viewer_type(
        path: Path,
    ) -> str:
        suffix = path.suffix.lower()

        if suffix in IMAGE_SUFFIXES:
            return "image"

        if suffix in VIDEO_SUFFIXES:
            return "video"

        if suffix == ".json":
            return "json"

        if suffix in TEXT_SUFFIXES:
            return "text"

        if suffix in ARRAY_SUFFIXES:
            return "array"

        return "binary"


    def resolve_file(
        self,
        relative_path: str,
    ) -> Path:
        root = self.get_root()

        if root is None:
            raise RuntimeError(
                "No artifact directory is currently active."
            )

        candidate = (
            root
            / relative_path
        ).resolve()

        if (
            candidate != root
            and root not in candidate.parents
        ):
            raise ValueError(
                "Artifact path escapes the active run directory."
            )

        if not candidate.is_file():
            raise FileNotFoundError(
                f"Artifact does not exist: {relative_path}"
            )

        return candidate


    def _build_node(
        self,
        path: Path,
        root: Path,
    ) -> dict | None:

        try:
            relative_path = (
                path.relative_to(root)
                .as_posix()
            )

            if path.is_dir():

                children = []

                for child in sorted(
                    path.iterdir(),
                    key=lambda item: (
                        not item.is_dir(),
                        item.name.lower(),
                    ),
                ):
                    node = self._build_node(
                        child,
                        root,
                    )

                    if node is not None:
                        children.append(
                            node
                        )

                return {
                    "type": "directory",
                    "name": path.name,
                    "path": relative_path,
                    "children": children,
                }


            if not path.is_file():
                return None


            stat = path.stat()

            return {
                "type": "file",
                "name": path.name,
                "path": relative_path,
                "viewer": self._viewer_type(
                    path
                ),
                "size": stat.st_size,
                "modified_ns": stat.st_mtime_ns,
            }

        except FileNotFoundError:
            # A file may disappear/change while the
            # pipeline is writing its artifacts.
            return None


    @staticmethod
    def _count_files(
        node: dict,
    ) -> int:

        if node["type"] == "file":
            return 1

        return sum(
            ArtifactManager._count_files(
                child
            )
            for child in node.get(
                "children",
                []
            )
        )


    def tree(self) -> dict:
        root = self.get_root()

        if root is None:
            return {
                "ready": False,
                "root": None,
                "file_count": 0,
                "tree": None,
            }

        if not root.is_dir():
            return {
                "ready": False,
                "root": str(root),
                "file_count": 0,
                "tree": None,
            }

        tree = self._build_node(
            root,
            root,
        )

        return {
            "ready": True,
            "root": str(root),
            "root_name": root.name,
            "file_count": (
                self._count_files(tree)
                if tree is not None
                else 0
            ),
            "tree": tree,
        }


    def preview(
        self,
        relative_path: str,
    ) -> dict:

        path = self.resolve_file(
            relative_path
        )

        viewer = self._viewer_type(
            path
        )

        result = {
            "name": path.name,
            "path": relative_path,
            "viewer": viewer,
            "size": path.stat().st_size,
        }


        if viewer == "json":

            with path.open(
                "r",
                encoding="utf-8",
            ) as stream:
                data = json.load(
                    stream
                )

            result["text"] = json.dumps(
                data,
                indent=2,
                ensure_ascii=False,
            )

            return result


        if viewer == "text":

            with path.open(
                "r",
                encoding="utf-8",
                errors="replace",
            ) as stream:
                result["text"] = (
                    stream.read()
                )

            return result


        if viewer == "array":

            result["arrays"] = (
                self._describe_numpy_file(
                    path
                )
            )

            return result


        return result


    @staticmethod
    def _describe_array(
        name: str,
        array,
    ) -> dict:

        flat = array.reshape(-1)

        sample_count = min(
            int(flat.size),
            32,
        )

        sample = (
            flat[:sample_count]
            .tolist()
        )

        return {
            "name": name,
            "shape": list(
                array.shape
            ),
            "dtype": str(
                array.dtype
            ),
            "size": int(
                array.size
            ),
            "sample": sample,
        }


    def _describe_numpy_file(
        self,
        path: Path,
    ) -> list[dict]:

        if path.suffix.lower() == ".npy":

            array = np.load(
                path,
                mmap_mode="r",
                allow_pickle=False,
            )

            return [
                self._describe_array(
                    path.name,
                    array,
                )
            ]


        with np.load(
            path,
            allow_pickle=False,
        ) as archive:

            return [
                self._describe_array(
                    name,
                    archive[name],
                )
                for name
                in archive.files
            ]


    def media_type(
        self,
        relative_path: str,
    ) -> str:

        path = self.resolve_file(
            relative_path
        )

        mime_type, _ = (
            mimetypes.guess_type(
                path.name
            )
        )

        return (
            mime_type
            or "application/octet-stream"
        )


ARTIFACT_MANAGER = ArtifactManager()
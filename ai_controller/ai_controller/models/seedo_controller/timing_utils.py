import json
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import torch


class TimingRecorder:

    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self.timings = {}

    def _record(self, name, elapsed):
        with self._lock:
            self.timings.setdefault(name, []).append(float(elapsed))

        print(
            f"[TIMING] {name}: {elapsed:.6f} s"
        )

    def start(self):
        return time.perf_counter()

    def stop(self, name, start_time):
        elapsed = time.perf_counter() - start_time
        self._record(name, elapsed)
        return elapsed

    @contextmanager
    def measure(self, name, cuda=False):

        if cuda and torch.cuda.is_available():
            torch.cuda.synchronize()

        start = time.perf_counter()

        try:
            yield

        finally:
            if cuda and torch.cuda.is_available():
                torch.cuda.synchronize()

            elapsed = time.perf_counter() - start

            self._record(
                name,
                elapsed,
            )

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        summary = {}

        with self._lock:
            for name, samples in self.timings.items():
                summary[name] = {
                    "samples_s": samples,
                    "total_s": sum(samples),
                    "mean_s": sum(samples) / len(samples),
                    "count": len(samples),
                }

        with path.open(
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(
                summary,
                file,
                indent=2,
            )


TIMING = TimingRecorder()
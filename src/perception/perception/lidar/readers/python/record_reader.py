import os
import glob
import numpy as np


class NPZFrameReader:
    """
    Minimal NPZ reader that loads ALL fields
    and returns a structured NumPy array.
    """

    def __init__(self, data_dir: str = "/workspace/Perception/data", pattern: str = "*.npz", loop: bool = True):
        self.files = sorted(glob.glob(os.path.join(data_dir, pattern)))
        if not self.files:
            raise FileNotFoundError(f"No NPZ files found in {data_dir} with pattern {pattern}")

        self.idx = 0
        self.loop = loop

    def read(self) -> np.ndarray:
        if self.idx >= len(self.files):
            if self.loop:
                self.idx = 0
            else:
                raise StopIteration

        path = self.files[self.idx]
        self.idx += 1

        d = np.load(path, allow_pickle=False)

        arr = np.empty(len(d["x"]), dtype=[
            ("Frame", np.int32),
            ("Timestamp", "U32"),
            ("Epoch time (usec)", np.int64),
            ("Pixel Number", np.int32),
            ("Distance", np.float32),
            ("Reflectivity", np.float32),
            ("Probability of False", np.float32),
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("theta", np.float32),
            ("phi", np.float32),
        ])

        arr["Frame"] = d["Frame"]
        arr["Timestamp"] = d["Timestamp"].astype(str)
        arr["Epoch time (usec)"] = d["Epoch time (usec)"]
        arr["Pixel Number"] = d["Pixel Number"]
        arr["Distance"] = d["Distance"]
        arr["Reflectivity"] = d["Reflectivity"]
        arr["Probability of False"] = d["Probability of False"]
        arr["x"] = d["x"]
        arr["y"] = d["y"]
        arr["z"] = d["z"]
        arr["theta"] = d["theta"]
        arr["phi"] = d["phi"]

        return arr
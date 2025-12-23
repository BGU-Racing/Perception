import os
import glob
import numpy as np

class NPZFrameReader:
    """
    Reads Innoviz exported per-frame NPZ files.

    Each NPZ file contains (per your exporter):
      x, y, z, distance, reflectivity, pfa, confidence, theta, phi, valid,
      frame_id, timestamp_usec

    Usage:
      reader = NPZFrameReader("data/Recording_..._NPZ/PSM", pattern="Frame_*_PSM.npz")
      path, meta, points = reader.read_points_cloud()
    """

    def __init__(
        self,
        frames_dir: str,
        pattern: str = "Frame_*.npz",
        start_index: int = 0,
        frame_skip: int = 1,
        use_valid_mask: bool = True,
        pfa_max: float | None = None,
        confidence_min: int | None = None,
    ):
        self.frames_dir = frames_dir
        self.pattern = pattern
        self.frame_skip = max(1, int(frame_skip))
        self.use_valid_mask = use_valid_mask
        self.pfa_max = pfa_max
        self.confidence_min = confidence_min

        search = os.path.join(frames_dir, pattern)
        self.files = sorted(glob.glob(search))
        if not self.files:
            raise FileNotFoundError(f"No NPZ frames found: {search}")

        self.idx = int(start_index)

    def __len__(self):
        return len(self.files)

    def _load_npz(self, path: str):
        data = np.load(path, allow_pickle=False)

        # required keys
        x = data["x"]
        y = data["y"]
        z = data["z"]

        # optional keys (exist in your exporter)
        valid = data["valid"] if "valid" in data.files else None
        pfa = data["pfa"] if "pfa" in data.files else None
        conf = data["confidence"] if "confidence" in data.files else None

        frame_id = int(data["frame_id"]) if "frame_id" in data.files else None
        timestamp_usec = int(data["timestamp_usec"]) if "timestamp_usec" in data.files else None

        return x, y, z, valid, pfa, conf, frame_id, timestamp_usec

    def read_points_cloud(self):
        """
        Returns:
          (path, meta, points_np)

        meta = {"frame_id": ..., "timestamp_usec": ...}
        points_np shape: (N,3)
        """
        if self.idx >= len(self.files):
            raise StopIteration("No more frames")

        path = self.files[self.idx]
        self.idx += self.frame_skip

        x, y, z, valid, pfa, conf, frame_id, timestamp_usec = self._load_npz(path)

        mask = np.ones_like(x, dtype=bool)

        if self.use_valid_mask and valid is not None:
            mask &= valid

        if self.pfa_max is not None and pfa is not None:
            mask &= (pfa < self.pfa_max)

        if self.confidence_min is not None and conf is not None:
            mask &= (conf >= self.confidence_min)

        points = np.column_stack((x[mask], y[mask], z[mask])).astype(np.float32, copy=False)

        meta = {
            "frame_id": frame_id,
            "timestamp_usec": timestamp_usec,
            "num_points_raw": int(x.shape[0]),
            "num_points_filtered": int(points.shape[0]),
        }

        return path, meta, points


"""
Exports Innoviz lidar frames to NumPy (.npz) files.

Input:
- recording_path: path to Innoviz recording folder
- first_frame, last_frame: frame range to export
- refl:
    0 = PSM (summation)
    1 = reflection 1
    2 = reflection 2
    3 = reflection 3

Output:
- One .npz file per frame:
  <recording_path>_NPZ/<refl_type>/Frame_<frame_id>_<refl_type>.npz

Each .npz file contains:
- x, y, z           : point coordinates
- distance          : range (0 if invalid)
- reflectivity      : reflectivity (0 if invalid)
- pfa               : probability of false alarm
- confidence        : confidence flag
- theta, phi        : angles in degrees
- valid             : boolean mask for valid points
- frame_id          : frame number
- timestamp_usec    : timestamp in microseconds


Loading Example
---------------
```python
import numpy as np

data = np.load("Frame_42_PSM.npz")

x = data["x"]
y = data["y"]
z = data["z"]

theta = data["theta"]
phi = data["phi"]

valid = data["valid"]

# filter only valid points
points = np.column_stack((x[valid], y[valid], z[valid]))
"""

from __future__ import print_function
from datetime import datetime
import os
import numpy as np
from innopy.api import GrabType, FrameDataAttributes, FileReader

# -----------------------------
# configuration
# -----------------------------

recording_path = "/home/bgr/Desktop/InnovizPlayer_iOne_10.5.33_Linux/Recordings/Recording_2025_05_24_17_27_12"

first_frame = 0
last_frame = 1331

# reflection selector
# 0 = PSM (summation)
# 1 = reflection 1
# 2 = reflection 2
# 3 = reflection 3
refl = 0

refl_types = {
    0: "PSM",
    1: "Refl_1",
    2: "Refl_2",
    3: "Refl_3",
}

# -----------------------------
# grab attributes
# -----------------------------

attr = [
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0),
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1),
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2),
    FrameDataAttributes(GrabType.GRAB_TYPE_SUMMATION_REFLECTION0),
    FrameDataAttributes(GrabType.GRAB_TYPE_METADATA),
]

# -----------------------------
# output directories
# -----------------------------

np_output_root = recording_path + "_NPZ"
np_output_dir = os.path.join(np_output_root, refl_types[refl])
os.makedirs(np_output_dir, exist_ok=True)

# -----------------------------
# reader
# -----------------------------

fr = FileReader(recording_path)

# -----------------------------
# helpers
# -----------------------------

def parse_firmware_version(value):
    v = hex(value)[2:].zfill(6)
    return f"{int(v[0],16)}.{int(v[0:2],16)}.{int(v[-4:],16)}"

def print_metadata():
    res = fr.get_frame(1, attr)
    meta = res.results[str(GrabType.GRAB_TYPE_METADATA)][0]

    print("Firmware:", parse_firmware_version(meta["fw_version"]))
    print("Hardware:", parse_firmware_version(meta["hw_version"]))
    serial = "".join(chr(i) for i in meta["lidar_serial_number"])
    print("Serial:", serial)

# -----------------------------
# frame conversion
# -----------------------------

def convert_frame(frame_id):
    try:
        res = fr.get_frame(frame_id, attr)
        if not res.success:
            print(f"Frame {frame_id}: read failed")
            return
    except Exception as e:
        print(f"Frame {frame_id}: exception {e}")
        return

    print(f"Processing frame {frame_id}/{fr.num_of_frames}")

    # timestamp
    meta = res.results[str(GrabType.GRAB_TYPE_METADATA)][0]
    timestamp_usec = meta["timestamp_utc_secs"] * 1_000_000 + meta["timestamp_utc_micro"]
    if timestamp_usec <= 0:
        timestamp_usec = res.timestamp

    # select measurement block
    block_map = {
        0: GrabType.GRAB_TYPE_SUMMATION_REFLECTION0,
        1: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0,
        2: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1,
        3: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2,
    }

    block_key = str(block_map[refl])
    if block_key not in res.results:
        print(f"Frame {frame_id}: reflection block missing")
        return

    mes = res.results[block_key]
    if mes.size == 0:
        print(f"Frame {frame_id}: empty frame")
        return

    # -----------------------------
    # vectorized extraction
    # -----------------------------

    X = np.array([p["x"] for p in mes], dtype=np.float32)
    Y = np.array([p["y"] for p in mes], dtype=np.float32)
    Z = np.array([p["z"] for p in mes], dtype=np.float32)

    Dist = np.array([p["distance"] for p in mes], dtype=np.float32)
    Refl = np.array([p["reflectivity"] for p in mes], dtype=np.float32)
    PFA = np.array([p["pfa"] for p in mes], dtype=np.float32)
    Conf = np.array([p["confidence"] for p in mes], dtype=np.uint8)

    # -----------------------------
    # validity mask
    # -----------------------------

    valid = (Conf >= 1) & (Dist > 0)

    Dist = np.where(valid, Dist, 0.0)
    Refl = np.where(valid, Refl, 0.0)
    PFA = np.where(valid, PFA, 1.0)

    # -----------------------------
    # angles (degrees)
    # -----------------------------

    theta = np.degrees(np.arctan2(Y, X))
    phi = np.degrees(np.arctan2(Z, np.sqrt(X**2 + Y**2)))

    theta = np.where(valid, theta, 0.0)
    phi = np.where(valid, phi, 0.0)

    # -----------------------------
    # save NPZ
    # -----------------------------

    out_file = os.path.join(
        np_output_dir,
        f"Frame_{frame_id}_{refl_types[refl]}.npz"
    )

    np.savez_compressed(
        out_file,
        frame_id=frame_id,
        timestamp_usec=timestamp_usec,
        x=X,
        y=Y,
        z=Z,
        distance=Dist,
        reflectivity=Refl,
        pfa=PFA,
        confidence=Conf,
        theta=theta,
        phi=phi,
        valid=valid
    )

# -----------------------------
# main
# -----------------------------

def main():
    print_metadata()
    for frame_id in range(first_frame, last_frame + 1):
        convert_frame(frame_id)

if __name__ == "__main__":
    main()
"""
Exports Innoviz lidar frames to CSV files.

what you give it:
- recording_path: path to an innoviz recording folder
- first_frame, last_frame: which frames to export (inclusive)
- refl:
    0 = psm (summation reflection 0)
    1 = reflection 1 (measurements reflection0)
    2 = reflection 2 (measurements reflection1)
    3 = reflection 3 (measurements reflection2)

what it creates:
- a folder: <recording_path>_CSV/<refl_type>/
- one csv per frame: Frame_<frame_id>_<refl_type>.csv

each csv row is one "pixel/point" and includes:
frame id, timestamp, point index, distance, reflectivity, pfa, x,y,z, theta, phi

loading example (csv):
    import pandas as pd
    df = pd.read_csv("Frame_10_PSM.csv")
    pts = df[["x","y","z"]].to_numpy()
"""

from __future__ import print_function
from datetime import datetime
import csv
import os
import numpy as np
from innopy.api import GrabType, FrameDataAttributes, FileReader


# -----------------------------
# 1) what data we request per frame from the sdk
# -----------------------------
# we ask for: metadata + multiple reflection blocks.
# later we will choose one block to export based on `refl`.
attr = [
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0),
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1),
    FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2),
    FrameDataAttributes(GrabType.GRAB_TYPE_SUMMATION_REFLECTION0),
    FrameDataAttributes(GrabType.GRAB_TYPE_METADATA),
]

# just names for output folders/files
refl_types = {0: "PSM", 1: "Refl_1", 2: "Refl_2", 3: "Refl_3"}

# choose which reflection block to export
refl = 0  # change if needed


# -----------------------------
# 2) paths + output folders
# -----------------------------
recording_path = "/home/bgr/Desktop/InnovizPlayer_iOne_10.5.33_Linux/Recordings/Recording_2025_05_24_17_27_12"

# we export into: "<recording>_CSV/<refl_type>/"
csv_output_dir = recording_path + "_CSV"
refl_dir = os.path.join(csv_output_dir, refl_types[refl])
os.makedirs(refl_dir, exist_ok=True)


# -----------------------------
# 3) open recording
# -----------------------------
# FileReader lets us grab frames by frame index
fr = FileReader(recording_path)

# export range (inclusive)
first_frame = 0
last_frame = 1331


# -----------------------------
# 4) helper: parse fw/hw versions
# -----------------------------
def parse_firmware_version(value):
    """
    converts the fw/hw integer value into a readable string.

    note:
    this format is vendor-specific. it splits the integer into parts
    using hex positions (not standard semantic versioning).
    """
    version = hex(value)[2:].zfill(6)  # e.g. "01a2ff"
    return f"{int(version[0],16)}.{int(version[0:2],16)}.{int(version[-4:],16)}"


def metadata():
    """
    reads frame 1 metadata and prints:
    firmware version, hardware version, and lidar serial number.
    """
    res = fr.get_frame(1, attr)
    metadata_i = res.results[str(GrabType.GRAB_TYPE_METADATA)][0]

    print("Firmware version:", parse_firmware_version(metadata_i["fw_version"]))
    print("Hardware version:", parse_firmware_version(metadata_i["hw_version"]))

    # serial comes as list of ascii codes, so convert to string
    serial = "".join(chr(i) for i in metadata_i["lidar_serial_number"])
    print("Serial number:", serial)


# -----------------------------
# 5) export one frame to csv
# -----------------------------
def convert_frame(frame_id):
    """
    exports one frame into a csv file.

    important logic:
    - reads chosen reflection block (based on `refl`)
    - builds numpy arrays (fast math)
    - masks invalid points:
        valid = (confidence >= 1) and (distance > 0)
      invalid points are forced to:
        distance=0, reflectivity=0, pfa=1
    - computes:
        theta = atan2(y, x) in degrees
        phi   = atan2(z, sqrt(x^2+y^2)) in degrees
    """
    # read frame from recording
    try:
        res = fr.get_frame(frame_id, attr)
        if not res.success:
            print(f"Frame {frame_id}: read failure.")
            return
    except Exception as e:
        print(f"Frame {frame_id}: exception - {e}")
        return

    print(f"Processing Frame {frame_id}/{fr.num_of_frames}")

    results = res.results

    # get timestamp from metadata (utc seconds + utc microseconds)
    # fallback to sdk timestamp if metadata timestamp is not set
    meta = results[str(GrabType.GRAB_TYPE_METADATA)][0]
    timestamp = meta["timestamp_utc_secs"] * 1_000_000 + meta["timestamp_utc_micro"]
    timestamp = timestamp if timestamp > 0 else res.timestamp

    # this is for human-readable time inside the csv
    # note: fromtimestamp uses local timezone
    dt = datetime.fromtimestamp(timestamp / 1_000_000)

    # choose which data block we export
    block_map = {
        0: GrabType.GRAB_TYPE_SUMMATION_REFLECTION0,     # psm
        1: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0,   # reflection 1
        2: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1,   # reflection 2
        3: GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2,   # reflection 3
    }

    block_key = str(block_map.get(refl))
    if block_key not in results:
        print(f"Frame {frame_id}: reflection type {refl} not found.")
        return

    mes = results[block_key]
    if mes.size == 0:
        print(f"Frame {frame_id}: empty frame.")
        return

    # create csv file per frame
    output_file = os.path.join(refl_dir, f"Frame_{frame_id}_{refl_types[refl]}.csv")
    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)

        # header: one row per point afterwards
        writer.writerow([
            "Frame", "Timestamp", "Epoch time (usec)", "Pixel Number",
            "Distance", "Reflectivity", "Probability of False Alarm",
            "x", "y", "z", "theta", "phi"
        ])

        # extract fields into arrays so we can do fast vector math
        X = np.array([p["x"] for p in mes], dtype=np.float32)
        Y = np.array([p["y"] for p in mes], dtype=np.float32)
        Z = np.array([p["z"] for p in mes], dtype=np.float32)
        Dist = np.array([p["distance"] for p in mes], dtype=np.float32)
        Refl = np.array([p["reflectivity"] for p in mes], dtype=np.float32)
        PFA = np.array([p["pfa"] for p in mes], dtype=np.float32)
        Conf = np.array([p["confidence"] for p in mes], dtype=np.float32)

        # mark points as valid only if sensor says confidence>=1 and distance>0
        valid = (Conf >= 1) & (Dist > 0)

        # force invalid values to safe defaults
        Dist = np.where(valid, Dist, 0.0)
        Refl = np.where(valid, Refl, 0.0)
        PFA = np.where(valid, PFA, 1.0)

        # angles (degrees)
        # theta: azimuth in xy plane
        # phi: elevation
        theta = np.degrees(np.arctan2(Y, X))
        phi = np.degrees(np.arctan2(Z, np.sqrt(X**2 + Y**2)))

        # optional clamping rules (your original behavior)
        theta = np.where(X == 0, 0.0, theta)
        phi = np.where(Dist == 0, 0.0, phi)

        # write one csv row per point index
        for i in range(len(mes)):
            writer.writerow([
                frame_id, dt, timestamp, i,
                Dist[i], Refl[i], round(float(PFA[i]), 4),
                float(X[i]), float(Y[i]), float(Z[i]),
                round(float(theta[i]), 4), round(float(phi[i]), 4)
            ])


def main():
    """prints device info once, then exports all frames in the given range."""
    metadata()
    for frame_id in range(first_frame, last_frame + 1):
        convert_frame(frame_id)


if __name__ == "__main__":
    main()
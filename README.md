# KITTI dataset helper

Python helpers for loading KITTI odometry and raw Velodyne scans as NumPy arrays, together with sensor poses. Raw sequences also provide frame times and estimated per-point timestamps. Scans are loaded on demand; calibration and poses are loaded when a sequence is created.

## Setup

Python 3 and NumPy are required. From this directory:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy
```

Run your scripts from this directory, or put `kitti_dataset.py` and `kitti_raw_dataset.py` on your Python import path. There is no package installation step or command-line interface.

## Dataset layout

The following layout matches the dataset at `/Users/igo320341/Desktop/kitti_odometry`. Replace this path in the examples if your data is elsewhere.

```text
kitti_odometry/
├── poses/
│   └── 00.txt
├── sequences/
│   └── 00/
│       ├── calib.txt
│       ├── times.txt
│       └── velodyne/
│           ├── 000000.bin
│           └── ...
└── raw/
    └── 2011_10_03/
        ├── calib_imu_to_velo.txt
        ├── calib_cam_to_cam.txt
        ├── calib_velo_to_cam.txt
        ├── 2011_10_03_drive_0027_sync/
        │   ├── velodyne_points/
        │   │   ├── timestamps.txt
        │   │   ├── timestamps_start.txt
        │   │   ├── timestamps_end.txt
        │   │   └── data/
        │   │       ├── 0000000000.bin
        │   │       └── ...
        │   └── oxts/
        │       ├── timestamps.txt
        │       └── data/
        │           ├── 0000000000.txt
        │           └── ...
        └── 2011_10_03_drive_0027_extract/
            └── ...
```

Odometry requires `sequences/<sequence>/calib.txt`, Velodyne `.bin` files, and `poses/<sequence>.txt`. Its `times.txt` is not read. Raw loading requires the date's `calib_imu_to_velo.txt`, Velodyne and OXTS data, and both `timestamps.txt` files. Images, other calibration files, and scan start/end timestamps are not read by these helpers.

## Load an odometry sequence

```python
from kitti_dataset import KittiOdometrySequence

dataset = KittiOdometrySequence("/Users/igo320341/Desktop/kitti_odometry", "00")

print("Frames:", len(dataset))
frame = dataset.get_frame(0)
print("Frame index:", frame.index)
print("Scan file:", frame.filename)
print("Points:", frame.points_xyz.shape)
print("Reflectance:", frame.reflectance.shape)
print("Velodyne pose:\n", frame.T_w_velo)
```

Keep the sequence ID as a zero-padded string, such as `"00"`. The pose file is mandatory; sequences without a pose file cannot be loaded by this helper.

## Load a raw sequence

```python
from kitti_raw_dataset import KittiRawSequence

dataset = KittiRawSequence(
    base_path="/Users/igo320341/Desktop/kitti_odometry/raw",
    date="2011_10_03",
    drive="0027",
    dataset="sync",
)

print("Frames:", len(dataset))
frame = dataset.get_frame(0)
print("Frame time:", frame.frame_time)
print("Points:", frame.points_xyz.shape)
print("Point timestamps:", frame.point_timestamps.shape)
print("Velodyne pose:\n", frame.T_w_velo)
```

`base_path` must contain the date directory. Keep the drive ID zero-padded: `"0027"`. The helper builds the directory name `2011_10_03_drive_0027_sync` from these arguments.

Use `dataset="sync"` for scans and OXTS packets paired by index. Although `dataset="extract"` selects the corresponding extracted folder, the helper does not synchronize or interpolate OXTS measurements to scan times; it always pairs by sorted index.

## Select frames and iterate

Both constructors accept `frame_range`. For example, process every tenth frame among the first 100 frames:

```python
from kitti_dataset import KittiOdometrySequence

dataset = KittiOdometrySequence(
    "/Users/igo320341/Desktop/kitti_odometry",
    "00",
    frame_range=range(0, 100, 10),
)

for frame in dataset:
    print(frame.index, frame.points_xyz.shape[0])
```

`get_frame(i)` addresses the selected sequence: here, `get_frame(1)` returns original frame 10. `frame.index` retains the original index. Out-of-range indices in `frame_range` are discarded. Use `get_frame(i)` rather than `dataset[i]`.

## Frame data and coordinate transforms

| Field | Meaning |
| --- | --- |
| `index` | Original frame index before filtering |
| `filename` | Path to the Velodyne scan |
| `points_xyz` | Float64 array of shape `(N, 3)`, XYZ coordinates in the Velodyne frame, in meters |
| `reflectance` | Float64 array of shape `(N,)`, containing each point's reflectance |
| `T_w_velo` | Float64 homogeneous transform of shape `(4, 4)`, mapping Velodyne coordinates to the helper's world frame |
| `frame_time` | Raw only: Python `datetime` from the scan timestamp |
| `point_timestamps` | Raw only: float64 array of shape `(N,)`, estimated timestamps in seconds |

Transform points into world coordinates:

```python
points_world = (
    frame.points_xyz @ frame.T_w_velo[:3, :3].T
    + frame.T_w_velo[:3, 3]
)
```

Odometry poses are computed as `T_w_cam0 @ T_cam0_velo`, using the supplied camera-0 poses and `calib.txt`. The world frame is the reference frame of those poses.

Raw poses are derived from OXTS latitude, longitude, altitude, roll, pitch, and yaw, then converted with `T_w_imu @ T_imu_velo`. The first OXTS position is subtracted from all translations, but its orientation is retained, so the first pose is not necessarily identity. Odometry and raw world frames should not be assumed to coincide.

Calibration is available through `dataset.calib`. Odometry exposes `T_cam0_velo` and available projection matrices (`P_rect_00`, `P_rect_10`, `P_rect_20`, `P_rect_30`). Raw exposes `T_velo_imu` and its inverse, `T_imu_velo`.

## Raw timestamp limitations

Per-point times are estimated from point azimuth, assuming a full rotation takes `scan_duration` seconds (default `0.1`). Set `scan_duration` in the raw constructor to change that assumption. Offsets wrap around the frame timestamp and are approximately within half a scan duration on either side. These are estimates, not measured firing times, and the helper does not deskew scans.

Timestamp parsing truncates fractional seconds to microseconds. Parsed datetimes have no timezone; conversion to seconds uses Python's `datetime.timestamp()`, which interprets them in the machine's local timezone. Account for this when comparing them with external timestamps.

## Troubleshooting

- **Import fails:** run from the helper directory and install NumPy in the same Python environment used to run the script.
- **Missing odometry folder or pose file:** check the dataset root and zero-padded sequence ID. Calibration must be inside `sequences/<sequence>/calib.txt`.
- **Missing raw files:** check the date, drive, and dataset suffix, and keep calibration files directly inside the date directory.
- **Fewer frames than expected:** odometry truncates scans and poses to their shared length. Raw also truncates against timestamps and OXTS poses. Neither helper verifies matching frame identifiers or repairs missing frames; use complete, aligned files.

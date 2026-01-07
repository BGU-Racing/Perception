import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose


# ---- PointField helpers ----
FLOAT32 = PointField.FLOAT32
UINT32 = PointField.UINT32

def make_fields(include_intensity=False, include_reflectivity=False, include_pfalse=False, include_frame_id=False):
    fields = []
    offset = 0

    def add(name, datatype, count=1):
        nonlocal offset
        fields.append(PointField(name=name, offset=offset, datatype=datatype, count=count))
        offset += 4 * count  # for FLOAT32/UINT32 (4 bytes)
    add("x", FLOAT32)
    add("y", FLOAT32)
    add("z", FLOAT32)

    if include_intensity:
        add("intensity", FLOAT32)
    if include_reflectivity:
        add("reflectivity", FLOAT32)
    if include_pfalse:
        add("p_false", FLOAT32)
    if include_frame_id:
        add("frame_id", UINT32)

    point_step = offset
    return fields, point_step

def numpy_to_pointcloud2(points_xyz: np.ndarray, header: Header, fields, point_step: int) -> PointCloud2:
    points_xyz = np.asarray(points_xyz, dtype=np.float32)
    n = points_xyz.shape[0]

    # Create packed binary buffer (x,y,z only) unless fields specify more
    # Here we support x,y,z as the first 3 float32, and ignore extras unless you pack them yourself.
    buf = np.zeros((n, point_step // 4), dtype=np.float32)
    buf[:, 0:3] = points_xyz[:, 0:3]
    data = buf.tobytes()

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.is_dense = True
    msg.data = data
    return msg

def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    n = msg.width * msg.height
    step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)

    # view first 12 bytes as float32 (x,y,z)
    xyz = raw[:, :12].view(np.float32).reshape(n, 3).copy()
    return xyz

def centers_to_pose(centers: dict, header, frame_id: str) -> PoseArray:
    pa = PoseArray()
    pa.header = header
    pa.header.frame_id = frame_id

    if not centers:
        return pa

    PoseCls = Pose
    append_pose = pa.poses.append  

    for c in centers.values():
        p = PoseCls()
        p.position.x = float(c[0])
        p.position.y = float(c[1])
        p.position.z = float(c[2])
        p.orientation.w = 1.0
        append_pose(p)

    return pa
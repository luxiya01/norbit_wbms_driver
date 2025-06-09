import array
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

def create_pointcloud2_xyz_intensity(
    xyz: np.ndarray,
    intensity: np.ndarray,
    frame_id: str = "map"
) -> PointCloud2:
    """
    Create a PointCloud2 message from xyz coordinates and intensity values.
    """
    assert xyz.ndim == 2 and xyz.shape[1] == 3, "xyz must be shape (N, 3)"
    assert intensity.shape[0] == xyz.shape[0], "intensity must have same length as xyz"

    num_points = xyz.shape[0]
    point_step = 16  # 3 float32 (xyz) + 1 float32 (intensity)

    # Define fields
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    # Interleave xyz + intensity per point
    combined = np.hstack([xyz, intensity.reshape(-1, 1)]).astype(np.float32)
    byte_data = combined.tobytes()  # flat byte buffer

    # Create message
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = num_points
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * num_points
    msg.is_dense = True
    msg.data = array.array('B', byte_data)

    return msg

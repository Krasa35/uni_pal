from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

def insert_content(file_path, content, place):
  with open(file_path, 'r') as file:
      lines=file.readlines()
  lines.insert(place, content)
  lines.insert(place, "\n")
  with open(file_path, 'w') as file:
      file.writelines(lines)

def indent_content(file_path):
  with open(file_path, 'r') as file:
      lines=file.readlines()
  indented_lines = ["  " + line for line in lines]
  with open(file_path, 'w') as file:
      file.writelines(indented_lines)

def create_pose_stamped(**kwargs) -> PoseStamped:
    """
    @brief Creates a PoseStamped message.

    @param xyz_ A list of three floats representing the x, y, and z coordinates.
    @param rpy_ (optional) A list of three floats representing the roll, pitch, and yaw for orientation.
    @param xyzw_ (optional) A list of four floats representing the quaternion (x, y, z, w) for orientation.
    @param seconds_ (optional) The time in seconds for the header.
    @param frame_id_ (optional) The frame ID for the header.

    @return PoseStamped The created PoseStamped message.
    """
    if kwargs["rpy_"] is not None:
        quaternion_ = quaternion_from_euler(kwargs["rpy_"][0], kwargs["rpy_"][1], kwargs["rpy_"][2])
    elif kwargs["xyzw_"] is not None:
        quaternion_ = kwargs["xyzw_"]
    else:
        quaternion_ = [0.0, 0.0, 0.0, 1.0]
    orientation_ = Quaternion(quaternion_)
    pose = PoseStamped(
        header=Header(stamp=Time(sec=kwargs["seconds_"], nanosec=0), frame_id=kwargs["frame_id_"]),
        pose=Pose(position=Point(
                x=float(kwargs["xyz_"][0]), y=float(kwargs["xyz_"][1]), z=float(kwargs["xyz_"][2])), 
            orientation=orientation_
    ))
    return pose

def print_pose_stamped(pose: PoseStamped, **kwargs):
    if "rpy" in kwargs:
        orientation_ = euler_from_quaternion(pose.pose.orientation)
        forientation = f'''
        \troll: {orientation_[0]}
        \tpitch: {orientation_[1]}
        \tyaw: {orientation_[2]}'''
    else:
        orientation_ = pose.pose.orientation
        forientation = f'''
        \tx: {orientation_.x}
        \ty: {orientation_.y}
        \tz: {orientation_.z}
        \tw: {orientation_.w}'''
        
    fstring = f'''Position:
        \tx: {pose.pose.position.x}
        \ty: {pose.pose.position.y}
        \tz: {pose.pose.position.z}
    Orientation:{forientation}'''
    return fstring
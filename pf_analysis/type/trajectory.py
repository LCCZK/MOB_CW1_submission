from typing import List, Union

import numpy as np
import matplotlib.pyplot as plt
from rclpy.time import Time
from bag_lib.bag_provider import BagProvider
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry

# Type alias
PoseMsg = Union[PoseStamped, PoseWithCovarianceStamped]


class PosePoint:
    """ Represents a pose (position: [x, y, z] and quaternion: [w, x, y, z]) at a point in time. """

    def __init__(self, time: Time, pos: np.ndarray, quat: np.ndarray):
        """
        Initialise a new Pose Point object.
        :param time: Time stamp of the pose point
        :param pos: Position of pose
        :param quat: Quaternion of pose
        """
        self.t = time
        self.p = pos
        self.q = quat

    def __getitem__(self, idx):
        return [self.t, self.p, self.q][idx]


class Trajectory:
    """ Represents a collection of pose point objects. """

    def __init__(self, poses: List[PosePoint]):
        """
        Initialize a new trajectory object
        :param poses: A list of poses, each with a timestamp associated with them
        """
        self.poses = poses

    @staticmethod
    def from_bag(path: str, topic: str) -> 'Trajectory':
        """
        Extract the set of poses from a bag file.
        :param path: Path to a bag file
        :param topic: Topic of the poses
        :return: Trajectory object containing timestamps, positions and orientations
        """
        bag = BagProvider(path)
        if topic not in bag.get_topics():
            raise RuntimeError("Topic is not in bag file!")

        pose_messages: List[PoseMsg] = bag.get_messages(topic)["msg"]
        if len(pose_messages) == 0:
            raise RuntimeError("No messages!")

        # Extract time stamps
        time = [Time.from_msg(msg.header.stamp) for msg in pose_messages]

        # Support both PoseStamped and PoseWithCovarianceStamped
        if hasattr(pose_messages[0].pose, "pose"):
            pose_messages = [msg.pose for msg in pose_messages]

        # Extract position [x, y, z]
        pos = [np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z]) for msg in pose_messages]

        # Extract orientation [w, x, y, z] <- quaternion
        quat = [np.array([msg.pose.orientation.w,
                          msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z]) for msg in pose_messages]

        return Trajectory([PosePoint(*x) for x in zip(time, pos, quat)])

    def __getitem__(self, idx):
        """
        Return pose point at given index
        :param idx: Index position of pose point
        :return: Pose point: [time_ns, [x, y, z], [w, x, y, z]]
        """
        return self.poses[idx]

    def size(self) -> int:
        """
        Get the number of pose points in the trajectory
        :return: Number of pose points
        """
        return len(self.poses)

    @property
    def t(self) -> List[Time]:
        """ Return a list of timestamps of each pose point. """
        return [p.t for p in self.poses]

    @property
    def p(self) -> List[np.ndarray]:
        """ Return a list of positions of each pose point. """
        return [p.p for p in self.poses]

    @property
    def q(self) -> List[np.ndarray]:
        """ Return a list of quaternions of each pose point. """
        return [p.q for p in self.poses]

    def plot(self, axs: plt.Axes, **kwargs) -> plt.Axes:
        """
        Plot trajectory on given matplotlib axes
        :param axs: Matplotlib axes
        :param kwargs: Additional arguments to be passed to scatter method
        :return: Matplotlib axes
        """

        x = [p[0] for p in self.p]
        y = [p[1] for p in self.p]
        axs.scatter(x, y, **kwargs)
        return axs

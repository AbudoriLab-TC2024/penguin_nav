#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import rclpy.qos
import rclpy.clock
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
import pandas as pd
import math
import sys
from dataclasses import dataclass
from typing import List, Tuple
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter


# Columns configuration
X = "x"
Y = "y"
# for pcl_pose
# X = "/pcl_pose/pose/pose/position/x"
# Y = "/pcl_pose/pose/pose/position/y"

WINDOW_EXPAND = 4.0
GLOBAL_COSTMAP_NODE = "/global_costmap/global_costmap"


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_msg(self) -> Pose:
        q = Rotation.from_euler("z", self.yaw).as_quat()

        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.position.z = 0.0
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        return p


class PathFollower(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.marker_pub_ = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 5
        )
        self.global_costmap_node_param_set_ = self.create_client(
            SetParameters, f"{GLOBAL_COSTMAP_NODE}/set_parameters"
        )
        if not self.global_costmap_node_param_set_.wait_for_service(timeout_sec=1.0):
            self.warn(f"global costmap node {GLOBAL_COSTMAP_NODE} is not found.")

    def execute_one(self, plan: List[Pose2D]):
        if not self.set_window(
            (min([p.x for p in plan]), max([p.x for p in plan])),
            (min([p.y for p in plan]), max([p.y for p in plan])),
        ):
            return

        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        poses = [PoseStamped(header=header, pose=p.to_msg()) for p in plan]

        self.publish_rviz(poses)

        self.goThroughPoses(poses)

        i = 0
        while not self.isTaskComplete():
            i += 1
            if (feedback := self.getFeedback()) and i % 5 == 0:
                self.debug(
                    f"Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds"
                )

    def publish_rviz(self, poses: List[PoseStamped]):
        markers: List[Marker] = []
        self.marker_pub_.publish(
            MarkerArray(
                markers=[Marker(ns="current_waypoints", action=Marker.DELETEALL)]
            )
        )
        for i, p in enumerate(poses):
            m = Marker()
            m.ns = "current_waypoints"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime = Duration().to_msg()
            m.scale.x = 0.4
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0

            m.pose = p.pose
            m.header = p.header

            markers.append(m)

        self.marker_pub_.publish(MarkerArray(markers=markers))

    def set_window(self, x_range, y_range):
        origin_x = float(x_range[0] - WINDOW_EXPAND)
        origin_y = float(y_range[0] - WINDOW_EXPAND)
        width = int(x_range[1] - x_range[0] + WINDOW_EXPAND * 2)
        height = int(y_range[1] - y_range[0] + WINDOW_EXPAND * 2)
        future = self.global_costmap_node_param_set_.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(name="origin_x", value=origin_x).to_parameter_msg(),
                    Parameter(name="origin_y", value=origin_y).to_parameter_msg(),
                    Parameter(name="width", value=width).to_parameter_msg(),
                    Parameter(name="height", value=height).to_parameter_msg(),
                ]
            )
        )
        rclpy.spin_until_future_complete(self, future)
        ret = future.result()
        if not all([r.successful for r in ret.results]):
            self.warn(ret)
            return False
        return True


def df_to_poses(df: pd.DataFrame) -> List[Pose2D]:
    poses: List[Pose2D] = []

    for _, row in df.iterrows():
        poses.append(Pose2D(float(row[X]), float(row[Y]), 0.0))

    for i in range(len(poses) - 1):
        x1, y1 = poses[i].x, poses[i].y
        x2, y2 = poses[i + 1].x, poses[i + 1].y
        yaw = math.atan2(y2 - y1, x2 - x1)
        poses[i].yaw = yaw

    # repeat tail yaw
    if len(poses) > 1:
        poses[-1].yaw = poses[-2].yaw
    else:
        poses[-1].yaw = 0.0
    return poses


def split(df: pd.DataFrame) -> List[pd.DataFrame]:
    # split verticaly is the action col is not NaN
    df.loc[df["action"].isna(), ["action"]] = ""
    index = df[df["action"] != ""].index
    index = sorted(list(set([-1] + list(index) + [df.index[-1]])))

    dfs = [df.loc[index[i] + 1 : index[i + 1]] for i in range(len(index) - 1)]

    if dfs[-1].at[dfs[-1].index[-1], "action"] == "":
        dfs[-1].at[dfs[-1].index[-1], "action"] = "stop"

    return dfs


def read_dfs(files: List[str]) -> List[Tuple[pd.DataFrame, str]]:
    dfs = []
    for file in files:
        df = pd.read_csv(file)
        if "action" in df.columns:
            dfs.extend([(df, f"{file}:{i}") for i, df in enumerate(split(df))])
        else:
            df["action"] = ""
            df.at[df.index[-1], "action"] = "stop"
            dfs.append((df, file))
    return dfs


def main():
    rclpy.init()

    path_follower = PathFollower()

    files = sys.argv[1:]

    dfs = read_dfs(files)

    try:
        for i, (df, name) in enumerate(dfs):
            path_follower.get_logger().info(f"Running {name} ... {i+1}/{len(dfs)}")

            poses = df_to_poses(df)

            path_follower.execute_one(poses)

            if i != len(dfs) - 1:
                if df.at[df.index[-1], "action"] == "stop":
                    val = input(
                        f"Finish {i+1}th path. Press enter to continue. Type 'q' to quit."
                    )
                    if val == "q":
                        break

    except KeyboardInterrupt:
        path_follower.cancelTask()
        raise


if __name__ == "__main__":
    main()

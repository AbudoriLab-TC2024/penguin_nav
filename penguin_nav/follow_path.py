#!/usr/bin/env python3

import threading
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from typing import List
import pandas as pd
import sys
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.duration import Duration
from std_msgs.msg import Header

# Configurations
X = "x"
Y = "y"
YAW = "yaw"  # compuated by the script
ACTION = "action"
POSE_TOPIC = "/pcl_pose"
WINDOW_EXPAND = 4.0
GLOBAL_COSTMAP_NODE = "/global_costmap/global_costmap"

# Global condition to manipuate threading
pausing = False
thread_running = True


class Input(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.txt = None
        self.prompted = False
        self.start()

    def run(self):
        while True:
            self.txt = input()

    def input(self, message):
        if not self.prompted:
            print(message)
            self.prompted = True

        txt = self.txt
        if txt is not None:
            self.prompted = False
        self.txt = None
        return txt


@dataclass
class Pose2D:
    """Class that represents each csv element.
    Note that 'yaw' in csv file is not taken, calculated inside this script.
    """

    x: float
    y: float
    yaw: float
    action: str = ""

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

    @classmethod
    def from_dataframe(cls, df: pd.DataFrame) -> List["Pose2D"]:
        return [
            cls(x=row[X], y=row[Y], yaw=row[YAW], action=row[ACTION])
            for _, row in df.iterrows()
        ]


def read_csv(files: List[str]) -> pd.DataFrame:
    """Read CSV files and convert to DataFrame.

    This function reads csv files and concat them into one DataFrame.
    "action" column is added if it does not exist in the file.
    The tail "action" is set to "stop" if the tail "action" is empty in the file (checked one-by-one).
    """

    def read(f):
        df = pd.read_csv(f)
        df.loc[df[ACTION].isna(), [ACTION]] = ""
        if ACTION in df.columns:
            if df.at[df.index[-1], ACTION] == "":
                df.at[df.index[-1], ACTION] = "stop"
        else:
            df["action"] = ""
            df.at[df.index[-1], ACTION] = "stop"
        return df

    dfs = [read(f) for f in files]

    return pd.concat(dfs, ignore_index=True)


def split_df(df: pd.DataFrame) -> List[pd.DataFrame]:
    """Split DataFrame by "action" terminal."""
    index = df[df[ACTION] != ""].index
    index = sorted(list(set([-1] + list(index))))

    return [df.loc[index[i] + 1 : index[i + 1]] for i in range(len(index) - 1)]


def compute_yaw(df: pd.DataFrame) -> pd.DataFrame:
    """Compute yaw angle by points positions."""
    df[YAW] = 0.0
    for i in range(1, len(df)):
        dx = df.at[df.index[i], X] - df.at[df.index[i - 1], X]
        dy = df.at[df.index[i], Y] - df.at[df.index[i - 1], Y]
        df.at[i, YAW] = math.atan2(dy, dx)
    return df


class PathFollower(BasicNavigator):
    """SimpleCommander wrapper.

    The entry function is "run".
    In the function, the following procedure is looped.
    1. check if the planner task is completed
    2. take new plan segment if necessary (by spin_once).
    3. request new plan segment to planner

    Since ROS 2 does not allow to call 'spin' inside another 'spin' No.1 and No.2 need to be called outside of timer_callback.

    All the process are non-blocking, so the flags whether (e.g.) to request to planner are hand-shaked internally.
    """

    def __init__(self, df: pd.DataFrame):
        """Initialize PathFollower and visualize entire plan by publishing rviz marker."""
        super().__init__()
        # global costmap
        self.global_costmap_node_param_set_ = self.create_client(
            SetParameters, f"{GLOBAL_COSTMAP_NODE}/set_parameters"
        )
        if not self.global_costmap_node_param_set_.wait_for_service(timeout_sec=1.0):
            raise f"Global costmap node {GLOBAL_COSTMAP_NODE} is not found."

        # for visualization
        self.marker_pub_ = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 5
        )

        # for robot pose
        self.pose_sub_ = self.create_subscription(
            PoseWithCovarianceStamped, POSE_TOPIC, self.pose_callback, 5
        )
        self.last_pose_ = PoseWithCovarianceStamped()
        self.last_pose_.header.frame_id = "map"

        # for plan state
        self.plan_ = split_df(df)
        self.plan_enum_iter = enumerate(self.plan_)

        # loop
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.input = Input()

        # To controll infinite loop
        self.loop_running_ = True
        # Hold current short plan. set in timer_callback via request_new_plan and consomued in take_new_plan.
        self.current_plan_ = None

        # Flag to notify timer_callback should take new plan. Will turn true if the current planner task is completed (check_task_completed).
        self.should_take_new_plan_ = True
        # Flag to notify timer_callback should wait key input before taking new plan. If the last action is 'stop' this will be true.
        self.should_wait_input_ = True
        # Last index of plan segment. Used to check if the all segments are completed.
        self.last_index_ = None

        # publish global path
        self.info(
            f"Loaded {len(df)} points global plan with {len(self.plan_)} segments"
        )
        self.publish_marker(
            Pose2D.from_dataframe(df),
            "global_path",
            Header(frame_id="map"),
            (0.0, 1.0, 0.0),
            0.7,
        )

    def publish_marker(self, poses: List[Pose2D], name, header, color, scale=1.0):
        markers: List[Marker] = []
        self.marker_pub_.publish(
            MarkerArray(markers=[Marker(ns=name, action=Marker.DELETEALL)])
        )
        """Publish the plan as rviz markers."""
        for i, p in enumerate(poses):
            if p.action == "stop":
                this_color = (1.0, 0.0, 0.0)
            elif p.action == "continue":
                this_color = (0.0, 0.0, 1.0)
            else:
                this_color = color

            m = Marker()
            m.ns = name
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime = Duration().to_msg()
            m.scale.x = 0.4 * scale
            m.scale.y = 0.1 * scale
            m.scale.z = 0.1 * scale
            m.color.r = this_color[0]
            m.color.g = this_color[1]
            m.color.b = this_color[2]
            m.color.a = 1.0

            m.pose = p.to_msg()
            m.header = header

            markers.append(m)

        self.marker_pub_.publish(MarkerArray(markers=markers))

    def check_task_completed(self):
        """Check if the planner task is completed and set 'should_take_new_plan'.

        Should be called outside of another executor.
        """
        self.should_take_new_plan_ = self.isTaskComplete()

    def request_new_plan(self, new_plan: List[Pose2D]):
        """Request new plan to the planner by setting variable.

        This will be taken by 'take_new_plan' later.
        """
        self.current_plan_ = new_plan

    def take_new_plan(self):
        """Take new plan and send to the planner server.

        This function does:
        1. update global costmap (via set_window)
        2. send plan (via goThroughPoses)
        3. send visualization

        Should be called outside of another executor.
        """
        if self.current_plan_ is None:
            return
        short_plan = self.current_plan_
        self.current_plan_ = None

        robot_x = self.last_pose_.pose.pose.position.x
        robot_y = self.last_pose_.pose.pose.position.y

        if not self.set_window(
            (
                min([p.x for p in short_plan] + [robot_x]),
                max([p.x for p in short_plan] + [robot_x]),
            ),
            (
                min([p.y for p in short_plan] + [robot_y]),
                max([p.y for p in short_plan] + [robot_y]),
            ),
        ):
            return

        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        self.publish_marker(short_plan, "short_plan", header, (0.0, 1.0, 1.0))

        self.goThroughPoses(
            [PoseStamped(header=header, pose=p.to_msg()) for p in short_plan]
        )

    def timer_callback(self):
        """Timer callback function.

        This is is for event loop, and does:
        1. check if the all the plan segment is completed
        2. check keyboard input (non-blocking), and request quit if necessary
        3. request new plan segment
        """
        # Event loop
        if self.should_take_new_plan_:
            if self.last_index_ and self.last_index_ + 1 == len(self.plan_):
                self.loop_running_ = False
                return

            if self.should_wait_input_:
                txt = self.input.input(
                    "Press enter to continue. Type 'q' and enter to quit."
                )
                if txt is None:
                    # to continue
                    return
                if txt == "q":
                    self.info("Quit")
                    self.loop_running_ = False
                    return
                else:
                    self.info("Continue")

            i, df = next(self.plan_enum_iter, (None, None))
            self.last_index_ = i
            if df is None:
                self.loop_running_ = False
                return
            self.info(f"Running ... {i+1}/{len(self.plan_)}")

            short_plan = Pose2D.from_dataframe(df)
            self.request_new_plan(short_plan)
            self.should_wait_input_ = short_plan[-1].action == "stop"
            self.should_take_new_plan_ = False

    def set_window(self, x_range, y_range):
        """Update global costmap by setting origin_x/y, width and height parameters.

        Note that by setting these parameters, the costmap is reset.
        """
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        ret = future.result()
        if not all([r.successful for r in ret.results]):
            self.warn(ret)
            return False
        return True

    def pose_callback(self, msg):
        self.last_pose_ = msg

    def run(self):
        """Entry function"""
        while self.loop_running_:
            self.check_task_completed()
            rclpy.spin_once(self)
            self.take_new_plan()


def main():
    plan = read_csv(sys.argv[1:])
    compute_yaw(plan)
    rclpy.init()

    node = PathFollower(plan)

    try:
        node.run()
    except KeyboardInterrupt:
        print("Cancelling current task")
        if not node.isTaskComplete():
            node.cancelTask()
        raise


if __name__ == "__main__":
    main()

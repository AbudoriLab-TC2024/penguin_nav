#!/usr/bin/env python3

from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from typing import List, Tuple
import pandas as pd
import time
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

from penguin_nav.input import Input
from penguin_nav.waypoint import Loader, Plan, Waypoint
from penguin_nav.visualizer import Visualizer
from penguin_nav_msgs.srv import AdjustWaypoints, NeedAdjust


# Configurations
X = "x"
Y = "y"
YAW = "yaw"  # compuated by the script
ACTION = "action"
POSE_TOPIC = "/pcl_pose"
WINDOW_EXPAND = 4.0
GLOBAL_COSTMAP_NODE = "/global_costmap/global_costmap"


class DivideNavigator(BasicNavigator):
    def __init__(
        self,
        costmap_node_name: str = "/global_costmap/global_costmap",
        pose_topic: str = "/pcl_pose",
        adjust_service: str = "/adjust_waypoints",
        need_adjust_service: str = "/need_adjust",
        window_expand: float = 4.0,
    ):
        super().__init__()
        self._costmap_cli = self.create_client(
            SetParameters, f"{costmap_node_name}/set_parameters"
        )
        if not self._costmap_cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Costmap node {costmap_node_name} is not found.")

        self._adjust_cli = self.create_client(AdjustWaypoints, adjust_service)
        if not self._adjust_cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Adjust service {adjust_service} is not found.")

        self._need_adjust_cli = self.create_client(NeedAdjust, need_adjust_service)
        if not self._need_adjust_cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(
                f"Need adjust service {need_adjust_service} is not found."
            )

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self.pose_callback, 5
        )

        # configure
        self._window_expand = window_expand

        # buffers
        self._last_pose = None
        self._adjust_future = None
        self._set_window_future = None

    def pose_callback(self, msg):
        self._last_pose = msg

    def set_window(self, x_range, y_range):
        origin_x = float(x_range[0] - self._window_expand)
        origin_y = float(y_range[0] - self._window_expand)
        width = int(x_range[1] - x_range[0] + self._window_expand * 2)
        height = int(y_range[1] - y_range[0] + self._window_expand * 2)
        self._set_window_future = self._costmap_cli.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(name="origin_x", value=origin_x).to_parameter_msg(),
                    Parameter(name="origin_y", value=origin_y).to_parameter_msg(),
                    Parameter(name="width", value=width).to_parameter_msg(),
                    Parameter(name="height", value=height).to_parameter_msg(),
                ]
            )
        )

    def adjust_waypoints(self, plan: Plan):
        waypoints, left_torelances, right_torelances = plan.to_msg(
            self.make_map_header()
        )
        self._adjust_future = self._adjust_cli.call_async(
            AdjustWaypoints.Request(
                waypoints=waypoints,
                left_torelances=left_torelances,
                right_torelances=right_torelances,
            )
        )

    def need_adjust(self, plan: Plan):
        waypoints, *_ = plan.to_msg(self.make_map_header())
        future = self._need_adjust_cli.call_async(
            NeedAdjust.Request(waypoints=waypoints)
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future:
            return False
        return future.result().need_adjust

    def request_plan(self, plan: Plan, update_costmap: bool):
        if not self._last_pose:
            self.warn(f"Robot pose is not received yet.")
            return

        # self.cancel()
        if update_costmap:
            robot_x = self._last_pose.pose.pose.position.x
            robot_y = self._last_pose.pose.pose.position.y

            xx = [p.x for p in plan.waypoints] + [robot_x]
            yy = [p.y for p in plan.waypoints] + [robot_y]

            self.set_window((min(xx), max(xx)), (min(yy), max(yy)))
            if not self.is_setting_window_completed():
                return

        waypoints, *_ = plan.to_msg(self.make_map_header())

        self.goThroughPoses(waypoints)

    ###### should be called outside of another executor ######
    def get_adjusted_waypoints(self) -> List[PoseStamped]:
        if not self._adjust_future:
            return None
        rclpy.spin_until_future_complete(self, self._adjust_future, timeout_sec=0.1)
        if not self._adjust_future.result():
            return None
        ret = None
        if self._adjust_future.result().modified:
            ret = self._adjust_future.result().waypoints
        self._adjust_future = None
        return ret

    def is_setting_window_completed(self, timeout=1.0):
        if not self._set_window_future:
            return True
        rclpy.spin_until_future_complete(
            self, self._set_window_future, timeout_sec=timeout
        )
        ret = True
        if not all([r.successful for r in self._set_window_future.result().results]):
            self.warn(self._set_window_future.result())
            ret = False
        self._set_window_future = None
        return ret

    def cancel(self):
        if not self.isTaskComplete():
            self.cancelTask()

    def make_map_header(self):
        return Header(frame_id="map", stamp=self.get_clock().now().to_msg())


class Controller(Input):
    def __init__(self):
        super().__init__()

    def go_next(self) -> Tuple[bool, bool]:
        """(go_next, keep_running)"""
        txt = self.input("Please enter to continue. Type 'q' and enter to quit.")
        if txt is None:
            return False, True
        if txt == "q":
            return False, False
        else:
            return True, True


class Mission:
    def __init__(self, plans: List[Plan]):
        self._nav = DivideNavigator()
        self._vis = Visualizer(self._nav)
        self._controller = Controller()
        self._delta_time = 0.2

        self._plans = plans
        self._plan_i_iter = enumerate(self._plans)

        # visualize entire plan
        entire_waypoints = sum([p.waypoints for p in plans], [])
        self.info(
            f"Loaded {len(entire_waypoints)} waypoints totally with {len(plans)} segments"
        )
        self._vis.publish(entire_waypoints, "entire_plan", (0.0, 1.0, 0.0), 0.9)

    def info(self, message: str):
        self._nav.info(message)

    def run(self):
        # message
        current_plan = None
        adjusted_plan = None
        new_plan = None
        should_wait_next = True
        last_index = 0

        try:
            while True:
                start_time = time.time()
                rclpy.spin_once(self._nav)

                if not current_plan:
                    go_next, keep_running = (
                        self._controller.go_next() if should_wait_next else (True, True)
                    )
                    if not keep_running:
                        break

                    if go_next:
                        last_index, new_plan = next(self._plan_i_iter, (None, None))
                        if new_plan is None:
                            break

                        self.info(f"Running ... {last_index+1}/{len(self._plans)}")

                if new_plan:
                    self._nav.request_plan(new_plan, True)
                    self._vis.publish(
                        new_plan.waypoints, "current_plan", (0.0, 1.0, 1.0)
                    )
                    current_plan = new_plan
                    new_plan = None

                if current_plan and adjusted_plan:
                    if self._nav.need_adjust(adjusted_plan):
                        adjusted_plan = None

                if current_plan and not adjusted_plan:
                    self._nav.adjust_waypoints(current_plan)
                    if adjusted := self._nav.get_adjusted_waypoints():
                        adjusted_plan = Plan.from_msg(adjusted)
                        current_plan.waypoints = current_plan.waypoints[
                            -len(adjusted_plan.waypoints) :
                        ]
                        for i in range(len(adjusted_plan.waypoints)):
                            adjusted_plan.waypoints[i].action = current_plan.waypoints[
                                i
                            ].action

                        self._nav.request_plan(adjusted_plan, False)
                        self._vis.publish(
                            adjusted_plan.waypoints, "current_plan", (0.0, 1.0, 1.0)
                        )

                if current_plan and self._nav.isTaskComplete():
                    should_wait_next = current_plan.should_stop()
                    current_plan = None
                    adjusted_plan = None
                    if last_index + 1 == len(self._plans):
                        break

                end_time = time.time()
                delta = end_time - start_time
                remains = self._delta_time - delta
                if remains > 0:
                    time.sleep(remains)

        except KeyboardInterrupt:
            print("Cancelling current task.")
            self._nav.cancel()
            raise


def main():
    plans = Loader().load(sys.argv[1:])
    rclpy.init()

    node = Mission(plans)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Finish")


if __name__ == "__main__":
    main()

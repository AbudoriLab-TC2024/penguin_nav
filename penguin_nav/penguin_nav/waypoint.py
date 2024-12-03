import pandas as pd
import numpy as np
import math

from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from typing import List, Tuple

import geometry_msgs.msg
import std_msgs.msg


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    action: str = ""
    left_toralence: float = 0
    right_torelance: float = 0

    def to_pose(self) -> geometry_msgs.msg.Pose:
        q = Rotation.from_euler("z", self.yaw).as_quat()

        return geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=self.x, y=self.y, z=0.0),
            orientation=geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    @classmethod
    def from_pose(cls, pose: geometry_msgs.msg.Pose):
        q = pose.orientation
        yaw = Rotation([q.x, q.y, q.z, q.w]).as_euler("ZXY")[0]
        return cls(pose.position.x, pose.position.y, yaw)

    def _solve(self, offset):
        cs = math.cos(self.yaw)
        sn = math.sin(self.yaw)
        dx = -sn * offset
        dy = +cs * offset
        return (self.x + dx, self.y + dy)

    def left(self):
        return self._solve(self.left_toralence)

    def right(self):
        return self._solve(-self.right_torelance)


@dataclass
class Plan:
    waypoints: List[Waypoint]

    def to_msg(
        self, header: std_msgs.msg.Header
    ) -> Tuple[List[geometry_msgs.msg.PoseStamped], List[float], List[float]]:
        return (
            [
                geometry_msgs.msg.PoseStamped(header=header, pose=w.to_pose())
                for w in self.waypoints
            ],
            [p.left_toralence for p in self.waypoints],
            [p.right_torelance for p in self.waypoints],
        )

    @classmethod
    def from_msg(cls, poses: List[geometry_msgs.msg.PoseStamped]) -> "Plan":
        return cls([Waypoint.from_pose(p.pose) for p in poses])

    def should_stop(self) -> bool:
        return self.waypoints[-1].action == "stop"


class Loader:
    """CSVを読み込んで Plan オブジェクトを作成する

    CSV は以下の列を持つ。x,y 以外は optioanl

    - x : 点のx座標 (map frame)
    - y : 点のy座標 (map frame)
    - yaw (optional) : 点のyaw角 (radian, map frame)
    - yaw_deg (optional) : 点のyaw角 (degree, map frame)
    - action (optional) : その点でのアクション(後述)
    - left_torelance (optional) : 点調整時の左側の許容距離
    - right_torelance (optional) : 点調整時の右側の許容距離

    省略時は、列自体省略して良いし、特定の行だけ省略しても良い。

    yawおよびyaw_degが省略された場合、その点のyaw角は p[i] -> p[i+1] のベクトルから算出される。最後の点は1つ前の点の角度がコピーされる。
    yaw角は次の優先度で採用される。 yaw > yaw_deg > ベクトルからの算出

    action は以下のいずれか。

    - 空白
    - continue
    - stop

    action は、ファイルの最終行については空白時に stop が挿入される。
    ウェイポイントは continue あるいは stop で区切ってグルーピングされる。global costmap はこのグループが収まるように調整されるため、costmap が大きすぎないように区切ることを推奨する。

    left_torelance 及び right_torelance は、点の位置を調整量である。
    これは、ウェイポイントの点が障害物に埋まった時に、その外側に再配置するためのものである。省略時は 0 となる。
    """

    def __init__(self, x="x", y="y", yaw="yaw", yaw_deg="yaw_deg", action="action"):
        self._x = x
        self._y = y
        self._yaw = yaw
        self._yaw_deg = yaw_deg
        self._action = action
        self._left_torelance = "left_torelance"
        self._right_torelance = "right_torelance"

    def load(self, files: List[str]) -> List[Plan]:
        df = self._load_and_concat(files)
        df = self._compute_yaw(df)
        return [Plan(waypoints=self._convert(d)) for d in self._split(df)]

    def _convert(self, df: pd.DataFrame) -> List[Waypoint]:
        return [
            Waypoint(
                x=row[self._x],
                y=row[self._y],
                yaw=row[self._yaw],
                action=row[self._action],
                left_toralence=row[self._left_torelance],
                right_torelance=row[self._right_torelance],
            )
            for _, row in df.iterrows()
        ]

    def _load_and_concat(self, files: List[str]) -> pd.DataFrame:
        def _read(f):
            df = pd.read_csv(f)

            if self._action not in df.columns:
                df[self._action] = ""
            df.loc[df[self._action].isna(), [self._action]] = ""

            if df.at[df.index[-1], self._action] == "":
                df.at[df.index[-1], self._action] = "stop"

            # torelance
            if self._left_torelance not in df.columns:
                df[self._left_torelance] = 0.0
            df.loc[df[self._left_torelance].isna(), [self._left_torelance]] = 0.0

            if self._right_torelance not in df.columns:
                df[self._right_torelance] = 0.0
            df.loc[df[self._right_torelance].isna(), [self._right_torelance]] = 0.0

            return df

        df = pd.concat([_read(f) for f in files], ignore_index=True)

        if self._yaw not in df.columns:
            df[self._yaw] = np.nan

        return df

    def _compute_yaw(self, df: pd.DataFrame) -> pd.DataFrame:
        yaw = 0.0

        has_yaw_deg = self._yaw_deg in df.columns

        def _set_if_nan(i, yaw):
            if not np.isnan(df.at[df.index[i], self._yaw]):
                return
            if has_yaw_deg and not np.isnan(df.at[df.index[i], self._yaw_deg]):
                yaw = math.radians(df.at[df.index[i], self._yaw_deg])
            df.at[df.index[i], self._yaw] = yaw

        for i in range(len(df) - 1):
            dx = df.at[df.index[i + 1], self._x] - df.at[df.index[i], self._x]
            dy = df.at[df.index[i + 1], self._y] - df.at[df.index[i], self._y]
            yaw = np.arctan2(dy, dx)
            _set_if_nan(i, yaw)

        _set_if_nan(len(df) - 1, yaw)

        return df

    def _split(self, df: pd.DataFrame) -> List[pd.DataFrame]:
        index = df[df[self._action] != ""].index
        index = sorted(list(set([-1] + list(index))))

        return [df.loc[index[i] + 1 : index[i + 1]] for i in range(len(index) - 1)]

import pandas as pd
import numpy as np
import math

from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from typing import List

import geometry_msgs.msg


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    action: str = ""

    def to_pose(self) -> geometry_msgs.msg.Pose:
        q = Rotation.from_euler("z", self.yaw).as_quat()

        return geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=self.x, y=self.y, z=0.0),
            orientation=geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )


@dataclass
class Plan:
    waypoints: List[Waypoint]


class Loader:
    def __init__(self, x="x", y="y", yaw="yaw", action="action"):
        self._x = x
        self._y = y
        self._yaw = yaw
        self._action = action

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
            return df

        df = pd.concat([_read(f) for f in files], ignore_index=True)

        if self._yaw not in df.columns:
            df[self._yaw] = np.nan

        return df

    def _compute_yaw(self, df: pd.DataFrame) -> pd.DataFrame:
        yaw = 0.0

        def _set_if_nan(i, yaw):
            if np.isnan(df.at[df.index[i], self._yaw]):
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

# penguin_nav

Navigation 2 を使用した経路計画行うROS 2パッケージです。

## 概要

`penguin_nav` は Navigation 2 を使用して経路計画を行うROS 2パッケージです。以下の要素で構成されています。

- Navigation 2 の経路計画の利用
    - `nav2_bringup` パッケージの `navigation_launch.py` をベースに、 `collision_monitor` による衝突回避を追加しています。
- ウェイポイントを設定するスクリプト
    - CSVファイルに書かれた XY座標を読み込み `goThroughPoses` API を通して経路を渡しています。
    - Yaw 角は、`i` -> `i+1` のベクトルから計算しています。
    - ウェイポイントはグルーピングされた点列として表現されます。
        - 1つのグループでの `goThroughPoses` 実行を繰り返す形になっています。
        - `goThroughPoses` 終了時に、キー入力待ちするかそのまま継続するかを選択できます。
    - Ctrl+C でナビゲーションタスクをキャンセルします。
    - グループを読み込む毎にGlobal Costmap を更新します。

## インストール

このリポジトリをROS 2ワークスペースにクローンし、依存パッケージをインストールしてからビルドします。

```shell
# リポジトリをクローン
cd ~/ros2_ws/src
git clone https://github.com/AbudoriLab-TC2024/penguin_nav.git

# 依存パッケージをインストール
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup python3-pandas

# ビルド
cd ~/ros2_ws
colcon build
```

## 使用方法

Navigation 及びウェイポイント設定スクリプトを起動します。

**ターミナル1**

```shell
source install/setup.sh

# Navigation + pose_to_tf
ros2 launch penguin_nav nav.launch.xml
```

**ターミナル2**


```shell
source install/setup.sh

# ウェイポイント設定スクリプト
ros2 run penguin_nav follow_path.py -- <path/to/waypoints.csv> [<path/to/waypoints2.csv> ...]
```

```shell
# ワイルドカードにより一括で渡すこともできる
$ ls waypoints_list
waypoints_0001.csv waypoints_0002.csv waypoints_0003.csv

$ ros2 run penguin_nav follow_path.py -- waypoints_list/waypoints_*.csv
# ros2 ... -- waypoints_list/waypoints_0001.csv waypoints_list/waypoints_0002.csv waypoints_list/waypoints_0003.csv と等価
```

<details><summary>waypoints.csv の例</summary>

```csv
x,y,action,yaw_deg,left_torelance,right_torelance
1,0,,90
2,0
3,0,stop,180,1.0,1.0
4,0,,,0.5,0.5
5,0
6,0,continue
7,0
8,0
9,0
```

CSVファイルは以下の列を持ちます。x, y 以外は optional です。

- x : 点のx座標 (mapフレーム)
- y : 点のy座標 (mapフレーム)
- yaw (optional) : 点のyaw角 (radian, mapフレーム)
- yaw_deg (optional) : 点のyaw角 (degree, mapフレーム)
- action (optional) : その点でのアクション（後述）
- left_torelance (optional) : 点調整時の左側の許容距離
- right_torelance (optional) : 点調整時の右側の許容距離

省略時は、列自体を省略しても良いですし、特定の行だけ省略しても構いません。

yawおよびyaw_degが省略された場合、その点のyaw角は p[i] -> p[i+1] のベクトルから算出されます。最後の点は1つ前の点の角度がコピーされます。
yaw角は次の優先度で採用されます。 yaw > yaw_deg > ベクトルからの算出

action は以下のいずれかです。

- 空白
- continue
- stop

action は、ファイルの最終行については空白時に stop が挿入されます。
ウェイポイントは continue あるいは stop で区切ってグルーピングされます。global costmap はこのグループが収まるように調整されるため、costmap が大きすぎないように区切ることを推奨します。

left_torelance 及び right_torelance は、点の位置を調整するための量です。
これは、ウェイポイントの点が障害物に埋まった時に、その外側に再配置するためのものです。省略時は 0 となります。


</details>


## パラメータ

### nav.launch.xml

[config/nav2_params.yaml](./config/nav2_params.yaml) が Navigation 2 のパラメータです。各設定は [Configuration Guide](https://docs.nav2.org/configuration/index.html) に従います。

[config/navigate_through_poses_w_replanning_and_recovery.xml](./config/navigate_through_poses_w_replanning_and_recovery.xml) が `GoThroughPoses` が使用する BehaviorTree です。


### follow_path.py

[penguin_nav/follow_path.py](./penguin_nav/follow_path.py) に座標の列名がハードコーディングされているため、必要に応じて修正してください。

## トピック

- 入力:
    - `/scan` (`sensor_msgs/msg/LaserScan`) - 2D LiDAR 形式の点群データを受け取ります。Glocal costmap、Local costmap、及び collision monitor に使用されます。
    - `/pcl_pose` -> ロボットの位置を受け取ります。Global costmap 更新時のサイズ計算に使用されます。
    - `/tf`,`/static_tf` - `map` -> `base_link` が必要です。
- 出力:
    - `/cmd_vel` (`geometry_msgs/msg/Twist`) - 制御コマンドです。


## その他

### Quick fix and DDS issue with Nav2

DDSの問題により上手く動作しないかもしれないです。その場合は[こちら](https://roboticsbackend.com/ros2-nav2-tutorial/#Quick_fix_and_DDS_issue_with_Nav2) を参考に DDS を cyclonedds に変更してください。

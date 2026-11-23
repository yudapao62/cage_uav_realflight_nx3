# CAGE UAV NX3 实飞系统

这是从 NX3 实飞计算机 `/home/nx3/cage_uav_ws/src` 独立整理的 ROS Noetic Catkin 工程源码。当前源码对应 NX3 于 2026-11-06 部署的提交 `946bdf5`，并在 2026-11-07 完成照片坐标多点任务实飞。

仓库只包含程序、ROS 消息、配置、网页、RViz 配置、测试和实验说明。rosbag、运行日志、构建产物、缓存、现场数据库和备份文件均未上传。

## 已实现功能

- 双地面 GPS 中点投喂任务；
- A/B 作为矩形对角点的 N×M 网格中心航点和蛇形遍历；
- 初始化前强制选择 `GROUND_GPS`（双地面 GPS A/B）或 `PHOTO_COORDS`（照片/人工坐标）；
- 多大网箱数据库，每个大网箱支持 1～200 个小网箱、照片 EXIF、人工经纬度、坐标 CSV 和审核版本；
- 照片/人工坐标只采用经纬度，高度由网页参数设置；
- 快速照片任务可直接上传带 EXIF GPS 的 JPG，自动生成最近邻或人工顺序轨迹；
- 网页完成启动、自检、现场确认、预览、提交、录包和停止系统；
- H12 手动切换 `AUTO.MISSION`，CH5/E 右侧门控；
- PWM9 投喂器定时开关；
- 每个网格点可独立设置投喂时间和航点停留时间，要求投喂时间小于停留时间；
- 最后网格点结束后由状态机请求 `AUTO.RTL`，不再添加同坐标的 `GRID_EXIT`；
- 飞控任务上传前清除残留任务，上传后回读校验数量；
- 手动接管、GPS/FCU/投喂器检查和失败阻断。
- 网页实时显示飞控电量，并提供任意时刻可用的 `RETURN / 立即返航`；H12 接管优先级最高。

## 2026-11-07 实飞任务模式

11 月 7 日实飞采用 `PHOTO_COORDS + PHOTO_POINTS`，不是 `GPS_CENTER` 或 `GPS_GRID`：

- 从 `DJI_0195.JPG` 至 `DJI_0201.JPG` 的 EXIF WGS84 经纬度生成 7 个目标点；
- 照片高度不用于飞行，网页参数分别设置 10 m 起飞、8 m 过渡和 5 m 投喂相对高度；
- 飞手使用 H12 手动解锁并切换至 `AUTO.MISSION`；
- K9 飞控继续使用机载 GPS 执行航点，`PHOTO_COORDS` 只是不依赖两台外置地面 GPS A/B；
- 状态机在 7 个目标处依次发送投喂器开关命令；
- 最后目标完成后，系统请求并确认飞控进入 `AUTO.RTL`；
- 返航过程中飞手切回 `POSCTL` 接管，因此本次验证了自动返航请求与模式确认，但没有验证全自动降落。

Rosbag 能证明 GPS 轨迹、任务状态、航点和投喂控制命令，不能单独证明物料实际落下或真实落料量。

## 工作区准备

```bash
source /opt/ros/noetic/setup.bash
cd ~/cage_uav_ws
catkin_make
source devel/setup.bash
```

NX3 还需要已安装并可用的外部工作区：

- `/home/nx3/gps_ws`
- `/home/nx3/acfly_ws`

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/acfly_ws/devel/setup.bash
source ~/cage_uav_ws/devel/setup.bash

rosrun cage_realflight start_realflight_stack.sh \
  --input-profile photo_coords \
  --geometry circular \
  --feeding true \
  --rviz true \
  --web true
```

使用双地面 GPS 时改为 `--input-profile ground_gps`，并额外加载 `~/gps_ws/devel/setup.bash`。网页监督器会在每次启动前要求重新选择，运行期间不能热切换。

启动后访问 NX3 的 `8080` 端口，例如：

```text
http://NX3_IP:8080/
```

## 测试

```bash
cd ~/cage_uav_ws
source /opt/ros/noetic/setup.bash
source ~/acfly_ws/devel/setup.bash
source devel/setup.bash
catkin_make run_tests
catkin_test_results build/test_results
```

2026-11-23 使用 NX3 当前部署源码重新验证：87 项测试通过，0 errors，0 failures。NX3 `/home/nx3/cage_uav_ws/src` 与本仓库对应源码经过逐文件校验一致；实验结果、rosbag、数据库、缓存和构建产物不属于源码同步范围。

## 实飞安全

本工程不会自动解锁飞机。任务上传、H12 模式切换、现场坐标确认、起飞区域清空、飞手接管验证和 rosbag 录制必须按网页检查项执行。任何 `start_blockers` 未清空时均不得起飞。

详细流程见：

- `src/cage_realflight/PHOTO_COORDS_WEB_WORKFLOW.md`
- `src/cage_realflight/QUICK_PHOTO_MISSION_WORKFLOW.md`
- `src/cage_realflight/WEB_FULL_SYSTEM_TEST.md`
- `src/cage_realflight/THREE_MODE_WEB_TASKS_2026-10-30.md`
- `src/cage_realflight/REALFLIGHT_EXPERIMENT_QUICKSTART.md`

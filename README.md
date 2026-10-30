# CAGE UAV NX3 实飞系统

这是从 NX3 实飞计算机 `/home/nx3/cage_uav_ws/src` 独立整理的 ROS Noetic Catkin 工程源码，快照日期为 2026-10-30。

仓库只包含程序、ROS 消息、配置、网页、RViz 配置、测试和实验说明。rosbag、运行日志、构建产物、缓存、现场数据库和备份文件均未上传。

## 已实现功能

- 双地面 GPS 中点投喂任务；
- A/B 作为矩形对角点的 N×M 网格中心航点和蛇形遍历；
- DJI 照片 GPS 点任务，只采用经纬度，高度由网页参数设置；
- 网页完成启动、自检、现场确认、预览、提交、录包和停止系统；
- H12 手动切换 `AUTO.MISSION`，CH5/E 右侧门控；
- PWM9 投喂器定时开关；
- 每个网格点可独立设置投喂时间和航点停留时间，要求投喂时间小于停留时间；
- 最后网格点结束后由状态机请求 `AUTO.RTL`，不再添加同坐标的 `GRID_EXIT`；
- 飞控任务上传前清除残留任务，上传后回读校验数量；
- 手动接管、GPS/FCU/投喂器检查和失败阻断。

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
source ~/gps_ws/devel/setup.bash
source ~/acfly_ws/devel/setup.bash
source ~/cage_uav_ws/devel/setup.bash

rosrun cage_realflight start_realflight_stack.sh \
  --geometry circular \
  --feeding true \
  --rviz true \
  --web true
```

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

2026-10-30 在 NX3 上验证结果：75 项测试通过，0 errors，0 failures。

## 实飞安全

本工程不会自动解锁飞机。任务上传、H12 模式切换、现场坐标确认、起飞区域清空、飞手接管验证和 rosbag 录制必须按网页检查项执行。任何 `start_blockers` 未清空时均不得起飞。

详细流程见：

- `src/cage_realflight/WEB_FULL_SYSTEM_TEST.md`
- `src/cage_realflight/THREE_MODE_WEB_TASKS_2026-10-30.md`
- `src/cage_realflight/REALFLIGHT_EXPERIMENT_QUICKSTART.md`

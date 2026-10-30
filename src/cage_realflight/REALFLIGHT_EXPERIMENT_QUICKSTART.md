# NX3 GPS + 航点 + PWM9 投喂完整测试说明书

以下命令全部在 NX3 上执行。首次联调先拆桨；装桨实飞前必须已经完成无桨模式切换和投喂测试。

## 1. 上电和开关位置

- 飞控连接 `/dev/ttyUSB0`，地面 GPS 接收机连接 `/dev/ttyUSB1`。
- GPS A、GPS B、接收机、飞控、H12 和投喂器上电。
- 投喂器信号接飞控 PWM9并共地，`Aux_9Func=0`。
- 飞机保持未解锁；H12 E 开关先放中间或左边，使飞控处于 `POSCTL` 或 `ALTCTL`。
- 不要同时运行旧的 `roslaunch uav_bridge mission_auto.launch`。

检查设备：

```bash
ls -l /dev/ttyUSB0 /dev/ttyUSB1
```

如果旧航点节点仍在运行，先停止它：

```bash
rosnode list | grep mission_auto_bridge
rosnode kill /mission_auto_bridge
```

## 2. 终端1：启动完整系统

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/gps_ws/devel/setup.bash
source /home/nx3/acfly_ws/devel/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
export PYTHONPATH="/home/nx3/gps_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/home/nx3/gps_ws/devel/lib:${LD_LIBRARY_PATH:-}"

rosrun cage_realflight start_realflight_stack.sh \
  --geometry circular \
  --feeding true \
  --rviz true \
  --web true
```

该命令启动或复用地面 GPS、MAVROS、圆形网箱定位、航点规划、任务状态机、PWM9 投喂器、安全检查和网页接口。程序不会自动解锁。

## 3. 终端2：检查硬件和系统状态

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/gps_ws/devel/setup.bash
source /home/nx3/acfly_ws/devel/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
export PYTHONPATH="/home/nx3/gps_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/home/nx3/gps_ws/devel/lib:${LD_LIBRARY_PATH:-}"
```

确认 GPS A、B 和中心坐标都有新数据：

```bash
rostopic echo -n 1 /farm/scene_1/anchor_A/fix
rostopic echo -n 1 /farm/scene_1/anchor_B/fix
rostopic echo -n 1 /farm/scene_1/center_fix
```

确认飞控、遥控器和 E/CH5 数值：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/rc/in
```

CH5 是 `channels[4]`。实测中/左约为 1489/1137，右档约为 1841。

确认 AUTO 门控和 PWM9 参数：

```bash
rosparam get /vehicle/require_auto_permission_switch
rosparam get /vehicle/auto_permission_channel_index
rosparam get /vehicle/auto_permission_threshold
rosparam get /feeding/pwm_output_channel
rosparam get /feeding/pwm_on
rosparam get /feeding/pwm_off
```

预期依次为 `true`、`4`、`1650`、`9`、`1841`、`1137`。

运行总体检查：

```bash
rosrun cage_realflight start_realflight_stack.sh \
  --geometry circular \
  --feeding true \
  --rviz false \
  --web true \
  --check-only

curl -s http://127.0.0.1:8080/api/status | python3 -m json.tool
```

此时飞控模式必须是 `POSCTL` 或 `ALTCTL`。如果不是，先用 H12 切换。

## 4. 无桨投喂器测试

确认飞机未解锁后，让投喂器打开2秒并自动关闭：

```bash
rosservice call /feeder/run_for_duration "duration_sec: 2.0"
rostopic echo -n 1 /feeder/status
```

服务应返回 `success: true`，动作结束后的 `/feeder/status` 应为 `data: false`。

## 5. 确认现场

只有在 GPS A/B 固定、坐标正确、起飞区清空、H12接管已经验证且飞手就位后执行：

```bash
curl -s -X POST http://127.0.0.1:8080/api/realflight/confirm-site \
  -H 'Content-Type: application/json' \
  -d '{"anchors_fixed":true,"takeoff_area_clear":true,"h12_takeover_verified":true,"pilot_ready":true,"site_coordinates_verified":true}'
```

再次检查：

```bash
curl -s http://127.0.0.1:8080/api/status | python3 -m json.tool
```

必须看到 `"start_ready": true` 且 `"start_blockers": []`。

## 6. 只预览航点，不解锁

```bash
curl -s -X POST http://127.0.0.1:8080/api/preview \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"flight-test-01","cage_id":0,"repeat_count":1,"feed_relative_alt":15.0,"transit_relative_alt":25.0}'

rostopic echo -n 1 /mission/waypoints
```

必须人工核对三个规划点：

- `TAKEOFF`：当前位置，相对高度默认20m；
- `TRANSIT`：网箱中心，相对高度25m；
- `FEED_POINT`：网箱中心，相对高度15m，停留时间默认10秒。

坐标、高度或中心点不正确时禁止继续。

## 7. 提交任务并检查上传结果

保持飞机未解锁、E开关中间或左边：

```bash
curl -s -X POST http://127.0.0.1:8080/api/start \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"flight-test-01","cage_id":0,"repeat_count":1,"feed_relative_alt":15.0,"transit_relative_alt":25.0}'

rostopic echo /mission/state
```

等待状态进入 `WAIT_ARM` 后按 `Ctrl-C` 停止查看，再检查飞控收到的任务：

```bash
rostopic echo -n 1 /mavros/mission/waypoints
```

正常应包含 HOME、TAKEOFF、TRANSIT、FEED_POINT。上传失败、数量不符或坐标不符时禁止解锁。

`WAIT_ARM` 默认超时为60秒。实飞时应在进入 `WAIT_ARM` 并完成快速回读核对后，于60秒内按下一节完成解锁和E右授权；仅做上传测试而不解锁时，超时进入 `FAILSAFE: WAIT_ARM_TIMEOUT` 属于预期保护，下一次任务前需要安全重启完整栈。

## 8. 解锁和开始自动任务

1. 保持 E 中/左，在 `POSCTL/ALTCTL` 下由飞手解锁。
2. 解锁后将 E 拨到右档；只有 CH5 大于1650时，NX3才允许请求 `AUTO.MISSION`。
3. 同时观察：

```bash
rostopic echo /mavros/state
```

另一个终端观察：

```bash
rostopic echo /mission/state
```

必须看到飞控模式变为 `AUTO.MISSION`，任务状态由 `WAIT_ARM` 进入 `TAKEOFF` 或 `TRANSIT`。随后飞机到中心点，在配置的停留时间内打开投喂器，关闭后请求 RTL。

可监控投喂器和航点到达：

```bash
rostopic echo /feeder/status
rostopic echo /mavros/mission/reached
```

## 9. 人工接管和异常处置

空中需要终止自动任务时，第一动作是把 H12 E 开关拨回中间或左边，并由飞手接管。预期状态：

```text
/mavros/state.mode = POSCTL 或 ALTCTL
/mission/state.state = MANUAL_OVERRIDE
/feeder/status.data = false
```

检查命令：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mission/state
rostopic echo -n 1 /feeder/status
```

不要在空中关闭 ROS，也不要把 `/api/cancel` 当作飞行中止命令。人工接管后系统不会自动重新进入 AUTO；由飞手手动返航或使用 H12 的 RTL 功能。

## 10. 落地后停止

确认已经落地且 `armed: false`：

```bash
rostopic echo -n 1 /mavros/state
```

再停止完整系统：

```bash
rosrun cage_realflight stop_realflight_stack.sh
```

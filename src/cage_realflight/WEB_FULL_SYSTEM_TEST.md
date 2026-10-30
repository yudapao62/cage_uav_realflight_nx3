# GPS + 航点 + PWM9 投喂网页实飞说明

更新时间：2026-10-21

## 1. 网页入口

NX3 上电并接入同一网络后打开：

```text
http://192.168.1.91:8080/
```

常驻网页只负责启动和观察软件，不会解锁飞控。页面显示“系统未启动”时，点击“启动 GPS + MAVROS + 航点 + 投喂系统”。启动失败原因会直接显示在网页日志中。

## 2. 起飞前网页流程

1. 确认 FCU、UAV GPS、RC、GPS A/B、GEOMETRY、FEEDER 均为 `OK`。
2. H12 保持 `POSCTL` 或 `ALTCTL`，飞机保持未解锁。
3. 完成五项现场确认：GPS A/B 固定、起飞区清空、H12 接管验证、飞手就位、坐标核对。
4. 填写任务参数。当前建议初值：
   - 起飞相对高度：`10 m`
   - 过渡相对高度：`8 m`
   - 投喂相对高度：`5 m`
   - 投喂持续时间：`10 s`
   - 重复次数：`1`
5. 点击“生成预览”，人工核对 `TAKEOFF → TRANSIT → FEED_POINT` 的坐标、高度、半径和停留时间。
6. 点击“航点核对无误”。修改任何参数后必须重新预览并确认。
7. 点击“开始录制”，必须看到“正在录制”和实际目录后才能提交任务。
8. `START READY: YES` 后点击“提交实飞任务”。
9. 飞手使用 H12 解锁，再将 CH5/E 切到右档授权 `AUTO.MISSION`。

## 3. 飞行与返航判据

完整状态链：

```text
WAIT_ARM → TAKEOFF → TRANSIT → FEEDING_DWELL → RTL → LAND → FINISHED
```

投喂结束时 NX3 先关闭 PWM9 投喂器，再请求 `AUTO.RTL`。最多请求 3 次、请求间隔 1 秒；只有 `/mavros/state.mode` 实际变为 `AUTO.RTL`，网页才显示 `RTL_MODE_CONFIRMED`。

如果出现 `RTL_MODE_NOT_CONFIRMED_AFTER_3_ATTEMPTS`，立即由 H12 人工接管并返航。只有正确模式名经过实飞仍不稳定时，才进入增加 `RETURN_HOME` 航点的备用整改，不在本轮直接混用两套返航逻辑。

任何异常都应立即将 H12 切到 `POSCTL` 或 `ALTCTL`。人工接管一旦触发，NX3 会锁定让权，不会自动抢回控制权。

## 4. 落地后

飞机落地并确认已经上锁后，点击“停止并保存”。飞机处于解锁状态时，网页拒绝停止 rosbag。

记录目录：

```text
/home/nx3/cage_uav_ws/bags/realflight_YYYYMMDD_HHMMSS/
```

目录包含 `realflight.bag`、`rosparams.yaml`、`status_before.json`、`status_after.json` 和 `rosbag.log`。

## 5. 当前到达阈值

| 判据 | 当前值 | 配置文件 |
|---|---:|---|
| TAKEOFF/TRANSIT 接受半径 | 3.0 m | `config/mission_real.yaml` |
| FEED_POINT 接受半径 | 1.5 m | `config/realflight_circle_test.yaml` |
| 正常投喂高度容差 | ±3.0 m | `config/realflight_circle_test.yaml` |
| current_seq 兜底水平半径 | 5.0 m | `config/realflight_circle_test.yaml` |
| current_seq 兜底高度容差 | ±5.0 m | `config/realflight_circle_test.yaml` |

本轮没有继续放宽阈值。投喂兜底仍要求飞机已解锁、模式为 `AUTO.MISSION`、UAV GPS 新鲜、几何有效且没有人工接管。

## 6. 终端只读检查（备用）

```bash
curl -s http://127.0.0.1:8080/api/supervisor/status | python3 -m json.tool
curl -s http://127.0.0.1:8080/api/status | python3 -m json.tool
rostopic echo /mission/state
rostopic echo -n 1 /mission/waypoints
rostopic echo -n 1 /mavros/mission/waypoints
rostopic echo /mavros/state
```

网页飞行栈未启动时，第二条 `/api/status` 会返回 `CORE_UNAVAILABLE`，属于正常现象。

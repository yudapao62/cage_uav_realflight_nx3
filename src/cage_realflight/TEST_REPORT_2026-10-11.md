# 无人机航点投喂系统阶段测试简报

日期：2026-10-11  
平台：NX3 + K9/ACFly + H12 + 双地面GPS + PWM9投喂器

## 1. 测试结论

已完成从地面GPS定位、圆形网箱中心计算、任务规划、MAVLink Mission上传、H12自动模式授权到PWM9投喂控制的整链路地面验证。飞控成功回读 HOME、TAKEOFF、TRANSIT、FEED_POINT 四个任务项；任务状态机成功进入 WAIT_ARM。整个验证过程飞机保持未解锁，投喂器保持关闭。

## 2. 昨日单航点起飞实飞

2026-10-10 已完成单航点起飞实飞。使用 `roslaunch uav_bridge mission_auto.launch` 上传任务并进入 `AUTO.MISSION`，验证了 K9/ACFly 对 MAVROS Mission 清除、上传、回读、设置当前航点及 AUTO.MISSION 执行链路的实机兼容性；H12模式开关可切回人工控制。

## 3. 今日完成内容

- 将投喂器信号接入飞控 PWM9，释放 `Aux_9Func=0`。
- PWM9开启值为1841，关闭值为1137；定时地面投喂服务测试通过。
- 完成 GPS、网箱几何、航点规划、Mission FSM、投喂任务、Web接口和真实投喂器的一键启动集成。
- 修复多工作空间叠加后 `gps_receiver/AnchorFix` 消息类无法加载的问题。
- 修复 MAVROS 启动瞬间误判未连接以及启动失败未完整清理后台进程的问题。
- 根据双GPS约0.5 Hz、A/B错相约1.6秒的实测链路，将配对窗口调整为2.5秒、接收超时调整为3.5秒。
- 增加 H12 E/CH5 自动模式门控：只有 `channels[4] > 1650` 才允许请求 `AUTO.MISSION`。
- 增加GPS几何门控：几何无效时不请求AUTO；连续丢失超过5秒才进入失效保护，单次短暂抖动不误触发。
- FSM单元测试16项全部通过，无错误、无失败。
- 完成现场确认、航点预览、任务上传和飞控任务回读测试。

## 4. 关键实测 Topic Message

### 4.1 地面GPS A

Topic：`/farm/scene_1/anchor_A/fix`

```yaml
scene_id: 1
anchor_id: "A"
lat: 27.58243989
lon: 120.56654456
altitude: 7.46
valid: True
satellites: 41
seq: 1288
stamp: "2026-09-23 14:28:29.422"
```

### 4.2 地面GPS B

Topic：`/farm/scene_1/anchor_B/fix`

```yaml
scene_id: 1
anchor_id: "B"
lat: 27.58233649
lon: 120.56640357
altitude: 22.25
valid: True
satellites: 20
seq: 1258
stamp: "2026-09-20 17:48:50.282"
```

### 4.3 双GPS中心

Topic：`/farm/scene_1/center_fix`

```yaml
scene_id: 1
anchor_id: "CENTER"
lat: 27.58238728580211
lon: 120.56647320220301
altitude: 14.965751358193144
valid: True
satellites: 20
seq: 258
stamp: "2026-10-11 18:58:32.790"
```

### 4.4 圆形网箱几何

Topic：`/cage/geometry`

```yaml
cage_id: 0
geometry_mode: "circular"
endpoint_a_lat: 27.582439065000003
endpoint_a_lon: 120.56654722
endpoint_a_alt: 7.85
endpoint_b_lat: 27.582338245000003
endpoint_b_lon: 120.566399435
endpoint_b_alt: 22.17
center_lat: 27.582388654963
center_lon: 120.56647332738332
center_alt: 15.009993370622396
diameter_m: 23.29826277205251
radius_m: 11.649131386026255
```

### 4.5 H12输入

Topic：`/mavros/rc/in`

```yaml
rssi: 255
channels: [1489, 1489, 1489, 1489, 1489, 1489, 1841, 1841, 1137, 1137, 1817, 1399, 1000, 1000, 1000, 1000]
```

其中E/CH5为 `channels[4]=1489`，对应任务前人工模式位置。

### 4.6 飞控状态

Topic：`/mavros/state`

```yaml
connected: True
armed: False
guided: False
manual_input: True
mode: "POSCTL"
system_status: 3
```

### 4.7 规划航点

Topic：`/mission/waypoints`

```yaml
mission_id: "PREVIEW"
cage_id: 0
waypoints:
  - seq: 0
    lat: 27.582543800000003
    lon: 120.56641190000002
    alt: 20.0
    hold_time: 0.0
    waypoint_type: "TAKEOFF"
  - seq: 1
    lat: 27.582383890163378
    lon: 120.56647892988778
    alt: 25.0
    hold_time: 0.0
    waypoint_type: "TRANSIT"
  - seq: 2
    lat: 27.582383890163378
    lon: 120.56647892988778
    alt: 15.0
    acceptance_radius: 1.5
    hold_time: 10.0
    waypoint_type: "FEED_POINT"
```

### 4.8 飞控回读的MAVLink Mission

Topic：`/mavros/mission/waypoints`

```yaml
current_seq: 0
waypoints:
  - frame: 0
    command: 16
    is_current: True
    autocontinue: True
    param1: 0.0
    param2: 0.0
    x_lat: 27.5825432
    y_long: 120.5664126
    z_alt: 15.374093055725098
  - frame: 3
    command: 22
    is_current: False
    autocontinue: True
    param1: 0.0
    param2: 3.0
    x_lat: 27.5825432
    y_long: 120.5664126
    z_alt: 20.0
  - frame: 3
    command: 16
    is_current: False
    autocontinue: True
    param1: 0.0
    param2: 3.0
    x_lat: 27.5823838
    y_long: 120.5664789
    z_alt: 25.0
  - frame: 3
    command: 16
    is_current: False
    autocontinue: True
    param1: 10.0
    param2: 1.5
    x_lat: 27.5823838
    y_long: 120.5664789
    z_alt: 15.0
```

说明：`command: 22` 为起飞，最后一个 `command: 16` 的 `param1: 10.0` 为目标点停留10秒，`param2: 1.5` 为接受半径1.5米。

### 4.9 任务状态

上传成功后的 Topic：`/mission/state`

```yaml
mission_id: "feed-flight-test-01-r1-1791715882"
state: "WAIT_ARM"
reason: ''
current_waypoint: -1
feeder_enabled: False
manual_override: False
```

本次只做上传测试，没有解锁。超过60秒后按设计进入：

```yaml
state: "FAILSAFE"
reason: "WAIT_ARM_TIMEOUT"
current_waypoint: -1
feeder_enabled: False
manual_override: False
```

### 4.10 投喂器状态

Topic：`/feeder/status`

```yaml
data: False
```

## 5. 当前状态与后续测试

本次地面测试完成，飞机未解锁、未起飞，投喂器关闭。`WAIT_ARM_TIMEOUT` 是未在60秒内人工解锁产生的预期保护，并非任务上传失败。

下一阶段为整链路实飞：重新启动系统并确认现场，预览及上传任务后，在60秒内由飞手解锁，再将H12 E开关拨到右档授权 `AUTO.MISSION`；到达 FEED_POINT 后自动投喂10秒、关闭投喂器并进入RTL。实飞前必须再次人工核对A/B位置、中心坐标、实际网箱直径和全部任务高度。

# 无双地面 GPS 的照片/坐标实飞与多网箱数据库改造方案

版本：V1.1
日期：2026-11-05  
目标仓库：`yudapao62/cage_uav_realflight_nx3`  
状态：待确认，确认后严格按本文实施

## 1. 术语和必须明确的边界

本文中的：

- **有 GPS 模式**：使用原来的两台外置地面 GPS A/B；
- **无 GPS 模式**：不连接、不启动、不检查两台外置地面 GPS A/B，任务坐标来自大疆照片 EXIF 或人工输入；
- **飞控 GPS**：K9/ACFly 飞控自身的机载 GPS。

“无 GPS 模式”只表示**没有外置 A/B 地面 GPS**。实飞、全球航点和 `AUTO.RTL` 仍然必须依赖飞控自身 GPS；飞控 GPS 无定位时禁止上传并禁止起飞。

## 2. 本次改造目标

本次建立两套并行且互相隔离的实飞初始化配置：

1. `GROUND_GPS`：保留现有双地面 GPS 中点和 A/B 对角网格任务；
2. `PHOTO_COORDS`：不使用外置 A/B，目标点来自照片 GPS 或人工经纬度数据库。

本次现场主要使用 `PHOTO_COORDS`：

1. 用大疆飞机飞到小网箱上空拍照；
2. 将 JPG/JPEG 上传到系统；
3. 系统读取每张照片的 WGS84 纬度、经度；
4. 照片绑定到具体大网箱下的小网箱；
5. 人工审核坐标并保存为版本化投喂点；
6. 在网页选择一个或多个小网箱；
7. 使用 K9 当前飞控 GPS 冻结为本次任务 HOME；
8. 生成并人工核对航点；
9. 上传飞控后由飞手使用 H12 手动切入 `AUTO.MISSION`；
10. 完成投喂后自动请求 `AUTO.RTL` 返回起飞点。

选择一个目标时就是“原点起飞 → 目标点 → 自动返航”的单点任务；选择多个目标时按规划顺序执行后自动返航。

## 3. 初始化前选择输入模式

### 3.1 启动页

系统核心进程启动前，`8080` 启动页必须先显示两个互斥选项：

- **双地面 GPS A/B**：`GROUND_GPS`；
- **照片/人工坐标，不使用地面 GPS**：`PHOTO_COORDS`。

本次现场默认推荐 `PHOTO_COORDS`，但不得静默沿用上次选择。每次系统停止后重新启动，必须再次人工选择。

点击“启动系统”时，网页提交：

```json
{
  "input_profile": "PHOTO_COORDS",
  "feeding": true,
  "rviz": false
}
```

系统启动后输入模式锁定。运行期间不得从一个模式热切换到另一个模式；必须先确认飞机未解锁、停止系统，再重新初始化。

### 3.2 启动脚本行为

`start_realflight_stack.sh` 新增：

```text
--input-profile ground_gps|photo_coords
```

`GROUND_GPS`：

- 检查 `/dev/ttyUSB1`；
- 启动 `gps_receiver`；
- 等待 GPS A/B 话题；
- 启动 `beacon_gps_adapter` 和地面 GPS 几何链路；
- 保留现有中点和 A/B 对角网格功能。

`PHOTO_COORDS`：

- 不要求 `/dev/ttyUSB1` 存在；
- 不启动 `gps_receiver`；
- 不等待 A/B 话题；
- 不启动 `beacon_gps_adapter`；
- 不把 A/B、基线或地面几何加入启动阻塞项；
- 仍启动 MAVROS、任务状态机、投喂器、网页、坐标数据库和必要的可视化节点。

启动状态必须显示当前输入模式，并写入只读运行参数：

```text
/realflight/input_profile = GROUND_GPS | PHOTO_COORDS
```

## 4. 自检和安全门控

### 4.1 两种模式共同必须通过

- `FCU` 已连接；
- 飞控自身 `UAV_GPS` 有效且新鲜；
- `/mavros/battery` 电量遥测可读取；若飞控不提供百分比，网页必须明确显示“未知”，不得用 0% 或假数据代替；
- H12/RC 输入有效；
- 起飞前模式为 `POSCTL` 或 `ALTCTL`；
- `auto_arm=false`；
- 无仿真节点和冲突的 OFFBOARD/setpoint 发布者；
- 真实投喂时 PWM9 和投喂状态有效；
- 状态机处于可开始状态；
- 现场确认完成；
- 航点预览已确认；
- rosbag 已开始录制；
- 飞机未解锁时才允许停止系统。

### 4.2 `GROUND_GPS` 额外要求

- GPS A、GPS B 健康；
- A/B 几何有效；
- 中点或网格路线有效。

### 4.3 `PHOTO_COORDS` 额外要求

- 已选择一个大网箱；
- 所有任务目标都属于该大网箱；
- 所有目标都有当前有效的已审核坐标版本；
- 照片/人工坐标路线有效；
- 冻结预览所引用的坐标版本没有变化。

在 `PHOTO_COORDS` 下，页面应把 `GROUND_GPS_A`、`GROUND_GPS_B`、`GEOMETRY` 显示为 `SKIPPED`，不能显示为失败，也不能阻塞下一步。

现场确认项按模式变化：

- `GROUND_GPS` 显示“GPS A/B 已固定”；
- `PHOTO_COORDS` 改为“大网箱、小网箱和坐标版本已核对”；
- 两种模式都保留起飞区清空、H12 接管验证、飞手就位和坐标核对。

## 5. 照片和人工坐标输入

### 5.1 照片输入

只接收 JPG/JPEG，并执行：

1. 校验文件类型、大小和 SHA256；
2. 使用 ExifTool 读取 `GPSLatitude` 和 `GPSLongitude`；
3. 验证 WGS84 范围，拒绝空坐标和 `(0,0)`；
4. 保存原图、缩略图、原始 EXIF 和审计信息；
5. 将照片绑定到一个小网箱；
6. 生成待审核坐标；
7. 人工批准后生成新的坐标版本。

照片中的高度、海拔基准和 `GPSAltitude` 只保留为证据，**不得进入任务航点 Z 值**。

### 5.2 人工直接输入 GPS

现有系统没有独立、清晰的手工坐标录入入口，需要新增。

网页字段：

- 大网箱；
- 小网箱；
- 纬度；
- 经度；
- 审核人；
- 说明。

人工输入先创建 `MANUAL_WGS84` 候选点，必须再次点击“审核并启用”后才能用于实飞。审核时允许修正纬度、经度，并生成新版本；历史版本不可覆盖或删除。

### 5.3 批量坐标导入

考虑每个大网箱有 100～200 个小网箱，增加“坐标表 CSV”导入，但它不是 DJI 航迹。

固定列：

```csv
small_cage_label,latitude,longitude,row_index,col_index,note
A-001,27.12345678,120.12345678,1,1,photo-20261105
```

导入结果全部先进入待审核状态。出现重复标签、无效经纬度、跨大网箱或缺失字段时，必须逐行报告，禁止静默跳过。

## 6. 删除 DJI 航迹功能

本次不再需要上传 DJI Fly `DJIFlightRecord_*.txt`，也不需要航迹回放。

程序修改要求：

- 删除网页的 DJI TXT 上传入口；
- 删除“标准航迹 CSV”入口；
- 删除地图中的 DJI 实测航迹图层和回放滑块；
- 删除新请求入口 `/api/site-db/imports/dji-txt`；
- 删除把 CSV 当航迹解析的入口；
- 停止使用 `dji_txt_adapter.py`、航迹分段和航迹回放代码；
- 保留照片点、已审核点和任务规划路线显示。

旧数据库中的航迹表和历史数据不主动删除，迁移后标记为遗留数据，程序不再读取或展示，避免升级时丢失已有数据。

## 7. 高度和原有可调参数

照片和人工坐标只提供 XY：

- X/Y：WGS84 纬度、经度；
- Z：完全使用网页参数，不读取照片高度。

以下现有参数必须保留且继续可调：

- 起飞相对高度 `takeoff_relative_alt`；
- 过渡相对高度 `transit_relative_alt`；
- 投喂相对高度 `feed_relative_alt`；
- 投喂器开启时间 `feeding_dwell_sec`；
- 每个目标点停留时间 `waypoint_hold_sec`；
- 重复次数 `repeat_count`；
- 航点速度；
- 航点接受半径；
- 最大航段距离。

网格/数据库点任务继续强制：

```text
feeding_dwell_sec < waypoint_hold_sec
```

## 8. 无双地面 GPS 的航点任务

### 8.1 HOME

生成预览时读取 K9 飞控当前 GPS，冻结为本次任务 HOME。人工确认预览后，提交必须使用同一份冻结 HOME 和目标点，不得因实时 GPS 漂移重新规划。

### 8.2 航点序列

选择一个或多个已审核小网箱后生成：

1. `TAKEOFF`：当前 HOME，经纬度为起飞点，相对高度为网页起飞高度；
2. `TRANSIT`：第一个目标 XY，使用过渡高度；
3. `GRID_ENTER`：第一个目标 XY，下降到投喂高度；
4. 一个或多个 `CELL_TARGET`：目标 XY，全部使用网页投喂高度；
5. 最后一个目标停留完成后由状态机请求 `AUTO.RTL`。

不得增加与最后目标同坐标、零停留的 `GRID_EXIT`。

单目标时即为：

```text
HOME → TAKEOFF → TRANSIT → CELL_TARGET → AUTO.RTL
```

多目标默认从 HOME 开始使用最近邻顺序；网页必须允许人工调整顺序。最终执行顺序以人工确认的冻结预览为准。

### 8.3 投喂和返航

每个目标点：

1. 到达目标并通过水平/高度阈值；
2. 投喂器开启网页设定的 N 秒；
3. 投喂器关闭；
4. 继续悬停到航点停留时间结束；
5. 前往下一目标。

最后目标完成后：

- 强制关闭投喂器；
- FSM 进入 `RTL`；
- 请求 `AUTO.RTL`；
- 必须等待 `/mavros/state` 确认 RTL；
- 多次请求仍无法确认时进入错误/恢复流程，不能显示任务成功。

H12 切入 `AUTO.MISSION` 仍由飞手手动完成，程序不自动切入任务模式；自动返航请求保持程序控制。

### 8.4 全局 RETURN / 立即返航

实飞控制网页顶部增加始终可见的 `RETURN / 立即返航` 按钮。该按钮不依赖任务是否已经预览、上传、开始或正在投喂，也不受地面 GPS、数据库、rosbag、航点完成度和普通 `start_blockers` 限制。

飞机已解锁时按下 RETURN，必须按以下顺序执行：

1. 立即发送投喂器关闭命令；
2. 将本次任务标记为人工放弃，禁止继续执行剩余航点和禁止自动恢复；
3. FSM 进入独立的 `USER_RETURN`/返航请求状态；
4. 请求飞控切换 `AUTO.RTL`；
5. 读取 `/mavros/state` 确认飞控确实进入 `AUTO.RTL`；
6. 在网页持续显示“请求中、已确认、被手控接管、请求失败”之一；
7. 飞机落地并解除锁定后再清除飞控残留任务。

RETURN 必须是幂等操作：重复点击不能创建多份任务、不能重新打开投喂器，也不能破坏已经确认的 RTL。

飞机未解锁时按下 RETURN：关闭投喂器、取消和作废待执行任务、清除允许起飞提示，不发送无意义的飞行模式命令。

FCU 断连或飞控 GPS 无效时，系统无法保证自动返航。网页必须立即显示明确失败原因并提示飞手手动接管，禁止显示“返航成功”。

控制优先级固定为：

```text
H12 手动接管 > 网页 RETURN > 普通自动任务
```

网页 RETURN 可以主动打断普通任务并请求 RTL；但飞手随后通过 H12 切回 `POSCTL`、`ALTCTL` 或其他手控模式时，视为最高优先级接管。程序必须立即停止重复发送 RTL/任务模式请求、保持投喂器关闭，并锁定为 `MANUAL_OVERRIDE`，不得与飞手抢夺模式控制权。

普通“取消任务”与 RETURN 含义不同：取消按钮只取消未起飞任务；飞机已解锁或任务已运行时，网页必须引导使用 RETURN 或 H12 接管。

## 9. 多大网箱数据库

现有 `sites` 表继续保留，业务含义和网页名称统一改为“**大网箱**”，避免破坏已有数据库。

关系为：

```text
大网箱 sites（多个）
└── 小网箱 subcages（每个大网箱 1～200 个）
    ├── 照片绑定 photo_bindings
    ├── 坐标候选 candidate_points
    └── 已审核坐标版本 approved_point_versions
```

### 9.1 大网箱

记录：编号、名称、状态、备注、创建时间。网页支持创建、选择、停用，禁止删除仍有小网箱或任务记录的大网箱。

### 9.2 小网箱

记录：唯一 ID、所属大网箱、标签、行列编号、状态、备注。支持：

- 单个创建；
- 按前缀和数量批量创建，例如 `A-001`～`A-200`；
- CSV 批量创建；
- 在当前大网箱内搜索、分页和多选；
- 停用但不物理删除。

### 9.3 坐标版本

同一小网箱只能有一个当前生效版本，但保留全部历史版本。每个版本记录：

- 来源：`PHOTO_EXIF` 或 `MANUAL_WGS84`；
- 纬度、经度；
- 照片/样本证据；
- 审核人、审核时间和说明；
- 是否人工修改过原始坐标；
- 启用/停用状态。

### 9.4 任务快照

新增任务快照表，保存：

- 任务 ID；
- 输入模式；
- 大网箱 ID；
- HOME 快照；
- 小网箱 ID、坐标版本 ID、执行顺序；
- 起飞/过渡/投喂高度；
- 投喂时间、停留时间、速度、接受半径；
- 预览哈希；
- 提交、上传校验、开始、完成、RTL 和失败状态；
- 用户按下 RETURN 的时间、当时模式、电量、位置、RTL 请求次数、确认结果和是否被 H12 接管；
- 创建和更新时间。

任务提交后即使数据库坐标后来更新，本次任务仍使用已经人工确认的冻结版本。

### 9.5 数据库迁移

新增 `schema_migrations` 版本表和顺序迁移脚本。升级流程必须：

1. 自动备份 SQLite；
2. 事务内执行迁移；
3. 失败回滚；
4. 不删除旧照片、坐标版本、航迹表或历史任务；
5. 测试旧数据库可直接升级。

## 10. 网页结构

### 10.1 初始化页

- 先选择 `GROUND_GPS` 或 `PHOTO_COORDS`；
- 显示该模式会启动和跳过的硬件；
- 启动后显示锁定的模式；
- 系统停止后才允许重新选择。

### 10.2 坐标数据库页

改名为“网箱坐标数据库”，包含：

1. 大网箱管理；
2. 小网箱单个/批量创建；
3. 照片批量上传；
4. 照片 GPS 结果和缩略图；
5. 照片与小网箱绑定；
6. 人工经纬度录入；
7. 候选点审核和版本历史；
8. 已审核点地图；
9. 规划路线地图。

移除 DJI 航迹上传、轨迹图层和回放区域。

### 10.3 实飞控制页

`GROUND_GPS` 下显示原来的：

- 双 GPS 中点；
- 双 GPS A/B 对角网格。

`PHOTO_COORDS` 下显示：

- 大网箱选择；
- 已审核小网箱搜索和多选；
- 照片来源/人工来源标识；
- 航点顺序调整；
- 原有高度、时间和重复参数；
- RViz 风格 XY/Z 向上的航点预览；
- 当前航点、完成数量、投喂状态和 RTL 状态。

页面顶部固定状态栏必须始终显示：

- 电池剩余百分比；
- 总电压；
- 数据更新时间/是否过期；
- 低电量告警；
- FCU 连接、解锁状态和当前飞行模式；
- 常驻的 `RETURN / 立即返航` 按钮。

电量来自 `/mavros/battery`，不能根据电压自行伪造百分比。`percentage` 有效时显示 0～100%；无效时显示“未知”和实际电压。超过配置的遥测超时时间后显示“电量数据过期”。

保留现有低电量保护：低于 `/safety/low_battery_threshold`（默认 20%）且自动任务正在运行时，关闭投喂器并请求 RTL。若 H12 已接管，则只告警和保持投喂器关闭，不覆盖飞手模式。

RETURN 按钮必须在桌面和手机宽度下都固定可见。点击后立即执行，不因普通表单校验或页面局部错误失效；页面同时用醒目颜色显示返航确认状态。

不允许在 `PHOTO_COORDS` 页面选择双 GPS 任务，也不允许在 `GROUND_GPS` 页面通过切换任务来源绕过 A/B 自检。

## 11. 接口变更

计划新增或修改：

```text
GET  /api/supervisor/profiles
POST /api/supervisor/start
GET  /api/supervisor/status

POST /api/site-db/subcages/bulk
POST /api/site-db/imports/photos
POST /api/site-db/imports/coordinate-csv
POST /api/site-db/points/manual-candidate
POST /api/site-db/points/{id}/approve
GET  /api/site-db/large-cages/{id}/mission-points
GET  /api/site-db/subcages/{id}/point-versions

POST /api/preview
POST /api/realflight/confirm-preview
POST /api/start
POST /api/realflight/return-now
```

`/api/supervisor/start` 必须携带输入模式。`/api/preview` 和 `/api/start` 必须检查任务来源与已锁定输入模式一致。

`POST /api/realflight/return-now` 不需要任务参数，返回当前返航请求状态。它必须独立于 `/api/start` 的门控，并在 FCU 已连接时可打断所有普通任务。`GET /api/status` 增加：

```json
{
  "battery": {
    "percentage": 0.68,
    "voltage": 47.6,
    "fresh": true,
    "age_sec": 0.3,
    "low": false
  },
  "return_control": {
    "state": "IDLE|REQUESTED|RTL_CONFIRMED|MANUAL_OVERRIDE|FAILED|LANDED",
    "reason": "",
    "requested_at": null,
    "attempts": 0
  }
}
```

移除对外可用的：

```text
POST /api/site-db/imports/dji-txt
POST /api/site-db/imports/normalized-csv
GET  /api/site-db/sessions/*/tracks
GET  /api/site-db/tracks/*
```

## 12. 预计修改文件

主要修改范围：

- `src/cage_realflight/scripts/realflight_web_supervisor.py`
- `src/cage_realflight/scripts/start_realflight_stack.sh`
- `src/cage_realflight/scripts/realflight_precheck.py`
- `src/cage_realflight/launch/realflight.launch`
- `src/cage_realflight/launch/realflight_core.launch`
- `src/cage_web_bridge/scripts/web_bridge_node.py`
- `src/cage_web_bridge/scripts/mission_modes.py`
- `src/cage_web_bridge/www/index.html`
- `src/cage_mission_fsm/scripts/mission_fsm_node.py`
- `src/cage_mission_fsm/src/cage_mission_fsm/core.py`
- `src/cage_site_db/config/schema.sql`
- `src/cage_site_db/cage_site_db/db.py`
- `src/cage_site_db/cage_site_db/service.py`
- `src/cage_site_db/cage_site_db/importers/exif_importer.py`
- `src/cage_site_db/www/index.html`
- 对应配置、README 和测试文件。

根据依赖检查，停止使用或删除 DJI 航迹解析模块，但不得删除现场数据库中的旧数据。

## 13. 测试要求

### 13.1 单元测试

- 初始化模式参数验证；
- `PHOTO_COORDS` 不检查 `/dev/ttyUSB1`；
- `PHOTO_COORDS` 不等待 A/B 话题；
- `GROUND_GPS` 仍严格检查 A/B；
- EXIF 纬度、经度读取；
- EXIF 无 GPS、无效 GPS、重复照片处理；
- 照片高度不进入航点；
- 人工坐标候选、审核和版本化；
- 200 个小网箱批量创建和坐标导入；
- 单点和多点任务排序；
- 冻结 HOME 和冻结坐标版本；
- 投喂时间必须小于停留时间；
- 最后目标完成进入 RTL；
- 任意任务阶段按 RETURN 都先关闭投喂器、作废剩余任务并请求 RTL；
- RETURN 重复点击幂等；
- RETURN 后 H12 接管会立即停止 RTL 重试，且程序不再抢模式；
- 未解锁时 RETURN 只取消任务和关闭投喂，不发送飞行模式；
- FCU 断连、GPS 无效或 RTL 被拒绝时网页显示真实失败状态；
- 电池百分比、电压、数据新鲜度和低电量状态转换；
- 电池百分比未知时不显示伪造数值；
- 不生成重复 `GRID_EXIT`；
- 跨大网箱选择被拒绝；
- 模式与任务来源不一致被拒绝；
- 旧数据库迁移和回滚。

### 13.2 集成测试

在无外置 GPS 硬件条件下：

1. 选择 `PHOTO_COORDS`；
2. 系统成功启动；
3. 网页不出现 A/B 阻塞；
4. 上传带 EXIF GPS 的照片；
5. 绑定、审核并选择目标；
6. 预览包含 HOME、TAKEOFF、TRANSIT 和目标点；
7. 上传前清除飞控残留任务；
8. 上传后回读校验数量；
9. 状态进入 `WAIT_ARM` 后才允许提示飞手操作；
10. H12 手动切入 `AUTO.MISSION`；
11. 到点按设定时间投喂；
12. 最后目标完成后确认 `AUTO.RTL`；
13. rosbag 包含完整状态、航点、投喂和模式变化。

另做中途返航测试：分别在等待解锁、起飞、过渡飞行、目标投喂和多点任务中途按 RETURN，验证投喂器关闭、剩余任务放弃、RTL 被确认；RTL 后再用 H12 切回手控，验证程序立即停止 RTL 重试且不与飞手争夺控制权。

录包同时验证 `/mavros/battery` 与网页显示一致，断开/停止电量消息后网页能够显示数据过期。

同时回归 `GROUND_GPS` 模式，确保原来的双 GPS 中点和 A/B 网格功能没有被破坏。

## 14. 完成验收标准

只有满足以下条件才算完成：

- 无外置 A/B 和 `/dev/ttyUSB1` 时可以选择 `PHOTO_COORDS` 并正常初始化；
- 有 A/B 时原模式仍可用；
- 启动模式不能在运行中切换；
- DJI 航迹导入与回放已从网页和接口移除；
- 照片 EXIF 和手工经纬度都能生成待审核点；
- 任务只使用已审核坐标版本；
- 照片 Z 被忽略，航点 Z 使用网页参数；
- 支持多个大网箱，每个大网箱至少 200 个小网箱；
- 单点和多点任务均从当前 HOME 起飞并最终请求 RTL；
- 网页始终显示真实电量百分比/电压及数据新鲜度；
- 任何普通任务阶段均可通过 RETURN 放弃任务并请求返航；
- RETURN 的结果必须以飞控模式回读确认，不能只根据接口调用成功判定；
- H12 手动接管始终高于网页 RETURN 和所有自动任务，程序不得反复覆盖飞手模式；
- 预览确认后提交使用冻结计划；
- 上传飞控前清任务，上传后校验数量；
- 自动解锁仍为关闭；
- H12 切 `AUTO.MISSION` 仍为人工操作；
- 全部新增测试通过，原有测试无回归；
- README、网页操作说明和实飞测试清单同步更新。

## 15. 实施顺序

本文确认后按以下顺序修改 GitHub 仓库：

1. 建立数据库迁移和多大网箱/小网箱数据模型；
2. 完成照片 XY 和人工坐标录入、审核、版本化；
3. 移除 DJI 航迹入口和展示；
4. 改造初始化页和启动脚本的输入模式选择；
5. 改造 ROS launch 和自检条件；
6. 改造数据库点任务规划和冻结提交；
7. 增加电量状态和全局 RETURN/手控优先级链路；
8. 改造网页任务选择、地图、固定电量栏、RETURN 和进度显示；
9. 补齐单元与集成测试；
10. 在本地完成静态检查；
11. 提交并推送 GitHub；
12. 经确认后再 SSH 到 NX3 部署，不在本文确认前修改 NX3 运行环境。

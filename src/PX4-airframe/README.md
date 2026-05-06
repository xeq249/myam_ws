# PX4-airframe：自定义 Gazebo Classic 模型与仿真世界

本仓库提供多旋翼 + 机械臂等 SDF 模型及仿真世界文件。要在 **PX4 SITL + Gazebo Classic** 中使用，需把整个模型目录和资源放进 **PX4 固件仓库中的 `sitl_gazebo-classic`** 子模块目录（不是随便放在 `PX4-Autopilot` 根目录）。

以下路径以 **当前主线 PX4-Autopilot**（含子模块 `sitl_gazebo-classic`）为准。

---

## 1. 准备 PX4 与子模块

克隆或进入你的固件目录，并确保 Gazebo Classic 仿真资源已拉取：

```bash
cd ~/PX4-Autopilot   # 或你的 PX4-Autopilot 根路径
git submodule update --init --recursive Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

若该目录仍不存在，请先完成官方文档中的 [PX4 开发环境 / 子模块](https://docs.px4.io/main/en/dev_setup/dev_env.html) 配置。

**目标目录（请记准）：**

| 类型 | 路径（相对 `PX4-Autopilot` 根目录） |
|------|--------------------------------------|
| **模型**（每个模型一个子文件夹） | `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` |
| **世界**（`.world`） | `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/` |

等价绝对路径示例：`$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`。

> **旧版固件说明**：很老的 PX4 曾使用 `Tools/sitl_gazebo/models`。若你使用的是旧树，请对照自己仓库里实际存在的 `models`/`worlds` 路径；**当前官方主线请使用上表中的 `sitl_gazebo-classic`。**

---

## 2. 拷贝 SDF 模型目录

Gazebo 通过 **目录名** + **`model.config`** 识别模型：每个模型必须是 `models/<模型名>/` 下至少包含：

- `model.config`
- `*.sdf`（文件名需与 `model.config` 内 `<sdf>...</sdf>` 一致）

本仓库当前包含的模型文件夹为：

- `myam_sys/`
- `myam_sys_d435/`
- `myam_sys_no_arm/`
- `myuam/`

请**整目录复制**到 PX4 的 `models` 下（保留目录名不变），例如：

```bash
PX4_ROOT=~/PX4-Autopilot   # 改成你的路径
SRC=/path/to/PX4-airframe  # 改成本仓库 PX4-airframe 的路径

MODEL_DST="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"

cp -r "$SRC/myam_sys"        "$MODEL_DST/"
cp -r "$SRC/myam_sys_d435"   "$MODEL_DST/"
cp -r "$SRC/myam_sys_no_arm" "$MODEL_DST/"
cp -r "$SRC/myuam"           "$MODEL_DST/"
```

### 2.1 网格与其它资源（meshes 等）

若某 SDF 中包含 `model://模型名/meshes/...` 或相对路径的 `meshes/`、`materials/`，你必须把**这些子目录一并**放进对应模型文件夹，且与 SDF 中的引用一致。  
本仓库若未附带 `meshes/` 等二进制资源，需从你生成 SDF 的机器或备份中补全后再拷贝，否则 Gazebo 中会无法加载几何体。

---

## 3. 拷贝仿真世界（可选）

本仓库 `worlds/` 下例如：

- `myam_grasping.world`
- `apriltag.world`

复制到 PX4 的 `worlds` 目录：

```bash
WORLD_DST="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

cp "$SRC/worlds/"*.world "$WORLD_DST/"
```

在启动 SITL 时，通过 PX4/Gazebo 文档中说明的方式指定自定义 world（例如设置 `PX4_SITL_WORLD` 或通过 `make`/`px4` 仿真启动参数，具体以你使用的 PX4 版本脚本为准）。

---

## 4. 放置后自检

1. 确认下列路径存在且可读：
   - `$MODEL_DST/myam_sys/model.config`
   - `$MODEL_DST/myam_sys/myam_sys.sdf`  
   （其它模型同理）
2. 单独启动 Gazebo Classic 时，可选用环境变量增加模型路径（一般 PX4 会为 `sitl_gazebo-classic` 配置好）：
   ```bash
   export GAZEBO_MODEL_PATH=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}
   ```
3. 若模型仍找不到，检查 **`model.config` 里 `<name>`** 与 **目录名**、以及 SDF 内 `model://名称/...` 是否一致。

---

## 5. 本仓库内容一览（便于核对）

```
PX4-airframe/
├── README.md                 # 本说明
├── readme.txt                # 简短指向 README.md
├── myam_sys/                 # model.config + myam_sys.sdf
├── myam_sys_d435/
├── myam_sys_no_arm/
├── myuam/
└── worlds/                   # *.world
```

---

## 6. 参考链接

- [PX4 Gazebo Classic 仿真（官方）](https://docs.px4.io/main/en/sim_gazebo_classic/)
- [PX4-SITL_gazebo-classic 仓库 models](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/models)

# Dora-MoveIt2 + GEN72 双臂机器人精简课程（讲课原文）

# 第1章：Course Introduction & Environment Setup（课程介绍与环境搭建）

## 1.1 课程欢迎与目标

各位同学大家好，今天我们开始学习Dora-MoveIt2 双臂机器人控制的精简课程。和传统ROS2课程不同，我们用的是**dora-rs数据流框架**——一个更轻量、更现代的机器人框架，纯Python开发，不用编译C++，安装简单，十分钟就能跑通demo。

核心目标就一个——让大家能快速上手，用Dora-MoveIt2的MoveGroup API控制GEN72 7自由度机械臂完成运动规划和简单抓取，全程围绕实操，学完就能跑通demo。

## 1.2 课程适用人群与前置要求

适用人群：做机器人研发、学生课程设计、刚入门想快速掌握运动规划实操的同学。前置要求很简单：会基本的终端操作，能看懂简单的Python代码就行，不需要ROS2基础。

## 1.3 技术选型：为什么用Dora-MoveIt2

先说说为什么不用ROS2 + MoveIt2：

1. **安装复杂**：ROS2要求特定Ubuntu版本，MoveIt2依赖众多C++库，编译动辄几十分钟
2. **概念繁多**：节点、话题、服务、动作、TF树、URDF/SRDF、Setup Assistant……初学者容易迷失
3. **调试困难**：多进程通信、DDS中间件、colcon构建系统，出问题定位成本高

Dora-MoveIt2的优势：

1. **纯Python**：`pip install` 即装即用，无需编译
2. **数据流YAML**：一个YAML文件定义所有节点连接，一目了然
3. **MuJoCo仿真**：内置MuJoCo仿真，自带3D可视化
4. **API一致**：MoveGroup API与ROS MoveIt几乎完全一致，迁移零成本

## 1.4 所需环境与安装步骤

我们的环境非常简单，不限定操作系统版本，macOS、Linux都可以。

**第一步：安装Python环境**

确保Python 3.9+，打开终端：

```bash
python3 --version   # 确认 >= 3.9
pip install dora-rs  # 安装dora数据流框架
```

**第二步：克隆代码库并安装**

```bash
git clone https://github.com/dora-rs/dora-moveit2.git
cd dora-moveit2

# 安装核心库
pip install -e dora_moveit/

# 安装MuJoCo仿真节点（dora数据流用）
pip install -e dora-mujoco/

# 安装GEN72单臂示例
pip install -e examples/move_group_demo/

# 安装双臂示例（后续章节使用）
pip install -e examples/dual_gen72/
```

就这三行，全部依赖自动解决，没有colcon、没有CMake、没有rosdep。

**第三步：安装MuJoCo**

```bash
pip install mujoco
```

MuJoCo是物理仿真引擎，自带3D可视化窗口，不需要额外安装RViz。

## 1.5 环境测试

安装完成后，我们测试环境：

```bash
cd examples/move_group_demo
dora up
dora start dataflows/moveit_example_mujoco.yml
```

如果能看到MuJoCo窗口中出现一个GEN72机械臂，并且手臂开始自动运动（先回home位，再做各种运动演示），说明环境搭建成功。

测试完成后停止：

```bash
dora stop
```

如果出错，大概率是两个问题：一是Python版本太低；二是pip包没装全，重新执行`pip install -e`命令即可。

## 1.6 本章小结

本章完成了课程介绍和环境搭建。核心记住：我们用Python + dora-rs + MuJoCo，三条pip install搞定一切。下一章我们学习dora数据流的核心概念。

---

# 第2章：Dora Dataflow Basics（dora数据流基础）

## 2.1 本章前言：为什么需要理解数据流

上一章我们搭好了环境，这一章快速了解dora-rs的核心概念。dora-rs是我们整个系统的"骨架"，所有模块（仿真、规划、IK求解、轨迹执行）都通过数据流连接，理解数据流才能看懂系统架构。

## 2.2 核心概念对比

如果你学过ROS2，下面的对比帮你快速迁移；没学过也没关系，直接看dora这一列：

| ROS2概念 | Dora-MoveIt2 等价物 | 说明 |
|---------|-------------------|------|
| Node（节点） | Operator（操作器） | 每个Python脚本就是一个操作器 |
| Topic（话题） | 数据流连接 | 在YAML里声明输入/输出，自动连接 |
| Service（服务） | 输出→输入配对 | 没有专门的服务机制，用配对的输入输出模拟 |
| Parameter（参数） | 环境变量 | 每个操作器通过`env`设置配置 |
| Launch file | Dataflow YAML | 一个YAML定义所有节点和连接 |
| TF2 坐标树 | 配置文件中的FK链 | 静态配置，不需要动态TF广播 |
| URDF/SRDF | Python Config类 | 关节限制、碰撞几何等全在Python类里 |

## 2.3 Dora 操作器的基本结构

每个操作器就是一个标准的Python脚本，核心模式：

```python
from dora import Node
import pyarrow as pa

def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "my_input":
                # 接收数据
                data = event["value"].to_numpy()
                # 处理数据
                result = process(data)
                # 发送输出
                node.send_output("my_output", pa.array(result))
        elif event["type"] == "STOP":
            break
```

就这么简单——一个事件循环，收到输入就处理，处理完就发送输出。

## 2.4 数据流YAML文件

我们打开`examples/move_group_demo/dataflows/moveit_example_mujoco.yml`看看，这就是整个系统的"接线图"：

```yaml
nodes:
  # MuJoCo仿真
  - id: mujoco_sim
    path: ../../../dora-mujoco/dora_mujoco/main.py
    inputs:
      tick: dora/timer/millis/10          # 每10ms触发一次
      control_input: trajectory_executor/joint_commands  # 接收关节指令
    outputs:
      - joint_positions    # 输出当前关节角度
      - joint_velocities   # 输出关节速度

  # 运动规划器
  - id: planner
    path: ../move_group_demo/nodes/planner.py
    inputs:
      plan_request: user_node/plan_request   # 接收规划请求
      scene_update: planning_scene/scene_update  # 接收场景更新
    outputs:
      - trajectory       # 输出规划轨迹
      - plan_status      # 输出规划状态
    env:
      ROBOT_CONFIG_MODULE: "move_group_demo.config.gen72"  # 机器人配置

  # ... 其他节点类似
```

核心看三点：
1. **id**：节点名称
2. **inputs/outputs**：定义数据连接（格式：`节点名/输出名`）
3. **env**：环境变量，最重要的是`ROBOT_CONFIG_MODULE`指定机器人配置

## 2.5 机器人配置注入机制

这是Dora-MoveIt2最巧妙的设计：**库操作器不绑定任何特定机器人**。机器人特定的参数（关节限制、FK链、碰撞几何）通过环境变量注入：

```python
from dora_moveit.config import load_config
config = load_config()  # 读取 ROBOT_CONFIG_MODULE 环境变量
print(config.NUM_JOINTS)       # 7（GEN72）
print(config.HOME_CONFIG)      # [0, -0.5, 0, 0, 0, 0.5, 0]
```

换机器人只需要写一个新的Config类，改YAML里的环境变量，所有库操作器自动适配。

## 2.6 实操：查看系统数据流

启动demo后，打开另一个终端：

```bash
dora list    # 查看运行中的节点
```

你能看到5个节点在运行：mujoco_sim, planning_scene, planner, ik_solver, trajectory_executor, user_node。它们之间的数据就是通过YAML里定义的连接自动流转的。

## 2.7 本章小结

本章掌握了dora数据流的核心：操作器（Python脚本）+ 数据流YAML（接线图）+ 环境变量（机器人配置注入）。这比ROS2的节点/话题/服务/TF2体系简单得多，下一章我们学习GEN72机器人的模型结构。

---

# 第3章：GEN72 Robot Model & Configuration（GEN72机器人模型与配置）

## 3.1 本章前言：为什么要了解机器人配置

这一章我们来了解GEN72 7自由度机械臂的结构。不用建模（太复杂），重点是看懂配置文件，知道关节限制、碰撞几何、命名姿态这些参数的含义——因为后续我们控制机器人的每一步都依赖这些参数。

## 3.2 GEN72机器人基本结构

GEN72是Realman（睿尔曼）的7自由度机械臂，结构如下：

- **7个旋转关节**：joint1到joint7，从底座到末端
- **8个连杆**：base_link + Link1到Link7
- **末端执行器**：Link7是末端法兰，可安装夹爪
- **关节限制**：每个关节有最大/最小角度限制，防止机械损坏

```
base_link → joint1 → Link1 → joint2 → Link2 → ... → joint7 → Link7（末端）
```

## 3.3 Python Config 类（代替URDF/SRDF）

在ROS2中，机器人模型用URDF和SRDF描述。Dora-MoveIt2用一个Python类代替，打开`examples/move_group_demo/move_group_demo/config/gen72.py`：

```python
class GEN72Config:
    # 关节数量
    NUM_JOINTS = 7

    # 关节限制（弧度）
    JOINT_LOWER_LIMITS = np.array([-3.0014, -1.8323, -3.0014, -2.8792, -3.0014, -1.707, -3.0014])
    JOINT_UPPER_LIMITS = np.array([ 3.0014,  1.8323,  3.0014,  0.9597,  3.0014,  1.783,  3.0014])

    # FK链变换（每个关节相对于上一个连杆的位移和旋转）
    LINK_TRANSFORMS = [
        {"xyz": [0, 0, 0.218], "rpy": [0, 0, 0]},        # joint1
        {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]},       # joint2
        {"xyz": [0, -0.28, 0], "rpy": [1.5708, 0, 0]},    # joint3
        # ... 共7个
    ]

    # 碰撞几何（每个连杆的球形近似，用于碰撞检测）
    COLLISION_GEOMETRY = [
        ("sphere", [0.055]),  # base_link
        ("sphere", [0.045]),  # Link1
        # ... 共8个
    ]

    # 默认姿态
    HOME_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])
    SAFE_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])

    # 命名姿态
    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(7),
    }
```

和URDF对比的优势：
- **可读性强**：Python代码，一目了然
- **可计算**：直接用numpy，不用解析XML
- **可继承**：双臂配置可以直接复用单臂数据

## 3.4 MuJoCo XML模型

仿真用的3D模型在`examples/move_group_demo/models/GEN72_base.xml`，这是MuJoCo的XML格式，定义了：
- 机械臂的3D外观（STL网格文件）
- 关节的物理属性（惯性、阻尼）
- 执行器（每个关节一个位置控制器）

大家启动demo后，MuJoCo窗口里看到的就是这个模型。可以用鼠标旋转、缩放查看机器人结构。

## 3.5 用MuJoCo查看机器人

实操一下，启动demo：

```bash
cd examples/move_group_demo
dora up && dora start dataflows/moveit_example_mujoco.yml
```

在MuJoCo窗口中：
- **鼠标左键拖动**：旋转视角
- **鼠标右键拖动**：平移视角
- **滚轮**：缩放

观察机器人的7个关节位置，从底座到末端，和Config类里的描述一一对应。

## 3.6 关节限制与安全

提醒大家重点：每个关节都有角度限制，比如joint4的范围是[-2.8792, 0.9597]弧度（约[-165°, 55°]），不对称。MoveGroup API在设置目标时会自动检查限制：

```python
# 如果设置的角度超出限制，会抛出 ValueError
group.set_joint_value_target([0, 0, 0, 2.0, 0, 0, 0])  # joint4=2.0 超出上限0.9597
# ValueError: Joint 3 value 2.000 outside limits [-2.879, 0.960]
```

## 3.7 本章小结

本章了解了GEN72的结构：7个关节、8个连杆，以及Python Config类如何代替URDF/SRDF描述机器人。重点记住Config类的几个关键属性：`JOINT_LOWER_LIMITS`/`JOINT_UPPER_LIMITS`（关节限制）、`LINK_TRANSFORMS`（FK链）、`NAMED_POSES`（命名姿态）。下一章我们学习MoveGroup API。

---

# 第4章：Dora-MoveIt2 Configuration（Dora-MoveIt2配置）

## 4.1 本章前言：配置是如何工作的

在ROS2中，MoveIt2需要一个复杂的Setup Assistant GUI来配置规划组、碰撞矩阵、末端执行器。Dora-MoveIt2把这一切简化了：**配置就是一个Python类**，不需要任何GUI工具。

## 4.2 配置的三个层级

Dora-MoveIt2的配置分三层：

**第一层：RobotConfig协议**（库定义的接口）

```python
# dora_moveit/config.py
class RobotConfig(Protocol):
    NUM_JOINTS: int
    JOINT_LOWER_LIMITS: np.ndarray
    JOINT_UPPER_LIMITS: np.ndarray
    LINK_TRANSFORMS: List[Dict]
    COLLISION_GEOMETRY: List[tuple]
    HOME_CONFIG: np.ndarray
    NAMED_POSES: Dict[str, np.ndarray]
    # ...
```

只要你的Config类包含这些属性，库的所有操作器就能自动使用。

**第二层：具体机器人Config类**（用户编写）

```python
# examples/move_group_demo/move_group_demo/config/gen72.py
class GEN72Config:
    NUM_JOINTS = 7
    JOINT_LOWER_LIMITS = np.array([...])
    # 实现所有Protocol要求的属性
```

**第三层：数据流YAML注入**（运行时绑定）

```yaml
env:
  ROBOT_CONFIG_MODULE: "move_group_demo.config.gen72"
```

每个操作器启动时通过`load_config()`读取这个环境变量，动态加载Config类。

## 4.3 创建新机器人配置（对比Setup Assistant）

ROS2的Setup Assistant流程：打开GUI → 加载URDF → 创建规划组 → 设置碰撞矩阵 → 生成配置包 → colcon编译。

Dora-MoveIt2的流程：写一个Python类 → pip install → 改YAML环境变量 → 完成。

实操：假设我们要给一个新的6自由度机器人写配置，只需要：

```python
class MyRobotConfig:
    NUM_JOINTS = 6
    JOINT_LOWER_LIMITS = np.array([-3.14, -1.57, -3.14, -3.14, -1.57, -3.14])
    JOINT_UPPER_LIMITS = np.array([3.14, 1.57, 3.14, 3.14, 1.57, 3.14])
    LINK_TRANSFORMS = [...]  # 从URDF提取
    COLLISION_GEOMETRY = [...]  # 每个连杆的碰撞近似
    HOME_CONFIG = np.zeros(6)
    NAMED_POSES = {"home": np.zeros(6)}
    # ... 其他必要属性
```

## 4.4 碰撞检测配置

ROS2用Setup Assistant自动生成碰撞矩阵（ACM），Dora-MoveIt2的碰撞检测更直接：

- **自碰撞**：自动跳过相邻2个连杆的检测（运动学链上相邻的连杆不可能碰撞）
- **环境碰撞**：通过PlanningScene动态添加/移除障碍物
- **碰撞几何**：Config类里的`COLLISION_GEOMETRY`定义每个连杆的碰撞形状

```python
COLLISION_GEOMETRY = [
    ("sphere", [0.055]),  # base_link，球形，半径5.5cm
    ("sphere", [0.045]),  # Link1
    # ...
]
```

## 4.5 双臂配置扩展（DualArmConfig）

对于双臂机器人，Config类额外提供：

```python
class DualGEN72Config:
    ARM_CHAINS = ["left_arm", "right_arm"]
    NUM_JOINTS = 7  # 每个臂

    # 每个臂的基座位置（相对于世界坐标系）
    ARM_BASE_TRANSFORMS = {
        "left_arm": {"xyz": [-0.3, 0, 0.8], "rpy": [0, 0, 0]},
        "right_arm": {"xyz": [0.3, 0, 0.8], "rpy": [0, 0, 3.14159]},
    }

    # 每个臂复用相同的关节限制和FK链
    LINK_TRANSFORMS_PER_CHAIN = {
        "left_arm": _GEN72_LINK_TRANSFORMS,
        "right_arm": _GEN72_LINK_TRANSFORMS,
    }
    HOME_CONFIG_PER_CHAIN = {
        "left_arm": _GEN72_HOME,
        "right_arm": _GEN72_HOME,
    }
```

库通过`is_dual_arm(config)`检测是否为双臂配置，自动切换到14D规划模式。

## 4.6 本章小结

本章学习了Dora-MoveIt2的配置体系：RobotConfig协议 → 具体Config类 → YAML环境变量注入。比ROS2的Setup Assistant简单很多——没有GUI、没有SRDF、没有colcon编译，就是一个Python类。下一章我们进入实操核心——MoveGroup Python API。

---

# 第5章：MoveGroup Python API（MoveGroup Python接口）

## 5.1 本章前言：为什么用MoveGroup API

各位同学，这一章我们进入实操核心——用Python代码控制GEN72机械臂。Dora-MoveIt2提供的MoveGroup API和ROS2 MoveIt的`moveit_commander`接口几乎一致，如果你之前用过ROS2 MoveIt，迁移零成本。

## 5.2 核心代码框架

我们先写一个最基础的框架，连接MoveGroup：

```python
from dora_moveit.workflow.move_group import MoveGroup

def main():
    # 创建MoveGroup实例（自动连接到dora数据流）
    group = MoveGroup("gen72")
    # 获取规划场景接口
    scene = group.get_planning_scene_interface()

    print("MoveGroup连接成功！")
    print(f"当前关节角度: {group.get_current_joint_values()}")
    print(f"可用命名姿态: {group.get_named_targets()}")

    group.shutdown()

if __name__ == "__main__":
    main()
```

对比ROS2的代码：
```python
# ROS2 MoveIt
import rclpy
from moveitpy import MoveItPy
rclpy.init()
moveit = MoveItPy(node_name="...", config_package_name="...")
left_arm = moveit.get_planning_component("left_arm")
```

Dora-MoveIt2更简洁——不用初始化ROS2节点，不用指定配置包（YAML里已经配了），一行`MoveGroup("gen72")`搞定。

## 5.3 设置目标：关节角度

最直接的方式，设置每个关节的角度：

```python
# 方式1：传入7个关节角度的列表
group.go([1.57, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0], wait=True)

# 方式2：先设置再执行
group.set_joint_value_target([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])
group.go(wait=True)
```

`wait=True`表示阻塞等待执行完成，返回True/False表示是否成功。

## 5.4 设置目标：命名姿态

Config类里预定义了命名姿态，用名称直接设置：

```python
# 移动到home姿态
group.set_named_target("home")
group.go(wait=True)

# 移动到zero姿态（所有关节归零）
group.set_named_target("zero")
group.go(wait=True)
```

也可以自定义命名姿态：

```python
# 记住当前位置为"my_pose"
group.remember_joint_values("my_pose")
# 之后随时回到这个位置
group.set_named_target("my_pose")
group.go(wait=True)
```

## 5.5 设置目标：末端位姿（Cartesian Pose）

设置末端执行器（Link7）的空间位置和姿态，内部自动求解IK：

```python
# [x, y, z, roll, pitch, yaw]
group.set_pose_target([0.15, 0.1, 0.6, 0, 0, 0])
success = group.go(wait=True)
if success:
    pos, rot = group.get_current_pose()
    print(f"到达位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
else:
    print("运动失败（IK求解失败或规划失败）")
```

## 5.6 规划与执行分离

有时候我们想先看看规划结果，再决定是否执行：

```python
# 只规划，不执行
group.set_joint_value_target([1.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0])
success, trajectory = group.plan()
print(f"规划{'成功' if success else '失败'}，轨迹包含{len(trajectory)}个路点")

# 决定执行
if success:
    group.execute(trajectory, wait=True)
```

## 5.7 完整API对照表

| 功能 | Dora-MoveIt2 | ROS2 MoveIt |
|------|-------------|-------------|
| 连接规划组 | `MoveGroup("gen72")` | `MoveGroupCommander("manipulator")` |
| 命名目标 | `set_named_target("home")` | `set_named_target("home")` |
| 关节目标 | `go([1.57, ...])` | `go([1.57, ...])` |
| 位姿目标 | `set_pose_target([x,y,z,r,p,y])` | `set_pose_target(pose)` |
| 规划 | `plan()` | `plan()` |
| 执行 | `execute(traj)` | `execute(traj)` |
| 停止 | `stop()` | `stop()` |
| 当前关节 | `get_current_joint_values()` | `get_current_joint_values()` |
| 正运动学 | `compute_fk()` | 需要单独调用服务 |
| 逆运动学 | `compute_ik(pose)` | 需要单独调用服务 |
| 添加障碍物 | `scene.add_box(...)` | `scene.add_box(...)` |

## 5.8 本章小结

本章掌握了MoveGroup API的核心用法：关节目标、命名姿态、Cartesian位姿目标、规划/执行分离。API和ROS2 MoveIt几乎一致，迁移成本为零。下一章我们深入练习单臂运动规划。

---

# 第6章：Single Arm Motion Planning（单臂运动规划）

## 6.1 本章前言：先练熟单臂，再学双臂

各位同学，这一章我们重点练熟GEN72单臂的运动规划——双臂协调的基础是单臂控制。本章围绕四个核心操作：关节运动、笛卡尔直线运动、简单抓取、避障。

## 6.2 单臂关节空间规划

关节空间规划就是直接控制每个关节到达指定角度，最简单直接。

```python
from dora_moveit.workflow.move_group import MoveGroup
import time

def main():
    group = MoveGroup("gen72")

    # 1. 先回到home姿态
    print("回到home姿态...")
    group.set_named_target("home")
    group.go(wait=True)
    time.sleep(1)

    # 2. 关节空间目标：底座旋转90°，肩关节倾斜
    print("关节空间规划...")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.57     # joint1: 底座旋转90°
    joint_goal[1] = -0.785   # joint2: 肩关节倾斜
    joint_goal[3] = -1.57    # joint4: 肘关节弯曲
    joint_goal[5] = 0.785    # joint6: 腕关节倾斜
    group.go(joint_goal, wait=True)
    print(f"到达关节目标: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # 3. 回到home
    group.set_named_target("home")
    group.go(wait=True)

    group.shutdown()
```

大家可以修改关节角度值，运行代码观察机械臂运动变化。注意不要超出关节限制。

## 6.3 单臂笛卡尔路径规划（直线运动）

笛卡尔路径规划控制末端执行器沿**直线**运动，适合需要精准轨迹的场景（比如靠近物体时走直线）。

```python
    # 笛卡尔路径：先移到一个位置，然后沿直线运动
    print("\n笛卡尔路径规划...")
    group.set_pose_target([0.15, 0.1, 0.6, 0, 0, 0])
    group.go(wait=True)
    time.sleep(1)

    # 获取当前末端位置
    current_pos, _ = group.get_current_pose()

    # 定义直线路径：先下降10cm，再侧移10cm
    waypoints = [
        [current_pos[0], current_pos[1], current_pos[2] - 0.1, 0, 0, 0],      # 下降
        [current_pos[0], current_pos[1] + 0.1, current_pos[2] - 0.1, 0, 0, 0], # 侧移
    ]

    trajectory, fraction = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,   # 每1cm一个插值点
    )
    print(f"笛卡尔路径: {len(trajectory)}个路点, 完成{fraction * 100:.0f}%")

    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print("直线运动完成！")
    else:
        print("路径规划失败（IK求解失败的点太多）")
```

`compute_cartesian_path`的工作原理：在起点和每个路点之间，按`eef_step`间距做插值，每个插值点都求解IK，确保末端沿直线运动。`fraction`返回成功比例，1.0表示100%成功。

## 6.4 单臂简单抓取

抓取的核心流程：**接近 → 下降 → 抓紧 → 提升 → 移动 → 放下 → 松开**。在MuJoCo仿真中我们用关节目标模拟：

```python
    # 简单抓取流程（用关节目标模拟）
    print("\n简单抓取流程...")

    # 1. 移动到物体上方
    above_object = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(above_object, wait=True)
    print("  到达物体上方")
    time.sleep(0.5)

    # 2. 下降接近物体
    near_object = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go(near_object, wait=True)
    print("  下降到物体位置")
    time.sleep(0.5)

    # 3. 抓取（实际机器人这里控制夹爪关闭）
    print("  抓取物体...")
    time.sleep(0.5)

    # 4. 提升
    group.go(above_object, wait=True)
    print("  提升物体")
    time.sleep(0.5)

    # 5. 移动到目标位置上方
    above_target = [1.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(above_target, wait=True)
    print("  到达目标位置上方")
    time.sleep(0.5)

    # 6. 下降放置
    near_target = [1.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go(near_target, wait=True)
    print("  放下物体")
    time.sleep(0.5)

    # 7. 松开（实际机器人这里控制夹爪打开）
    print("  松开夹爪")
    time.sleep(0.5)

    # 8. 抬起回home
    group.set_named_target("home")
    group.go(wait=True)
    print("抓取流程完成！")
```

## 6.5 避障

Dora-MoveIt2内置碰撞检测，通过PlanningScene添加障碍物，规划器自动避障：

```python
    # 添加一个方块障碍物
    scene = group.get_planning_scene_interface()
    scene.add_box("obstacle_box", [0.0, 0.0, 0.5], [0.1, 0.1, 0.5])
    print("添加障碍物: obstacle_box 在 [0, 0, 0.5]")
    time.sleep(1)

    # 规划到home——规划器会自动避开障碍物
    group.set_named_target("home")
    success = group.go(wait=True)
    if success:
        print("到达home（规划器避开了障碍物）")
    else:
        print("规划失败——障碍物可能挡住了所有路径")

    # 移除障碍物
    scene.remove_world_object("obstacle_box")
    print("移除障碍物")
```

规划器（RRT-Connect算法）在搜索路径时，每个候选配置都会检查碰撞。如果存在碰撞，该配置被丢弃，直到找到一条无碰撞路径。

## 6.6 本章小结

本章练熟了GEN72单臂的四个核心操作：关节空间规划、笛卡尔直线运动、简单抓取流程、避障。大家一定要多实操，能独立控制单臂运动后，下一章我们进入双臂协调控制。

---

# 第7章：Dual-Arm Coordination（双臂协调控制）

## 7.1 本章前言：双臂协调的核心

各位同学，这一章我们进入课程核心——双臂协调控制。使用两个GEN72机械臂，通过Dora-MoveIt2的`DualMoveGroup` API实现协调运动。核心是：**14D统一规划**——把两个7D手臂的关节拼接成14D空间，用一个规划器同时规划，天然保证同步和避碰。

## 7.2 双臂系统架构

双臂系统和单臂只有两个区别：

1. **Config类**：`DualGEN72Config`代替`GEN72Config`，包含`ARM_CHAINS`等双臂字段
2. **MuJoCo模型**：`dual_gen72.xml`，两个GEN72臂安装在同一张桌子上

数据流架构完全相同——还是5个库操作器（仿真、场景、规划器、IK、执行器），它们检测到双臂配置后自动切换到14D模式。

```
用户 (DualMoveGroup API)
  → plan_request (14D) → 规划器 (14D RRT-Connect)
  → trajectory (14D) → 执行器 (拆分为左7D + 右7D)
  → joint_commands (14D) → MuJoCo (14个执行器)
```

## 7.3 启动双臂环境

```bash
cd examples/dual_gen72
dora up
dora start dataflows/dual_gen72_mujoco.yml
```

MuJoCo窗口中，你能看到一张桌子上有两个GEN72臂：
- **左臂**（蓝色调）在x=-0.3的位置
- **右臂**（红色调）在x=0.3的位置
- 中间有一个红色小球和一个绿色盘子

## 7.4 DualMoveGroup API

`DualMoveGroup`是双臂版的MoveGroup，用法非常直观：

```python
from dora_moveit.workflow.dual_move_group import DualMoveGroup

group = DualMoveGroup(left_name="left_arm", right_name="right_arm")
```

三种运动模式：

**1. 同步运动（双臂同时运动）**

```python
# 双臂同时回到home
group.set_named_target(left_name="home", right_name="home")
group.go(wait=True)

# 双臂同时到达指定关节角度
left_target = [0.5, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
right_target = [-0.5, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
group.go(left_joints=left_target, right_joints=right_target, wait=True)
```

同步运动使用14D规划——14个关节在同一个RRT-Connect树中搜索，天然保证双臂同步到达目标、且不会互相碰撞。

**2. 独立运动（只动一个臂）**

```python
# 只动左臂，右臂保持不动
group.go_left([0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0], wait=True)

# 只动右臂，左臂保持不动
group.go_right([0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0], wait=True)
```

内部实现：另一个臂的目标设为当前角度，所以14D规划中另一个臂不动。

**3. 命名姿态**

```python
# 单独设置
group.set_named_target(left_name="home")          # 只设左臂
group.set_named_target(right_name="zero")          # 只设右臂
group.go(wait=True)                                  # 同时执行

# 同时设置
group.set_named_target(left_name="home", right_name="home")
group.go(wait=True)
```

## 7.5 双臂独立控制（分时运动）

分时运动就是先控制一个臂完成动作，再控制另一个臂，适合不需要同步的场景：

```python
    # 1. 左臂抓取物体
    print("左臂移动到物体位置...")
    left_above_ball = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_above_ball, wait=True)
    time.sleep(0.5)

    # 2. 左臂下降抓取
    left_grasp = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_left(left_grasp, wait=True)
    print("左臂抓取完成")
    time.sleep(0.5)

    # 3. 左臂提升
    group.go_left(left_above_ball, wait=True)
    time.sleep(0.5)

    # 4. 右臂移动到接收位置
    print("右臂移动到接收位置...")
    right_receive = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_receive, wait=True)
    print("右臂就位")
```

## 7.6 双臂同步运动（同时运动）

同步运动让双臂同时开始、同时结束，适合需要保持相对姿态的场景：

```python
    # 双臂同步移动到交接位置
    print("双臂同步移动到交接位置...")
    left_handoff = [0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    right_handoff = [-0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(left_joints=left_handoff, right_joints=right_handoff, wait=True)
    print("双臂到达交接位置！")
    time.sleep(1)

    # 双臂同步上升0.1弧度（演示同步性）
    print("双臂同步上升...")
    left_up = left_handoff.copy()
    right_up = right_handoff.copy()
    left_up[1] -= 0.2   # 肩关节上抬
    right_up[1] -= 0.2
    group.go(left_joints=left_up, right_joints=right_up, wait=True)
    print("双臂同步上升完成！")
```

## 7.7 双臂间避障

14D统一规划的最大优势：**自动避免双臂碰撞**。规划器在搜索14D路径时，每个候选配置都会检查两个臂之间的碰撞（通过`check_inter_arm_collision()`）。

```python
    # 设置一个双臂可能交叉的目标
    # 规划器会自动规划避碰轨迹
    left_cross = [0.8, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    right_cross = [-0.8, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    success = group.go(left_joints=left_cross, right_joints=right_cross, wait=True)
    if success:
        print("双臂运动完成（自动避碰）")
    else:
        print("规划失败——目标姿态可能导致不可避免的碰撞")
```

## 7.8 本章小结

本章掌握了双臂协调控制的三种模式：同步运动（`go(left_joints, right_joints)`）、独立运动（`go_left()`/`go_right()`）、分时运动。核心理解：14D统一规划天然保证同步和避碰。下一章我们做综合实验。

---

# 第8章：Final Demo & Debug（综合实验与调试）

## 8.1 本章前言

各位同学，最后一章，我们跑通一个完整的双臂抓取-传递-放置Demo，同时学习调试方法。

## 8.2 综合实验：双臂抓取-传递-放置

完整代码（保存为`dual_arm_final_demo.py`）：

```python
#!/usr/bin/env python3
"""
双臂GEN72综合Demo：抓取-传递-放置
"""
import time
import numpy as np
from dora_moveit.workflow.dual_move_group import DualMoveGroup


def main():
    print("=" * 60)
    print("  双臂GEN72综合Demo：抓取-传递-放置")
    print("=" * 60)

    group = DualMoveGroup(left_name="left_arm", right_name="right_arm")

    # ===== Step 1: 双臂回到home =====
    print("\n[Step 1] 双臂回到home姿态")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    time.sleep(1.0)

    # ===== Step 2: 左臂移动到物体上方 =====
    print("\n[Step 2] 左臂→物体上方")
    left_above = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_above, wait=True)
    time.sleep(0.5)

    # ===== Step 3: 左臂下降抓取 =====
    print("\n[Step 3] 左臂下降抓取")
    left_grasp = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_left(left_grasp, wait=True)
    print("  夹爪关闭，抓取物体...")
    time.sleep(0.5)

    # ===== Step 4: 左臂提升 =====
    print("\n[Step 4] 左臂提升")
    group.go_left(left_above, wait=True)
    time.sleep(0.5)

    # ===== Step 5: 右臂移动到接收位置 =====
    print("\n[Step 5] 右臂→接收位置")
    right_receive = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_receive, wait=True)
    time.sleep(0.5)

    # ===== Step 6: 双臂同步到交接位置 =====
    print("\n[Step 6] 双臂同步移动到交接位置")
    left_handoff = [0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    right_handoff = [-0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(left_joints=left_handoff, right_joints=right_handoff, wait=True)
    print("  物体传递中...")
    time.sleep(1.0)

    # ===== Step 7: 右臂移动到放置位置 =====
    print("\n[Step 7] 右臂→放置位置上方")
    right_above_plate = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_above_plate, wait=True)
    time.sleep(0.5)

    # ===== Step 8: 右臂下降放置 =====
    print("\n[Step 8] 右臂下降放置")
    right_place = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_right(right_place, wait=True)
    print("  夹爪打开，放置物体...")
    time.sleep(0.5)

    # ===== Step 9: 右臂撤回 =====
    print("\n[Step 9] 右臂撤回")
    group.go_right(right_above_plate, wait=True)
    time.sleep(0.5)

    # ===== Step 10: 双臂回到home =====
    print("\n[Step 10] 双臂回到home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)

    print("\n" + "=" * 60)
    print("  综合Demo执行完成！")
    print("=" * 60)

    # 保持MuJoCo窗口不关闭
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
```

## 8.3 运行Demo

```bash
# 终端1：启动双臂仿真环境
cd examples/dual_gen72
dora up
dora start dataflows/dual_gen72_mujoco.yml
```

观察MuJoCo窗口中两个机械臂依次完成：home → 左臂抓取 → 提升 → 右臂就位 → 双臂交接 → 右臂放置 → 双臂home。

运行成功说明你已经掌握了课程核心内容！

## 8.4 常见错误与调试方法

实操中最常见的3个错误：

**1. 规划失败（go()返回False）**

原因：目标关节角度超出限制、碰撞、规划超时。

调试方法：
```python
# 检查关节限制
from dora_moveit.config import load_config
config = load_config()
print("下限:", config.JOINT_LOWER_LIMITS)
print("上限:", config.JOINT_UPPER_LIMITS)

# 确认目标在限制内
target = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
for i, (v, lo, hi) in enumerate(zip(target, config.JOINT_LOWER_LIMITS, config.JOINT_UPPER_LIMITS)):
    if v < lo or v > hi:
        print(f"关节{i}超出限制: {v} 不在 [{lo}, {hi}] 范围内")
```

**2. IK求解失败（位姿目标不可达）**

原因：目标位置超出机械臂工作空间。

调试方法：
```python
# 先用FK确认工作空间范围
pos, _ = group.compute_fk([0, 0, 0, 0, 0, 0, 0])
print(f"zero姿态末端位置: {pos}")  # 了解臂的到达范围

# 尝试不同的seed
solution = group.compute_ik([0.15, 0.1, 0.6, 0, 0, 0])
if solution is None:
    print("IK失败——目标不可达")
```

**3. 执行超时**

原因：轨迹太长、仿真速度慢、数据流连接断开。

调试方法：
```bash
# 检查数据流状态
dora list

# 如果节点不在运行，重新启动
dora stop
dora up
dora start dataflows/dual_gen72_mujoco.yml
```

## 8.5 系统架构回顾

```
┌──────────────────────────────────────────────────┐
│  用户代码 (MoveGroup / DualMoveGroup API)        │
│  - set_named_target / go / compute_cartesian_path │
└──────────────┬──────────────┬────────────────────┘
               │              │
     plan_request       ik_request
               │              │
     ┌─────────▼─────┐  ┌────▼──────┐
     │   Planner      │  │ IK Solver │
     │ (RRT-Connect)  │  │ (TracIK)  │
     └───────┬────────┘  └────┬──────┘
          trajectory       ik_solution
             │                │
     ┌───────▼────────────────┘
     │  Trajectory Executor
     │  (插值、平滑、输出关节指令)
     └───────┬───────┐
        joint_commands │
             │         │
     ┌───────▼───┐  ┌──▼──────────────┐
     │  MuJoCo   │  │ Planning Scene  │
     │  仿真+可视 │  │ (场景管理/碰撞) │
     └───────────┘  └─────────────────┘
```

所有模块通过YAML数据流连接，模块间零耦合，换机器人只需要换Config类和MuJoCo模型。

## 8.6 硬件对接思路

现在我们用的是MuJoCo仿真，对接真实GEN72只需要替换仿真节点：

1. **MuJoCo仿真节点** → **Realman SDK驱动节点**（发送关节角度到真实电机）
2. **关节指令格式不变**：都是7个浮点数的关节角度
3. **Config类不变**：关节限制、FK链完全一样
4. **YAML改一行**：把`mujoco_sim`的`path`改成真实驱动脚本

```yaml
# 仿真版
- id: mujoco_sim
  path: ../../../dora-mujoco/dora_mujoco/main.py

# 真实硬件版
- id: real_robot
  path: ../dual_gen72_demo/nodes/realman_driver.py
```

## 8.7 课程总结

各位同学，我们的Dora-MoveIt2 + GEN72双臂机器人精简课程就结束了。回顾一下：

| 章节 | 内容 | 关键代码/命令 |
|------|------|--------------|
| 第1章 | 环境搭建 | `pip install -e dora_moveit/` |
| 第2章 | 数据流基础 | YAML数据流、操作器模式、Config注入 |
| 第3章 | GEN72模型 | `GEN72Config`类、关节限制、FK链 |
| 第4章 | 配置体系 | RobotConfig协议、DualArmConfig扩展 |
| 第5章 | MoveGroup API | `go()`, `set_named_target()`, `compute_cartesian_path()` |
| 第6章 | 单臂规划 | 关节目标、笛卡尔路径、抓取、避障 |
| 第7章 | 双臂协调 | `DualMoveGroup`, `go_left()`, `go_right()`, 14D规划 |
| 第8章 | 综合实验 | 抓取-传递-放置完整Demo |

和ROS2 + MoveIt2的对比：

| 对比项 | ROS2 + MoveIt2 | Dora-MoveIt2 |
|--------|---------------|--------------|
| 安装时间 | 30分钟+ | 5分钟 |
| 依赖 | C++编译、colcon、rosdep | pip install |
| 配置方式 | Setup Assistant GUI | Python类 |
| API接口 | MoveItPy | MoveGroup（几乎一致） |
| 仿真 | Gazebo + RViz | MuJoCo（自带可视化） |
| 学习曲线 | 陡峭（概念多） | 平缓（纯Python） |

大家课后多练习，熟练掌握单臂和双臂的基本控制。有问题可以在GitHub上提issue。祝大家学习顺利！

# ROS2 + MoveIt2 双臂机器人精简课程（讲课原文）

# 第1章：Course Introduction & Environment Setup（课程介绍与环境搭建）

## 1.1 课程欢迎与目标

各位同学大家好，今天我们开始学习ROS2 + MoveIt2 双臂机器人控制的精简课程。咱们这门课不搞复杂理论，核心目标就一个——让大家能快速上手，用MoveIt2控制双臂机器人完成基本的运动和简单抓取，全程围绕实操，学完就能跑通demo，适合刚接触ROS2和双臂机器人的同学。

## 1.2 课程适用人群与前置要求

首先说下适用人群，不管你是做机器人研发、学生课程设计，还是刚入门想快速掌握实操，这门课都合适。前置要求很简单，不用你精通ROS2，只要大概知道Linux基本操作（比如打开终端、输命令），能看懂简单的代码逻辑就可以，剩下的我们一步步带着大家做。

## 1.3 所需环境与安装步骤

接下来我们搭建学习环境，大家注意，我们统一用Ubuntu系统，搭配ROS2 Humble版本——这是目前最稳定、最常用的版本，适合实操学习。第一步，先确认大家的Ubuntu系统是22.04版本（因为Humble只适配22.04），如果不是，先重装系统，或者升级到22.04。

然后我们安装ROS2 Humble，步骤我会一步步念，大家跟着输命令，不要着急。首先设置软件源，打开终端，输入sudo apt update && sudo apt upgrade，先更新系统软件。然后按照ROS2官方文档的步骤，添加ROS2的软件源、导入密钥，之后安装ROS2 Humble的桌面完整版，命令是sudo apt install ros-humble-desktop-full。

安装完成后，我们配置环境变量，输入echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc，然后source ~/.bashrc，这样每次打开终端，ROS2环境就会自动加载。

## 1.4 MoveIt2 与 demo 环境安装

环境变量配置好后，我们安装MoveIt2，输入sudo apt install ros-humble-moveit，等待安装完成即可。安装完MoveIt2，我们还要安装双臂机器人的demo环境，方便大家后续实操，命令是sudo apt install ros-humble-moveit-demo-moveit-config，这个demo里包含了现成的双臂机器人模型，我们不用自己建模，直接用就行。

## 1.5 环境测试与问题排查

安装完成后，我们测试一下环境是否正常。打开终端，输入ros2 launch moveit_demo_moveit_config demo.launch.py，这个命令会启动RViz2，并且加载双臂机器人模型。大家看自己的屏幕，如果能看到一个双臂机器人，RViz2没有报错，就说明环境搭建成功了。

如果出现报错，大概率是两个问题：一是ROS2环境变量没配置好，大家重新输入source /opt/ros/humble/setup.bash试试；二是demo包没安装成功，重新执行安装demo的命令即可。有问题的同学举手，我过来帮大家排查。

## 1.6 本章小结

好，本章我们就完成了课程介绍和环境搭建，核心就是记住：我们用Ubuntu 22.04 + ROS2 Humble + MoveIt2，环境测试成功后，后续的实操才能顺利进行。下一章我们快速回顾一下ROS2的核心知识点，为后续控制机器人做准备。

# 第2章：ROS2 Basics for Manipulation（机器人操作必备ROS2基础）

## 2.1 本章前言：为什么需要ROS2基础

各位同学，上一章我们搭好了环境，这一章我们快速回顾一下ROS2的核心知识点——不用深入，只要掌握和机器人操作相关的内容就可以，因为后续我们用MoveIt2控制双臂，本质上还是通过ROS2的节点、话题来实现的，所以这部分基础必须掌握。

## 2.2 ROS2 核心概念：Nodes、Topics、Services

首先我们讲三个核心概念：节点（Nodes）、话题（Topics）、服务（Services）。大家不用记复杂的定义，我用通俗的话讲：

节点：就是一个独立的程序，比如我们控制左臂运动的程序、控制夹爪开合的程序，每个程序都是一个节点，节点之间可以互相通信。

话题：就是节点之间传递数据的“通道”，比如机器人关节的角度、末端执行器的位姿，都是通过话题传递的。话题是“单向通信”，比如一个节点发布关节角度，另一个节点接收，接收方不能给发布方反馈。

服务：和话题不一样，是“双向通信”，比如我们给夹爪发一个“开合”的指令，夹爪执行完成后，会给我们反馈“执行成功”或“执行失败”，这就是服务。

## 2.3 机器人操作常用的话题与服务

结合我们的双臂机器人，大家重点记两个常用的话题和服务：

话题：/joint_states（关节状态话题），这个话题会发布机器人所有关节的角度、速度信息，我们可以通过这个话题，查看机器人当前的关节状态。

服务：/gripper/command（夹爪控制服务），我们通过这个服务，给夹爪发送“打开”或“关闭”的指令，并且能收到执行反馈。

## 2.4 TF2 坐标系统基础

接下来是TF2坐标系统，这个很重要，控制机器人运动，本质上就是控制机器人在不同坐标系下的位置和姿态。大家记住几个核心坐标系：

1. 基座坐标系（base_link）：机器人的“根”坐标系，固定在机器人的底座上，所有其他坐标系都相对于这个坐标系来定义。

2. 左臂坐标系（left_arm_link）：左臂的根坐标系，相对于base_link，控制左臂运动，就是在这个坐标系下规划轨迹。

3. 右臂坐标系（right_arm_link）：和左臂类似，是右臂的根坐标系。

4. 夹爪坐标系（left_gripper_link / right_gripper_link）：末端执行器的坐标系，我们控制抓取，就是控制这个坐标系到达目标位置。

简单说，TF2就是帮我们“定位”机器人各个部位的位置，后续我们用MoveIt2规划运动，都会用到这些坐标系。

## 2.5 命令行测试：查看话题与服务

我们来实操测试一下，打开终端，先启动上一章的demo，输入ros2 launch moveit_demo_moveit_config demo.launch.py。然后再打开一个新终端，输入ros2 topic list，就能看到所有正在发布的话题，找到/joint_states，输入ros2 topic echo /joint_states，就能看到机器人当前的关节状态数据。

再测试服务，输入ros2 service list，找到和夹爪相关的服务，输入ros2 service call /left_gripper/command std_srvs/srv/SetBool "{data: true}"，就能控制左夹爪打开；把true改成false，就是关闭夹爪，大家可以试试。

## 2.6 本章小结

好，本章我们快速回顾了ROS2的核心知识点，重点记住：节点、话题、服务的作用，以及TF2的几个核心坐标系，还有常用的命令行操作。这些内容足够我们后续用MoveIt2控制双臂机器人了，下一章我们学习机器人模型，看懂双臂机器人的结构。

# 第3章：Dual-Arm URDF & Robot Model（双臂机器人模型与URDF简介）

## 3.1 本章前言：为什么要了解机器人模型

各位同学，这一章我们来了解双臂机器人的模型，不用大家自己建模（太复杂），重点是看懂模型结构，知道哪些是左臂、右臂、夹爪，以及关节的作用——因为后续我们用MoveIt2配置规划组，需要选择对应的关节和连杆，看懂模型才能正确配置。

## 3.2 机器人模型的基本结构（双臂机器人）

首先，我们来看双臂机器人的基本结构，不管是自定义的双臂，还是工业上的Yumi、Sawyer，结构都大同小异，主要分为4个部分：

1. 底座（Base）：机器人的基础，固定在地面上，所有运动都围绕底座展开，对应坐标系base_link。

2. 左臂（Left Arm）：通常有6个关节（从底座到夹爪），每个关节可以旋转，控制左臂的姿态，对应left_arm_link系列坐标系。

3. 右臂（Right Arm）：和左臂结构对称，同样有6个关节，控制右臂的姿态，对应right_arm_link系列坐标系。

4. 夹爪（Gripper）：左右臂各一个，是末端执行器，用于抓取物体，对应left_gripper_link和right_gripper_link。

## 3.3 URDF 简介（不深入，只看懂）

机器人模型是用URDF文件描述的，URDF就是“机器人统一描述格式”，简单说，就是用代码把机器人的连杆、关节、尺寸、外观都描述出来，ROS2和MoveIt2就能识别这个模型。

大家不用去写URDF代码，我们重点看URDF里的两个核心部分：

1. 连杆（Link）：机器人的“骨骼”，比如底座连杆、左臂的各个连杆，每个连杆都有自己的尺寸、外观和惯性参数。

2. 关节（Joint）：连接两个连杆的“关节”，控制连杆的运动，比如旋转关节（可以绕一个轴旋转），每个关节都有运动范围限制（比如只能旋转0-180度）。

## 3.4 用 RViz2 查看机器人模型

我们实操一下，打开终端，启动demo：ros2 launch moveit_demo_moveit_config demo.launch.py，打开RViz2后，大家能看到一个双臂机器人。我们来操作一下，在RViz2的左侧“Displays”面板，找到“RobotModel”，勾选“Link Tree”，就能看到机器人所有的连杆和关节名称。

大家可以用鼠标拖动机器人的关节，看看机器人的运动——这就是RViz2的可视化功能，能让我们直观地看到机器人的结构和运动状态。我们重点区分：left_arm（左臂）、right_arm（右臂）、left_gripper（左夹爪）、right_gripper（右夹爪），记住这些名称，下一章配置MoveIt2会用到。

## 3.5 关节限制与运动范围

这里提醒大家一个重点：每个关节都有运动范围限制，比如左臂的肩关节，不能360度旋转，这在URDF里已经定义好了。后续我们用MoveIt2规划运动时，MoveIt2会自动遵守这个限制，不会让关节超出运动范围，避免损坏机器人。

## 3.6 本章小结

好，本章我们了解了双臂机器人的模型结构，重点记住：底座、左臂、右臂、夹爪的组成，能在RViz2中区分各个部分，知道URDF是描述机器人模型的文件，不用自己写，能看懂即可。下一章我们学习MoveIt2 Setup Assistant，快速配置机器人的规划组。

# 第4章：MoveIt2 Setup Assistant (Quick Config)（MoveIt2快速配置）

## 4.1 本章前言：MoveIt2 Setup Assistant 的作用

各位同学，这一章我们学习MoveIt2的核心工具——MoveIt2 Setup Assistant，它的作用很简单：帮我们快速配置机器人模型，生成MoveIt2的配置包，不用我们手动写复杂的配置文件，全程图形化操作，非常简单，大家跟着我做就行。

## 4.2 启动 MoveIt2 Setup Assistant

首先，我们启动MoveIt2 Setup Assistant，打开终端，输入命令：ros2 launch moveit_setup_assistant setup_assistant.launch.py，启动后会弹出一个图形化界面，这就是我们的配置工具。

界面上有两个选项：“Create New MoveIt Configuration Package”（创建新的配置包）和“Edit Existing MoveIt Configuration Package”（编辑已有的配置包），我们选择第一个，创建新的配置包。

## 4.3 加载双臂机器人URDF模型

下一步，我们需要加载双臂机器人的URDF模型。点击界面上的“Browse”，找到我们之前安装的demo包中的URDF文件，路径通常是：/opt/ros/humble/share/moveit_demo_moveit_config/urdf/robot.urdf。

加载完成后，点击“Load Files”，此时界面左侧会显示机器人的模型预览，和我们在RViz2中看到的一样，说明URDF加载成功。

## 4.4 创建规划组（核心步骤）

规划组（Planning Group）是MoveIt2的核心，简单说，就是把机器人的关节和连杆分组，比如把左臂的所有关节归为“left_arm”组，把右臂的归为“right_arm”组，这样我们后续就能单独控制左臂、右臂，或者同时控制双臂。

操作步骤：

1. 点击左侧菜单栏的“Planning Groups”，然后点击“Add Group”。

2. 第一个规划组：Name填“left_arm”，Group Type选择“MoveItSimpleControllerManager”，然后点击“Add Joints”，在弹出的窗口中，勾选所有属于左臂的关节（名称包含left_arm），点击“OK”。

3. 第二个规划组：点击“Add Group”，Name填“right_arm”，Group Type同样选择“MoveItSimpleControllerManager”，Add Joints时勾选所有属于右臂的关节（名称包含right_arm），点击“OK”。

4. 第三个规划组：点击“Add Group”，Name填“dual_arms”，Group Type选择“MoveItSimpleControllerManager”，Add Joints时勾选左臂和右臂的所有关节，点击“OK”——这个组用于控制双臂协同运动。

## 4.5 设置末端执行器（夹爪）

下一步，我们设置末端执行器（夹爪），让MoveIt2识别夹爪，后续才能控制夹爪抓取。操作步骤：

1. 点击左侧菜单栏的“End Effectors”，点击“Add End Effector”。

2. 第一个夹爪：Name填“left_gripper”，Type选择“Gripper”，Parent Group选择“left_arm”（左夹爪属于左臂），然后点击“Add Joints”，勾选左夹爪的关节，点击“OK”。

3. 第二个夹爪：点击“Add End Effector”，Name填“right_gripper”，Type选择“Gripper”，Parent Group选择“right_arm”，Add Joints勾选右夹爪的关节，点击“OK”。

## 4.6 生成配置包并测试

配置完成后，我们生成MoveIt2配置包。点击左侧菜单栏的“Configuration Files”，然后点击“Browse”，选择一个保存路径（比如我们的ROS2工作空间的src目录下），Name填“dual_arm_moveit_config”，点击“Generate Package”，等待生成完成。

生成完成后，我们测试一下配置包是否可用。打开终端，进入ROS2工作空间，输入colcon build，编译配置包，编译完成后，source install/setup.bash，然后输入ros2 launch dual_arm_moveit_config demo.launch.py，能正常启动RViz2并加载机器人模型，说明配置成功。

## 4.7 本章小结

好，本章我们用MoveIt2 Setup Assistant完成了机器人的快速配置，核心步骤是：加载URDF模型、创建三个规划组（left_arm、right_arm、dual_arms）、设置末端执行器、生成配置包。下一章我们学习MoveIt2的Python API，用代码控制机器人运动。

# 第5章：MoveIt2 Python API Basics（MoveIt2 Python接口基础）

## 5.1 本章前言：为什么用Python API

各位同学，这一章我们进入实操核心——用Python代码控制机器人运动。MoveIt2提供了专门的Python API（MoveItPy），语法简单，不用写复杂的C++代码，适合快速上手，我们这门课全程用Python，大家跟着我写代码，就能实现机器人的基本控制。

## 5.2 MoveItPy 简介与环境准备

首先，我们了解一下MoveItPy，它是MoveIt2专门为Python用户提供的接口，能让我们通过简单的代码，实现机器人的运动规划、轨迹执行等功能。

环境准备：我们需要安装MoveItPy的依赖包，打开终端，输入sudo apt install ros-humble-moveit-py，安装完成后，确认我们上一章生成的配置包已经编译成功，并且source了环境。

## 5.3 核心代码框架（连接规划组）

我们先写一个最基础的代码框架，实现“连接规划组”的功能，大家打开终端，用vim或者VS Code创建一个Python文件，命名为moveit_dual_arm_basic.py，然后复制下面的代码，我逐行讲解。

代码如下：

import rclpy
from rclpy.node import Node
from moveitpy import MoveItPy

def main(args=None):
    # 初始化ROS2节点
    rclpy.init(args=args)
    # 创建MoveItPy实例，加载配置包
    moveit = MoveItPy(node_name="moveit_py_node", config_package_name="dual_arm_moveit_config")
    # 连接左臂规划组
    left_arm = moveit.get_planning_component("left_arm")
    # 连接右臂规划组
    right_arm = moveit.get_planning_component("right_arm")
    # 连接双臂规划组
    dual_arms = moveit.get_planning_component("dual_arms")

    print("规划组连接成功！")

    # 保持节点运行
    rclpy.spin(moveit.node)
    # 关闭节点
 rclpy.shutdown()

if __name__ == "__main__":
 main()

逐行讲解：第一部分是导入依赖包，rclpy是ROS2的Python接口，MoveItPy是MoveIt2的Python接口；然后初始化ROS2节点，创建MoveItPy实例，加载我们上一章生成的配置包；接着通过get_planning_component方法，连接三个规划组；最后保持节点运行。

## 5.4 设置目标点位（关节角度/位姿）

连接规划组后，我们来设置机器人的目标点位，主要有两种方式：关节角度目标和位姿目标。

1. 关节角度目标：直接设置每个关节的角度，比如让左臂的肩关节旋转90度，代码如下（添加到main函数中）：

# 设置左臂关节角度目标（示例角度，大家可根据自己的机器人调整）
left_arm.set_joint_value_target({"left_arm_joint1": 0.0, "left_arm_joint2": 1.57, "left_arm_joint3": 0.0, "left_arm_joint4": 0.0, "left_arm_joint5": 0.0, "left_arm_joint6": 0.0})

2. 位姿目标：设置末端执行器（夹爪）的位置和姿态，比如让左夹爪移动到某个坐标位置，代码如下：

from geometry_msgs.msg import Pose
# 设置左夹爪位姿目标
pose = Pose()
pose.position.x = 0.5
pose.position.y = 0.2
pose.position.z = 0.8
pose.orientation.w = 1.0  # 姿态（四元数，这里是默认姿态）
left_arm.set_pose_target(pose)

## 5.5 规划与执行运动

设置好目标点位后，我们进行运动规划和执行，代码如下（添加到目标点位设置之后）：

# 规划运动轨迹
plan_result = left_arm.plan()
# 判断规划是否成功
if plan_result.success:
    # 执行轨迹
    left_arm.execute(plan_result.trajectory, wait=True)
    print("左臂运动执行成功！")
else:
    print("左臂运动规划失败！")

讲解：plan()方法用于规划轨迹，返回规划结果；如果规划成功，用execute()方法执行轨迹，wait=True表示等待执行完成后再继续；如果规划失败，会打印失败提示，大家可以检查目标点位是否超出关节限制。

我们运行代码，输入ros2 run dual_arm_moveit_config moveit_dual_arm_basic.py，就能看到左臂按照我们设置的目标运动，RViz2中会显示运动轨迹。

## 5.6 本章小结

好，本章我们掌握了MoveIt2 Python API的基础用法，核心是：连接规划组、设置目标点位（关节角度/位姿）、规划并执行运动。下一章我们重点学习单臂运动规划，练熟单臂的控制，再过渡到双臂。

# 第6章：Single Arm Motion Planning（单臂运动规划）

## 6.1 本章前言：先练熟单臂，再学双臂

各位同学，这一章我们重点练熟单臂的运动规划——因为双臂协调的基础是单臂控制，只有先能熟练控制左臂、右臂单独运动，后续才能实现双臂协同。本章我们围绕单臂的核心操作：关节运动、直线运动、简单抓取，一步步实操。

## 6.1 单臂关节空间规划（关节角度控制）

关节空间规划，就是直接控制机器人的每个关节到达指定角度，适合精准控制关节姿态。我们以上一章的代码为基础，修改目标关节角度，实现左臂的关节运动。

示例代码（左臂关节运动）：

# 左臂关节空间规划：到达“home”姿态（示例角度）
left_arm.set_joint_value_target({"left_arm_joint1": 0.0, "left_arm_joint2": 0.0, "left_arm_joint3": 0.0, "left_arm_joint4": 0.0, "left_arm_joint5": 0.0, "left_arm_joint6": 0.0})
plan_result = left_arm.plan()
if plan_result.success:
    left_arm.execute(plan_result.trajectory, wait=True)
    print("左臂回到home姿态！")

大家可以修改关节角度的值，比如把left_arm_joint2改成1.57（90度），运行代码，看看左臂的运动变化。注意：关节角度不能超出URDF中定义的限制，否则规划会失败。

## 6.2 单臂笛卡尔路径规划（直线运动）

笛卡尔路径规划，就是控制末端执行器（夹爪）沿直线运动，适合需要精准定位的场景（比如抓取物体时，夹爪沿直线靠近物体）。我们以右臂为例，实现直线运动。

示例代码（右臂直线运动）：

# 先设置右臂初始位姿
right_arm.set_joint_value_target({"right_arm_joint1": 0.0, "right_arm_joint2": 0.0, "right_arm_joint3": 0.0, "right_arm_joint4": 0.0, "right_arm_joint5": 0.0, "right_arm_joint6": 0.0})
right_arm.plan_and_execute(wait=True)  # 规划并执行初始姿态

# 设置笛卡尔路径目标（沿x轴移动0.2米）
from moveitpy.utils import pose_to_list
current_pose = right_arm.get_current_pose().pose
target_pose = current_pose
target_pose.position.x += 0.2  # 沿x轴正方向移动0.2米
right_arm.set_pose_target(target_pose)
# 规划并执行笛卡尔路径
right_arm.plan_and_execute(wait=True)
print("右臂直线运动完成！")

讲解：plan_and_execute()方法是plan()和execute()的组合，更简洁；我们先获取右臂当前的位姿，然后修改x轴位置，实现直线运动。大家运行代码，能看到右臂夹爪沿直线移动。

## 6.3 单臂简单抓取与夹爪控制

接下来我们实现单臂的简单抓取，核心是“移动到物体位置→夹爪关闭→移动到目标位置→夹爪打开”，步骤如下：

1. 控制左臂移动到物体上方（位姿目标）；
2. 控制左夹爪打开；
3. 控制左臂下降，靠近物体；
4. 控制左夹爪关闭，抓取物体；
5. 控制左臂上升，离开物体；
6. 控制左臂移动到目标位置；
7. 控制左夹爪打开，放下物体。

示例代码（左夹爪抓取）：

# 1. 左臂移动到物体上方
object_pose = Pose()
object_pose.position.x = 0.5
object_pose.position.y = 0.2
object_pose.position.z = 0.9  # 物体上方10cm
object_pose.orientation.w = 1.0
left_arm.set_pose_target(object_pose)
left_arm.plan_and_execute(wait=True)

# 2. 打开左夹爪（通过服务调用）
from rclpy.node import Node
from std_srvs.srv import SetBool
node = Node("gripper_control_node")
cli = node.create_client(SetBool, "/left_gripper/command")
req = SetBool.Request()
req.data = True  # True=打开，False=关闭
cli.wait_for_service()
cli.call_async(req)
print("左夹爪打开！")

# 3. 左臂下降，靠近物体
object_pose.position.z = 0.8  # 物体位置
left_arm.set_pose_target(object_pose)
left_arm.plan_and_execute(wait=True)

# 4. 关闭左夹爪，抓取物体
req.data = False
cli.call_async(req)
print("左夹爪关闭，抓取成功！")

# 5. 左臂上升
object_pose.position.z = 0.9
left_arm.set_pose_target(object_pose)
left_arm.plan_and_execute(wait=True)

## 6.4 单臂避障基本使用

MoveIt2自带避障功能，只要我们在RViz2中添加碰撞物体，MoveIt2规划轨迹时就会自动避开障碍物。实操步骤：

1. 启动demo：ros2 launch dual_arm_moveit_config demo.launch.py；
2. 在RViz2左侧“Displays”面板，找到“Planning Scene”，点击“Add”，选择“Box”，设置障碍物的位置和尺寸；
3. 运行我们的单臂运动代码，MoveIt2会自动规划避开障碍物的轨迹，不会碰撞。

大家可以试试，添加一个障碍物在左臂运动路径上，再运行代码，看看轨迹是否会绕开障碍物。

## 6.5 本章小结

好，本章我们练熟了单臂的核心操作：关节空间规划、笛卡尔直线运动、简单抓取，以及避障的基本使用。大家一定要多实操，确保能独立控制左臂和右臂单独运动，下一章我们学习双臂的基础协调控制。

# 第7章：Dual-Arm Basic Coordination（双臂基础协调控制）

## 7.1 本章前言：双臂协调的核心是什么

各位同学，这一章我们进入课程的核心——双臂基础协调控制。双臂协调，简单说就是让左臂和右臂配合运动，而不是各自独立运动。我们不搞复杂的协同算法，重点掌握两种最常用的协调模式：左右臂独立控制（分时运动）和双臂同步运动，学完就能实现简单的双臂搬运。

## 7.1 双臂独立控制（分时运动）

双臂独立控制，就是先控制一个手臂运动完成，再控制另一个手臂运动，适合不需要同步的场景（比如左臂抓取物体，右臂准备接收）。我们用之前的代码，实现“左臂抓取→右臂移动到接收位置”的分时运动。

示例代码：

# 1. 左臂抓取物体（复用第6章的抓取代码）
# 省略抓取代码，参考第6章，抓取完成后左臂保持抓取姿态

# 2. 右臂移动到接收位置（物体传递位置）
receive_pose = Pose()
receive_pose.position.x = 0.5
receive_pose.position.y = -0.2  # 右臂在左臂右侧
receive_pose.position.z = 0.9
receive_pose.orientation.w = 1.0
right_arm.set_pose_target(receive_pose)
right_arm.plan_and_execute(wait=True)
print("右臂到达接收位置！")

# 3. 左臂移动到传递位置，靠近右臂
pass_pose = Pose()
pass_pose.position.x = 0.5
pass_pose.position.y = 0.0  # 双臂中间位置
pass_pose.position.z = 0.9
left_arm.set_pose_target(pass_pose)
left_arm.plan_and_execute(wait=True)
print("左臂到达传递位置！")

讲解：这种模式很简单，就是分步骤控制两个手臂，先完成左臂的动作，再完成右臂的动作，适合简单的物体传递场景。大家运行代码，能看到左臂抓取后，右臂移动到指定位置，左臂再移动过去准备传递。

## 7.2 双臂同步运动（同时运动）

双臂同步运动，就是让左臂和右臂同时开始运动、同时结束运动，适合需要保持相对姿态的场景（比如双臂共同搬运一个物体，保持物体水平）。核心是使用“dual_arms”规划组，同时设置两个手臂的目标点位。

示例代码（双臂同步上升）：

# 1. 先让双臂回到初始姿态
left_arm.set_joint_value_target({"left_arm_joint1": 0.0, "left_arm_joint2": 0.0, "left_arm_joint3": 0.0, "left_arm_joint4": 0.0, "left_arm_joint5": 0.0, "left_arm_joint6": 0.0})
right_arm.set_joint_value_target({"right_arm_joint1": 0.0, "right_arm_joint2": 0.0, "right_arm_joint3": 0.0, "right_arm_joint4": 0.0, "right_arm_joint5": 0.0, "right_arm_joint6": 0.0})
left_arm.plan_and_execute(wait=True)
right_arm.plan_and_execute(wait=True)

# 2. 设置双臂同步上升的目标位姿（同时上升0.2米）
# 获取左臂当前位姿
left_current = left_arm.get_current_pose().pose
left_target = left_current
left_target.position.z += 0.2
# 获取右臂当前位姿
right_current = right_arm.get_current_pose().pose
right_target = right_current
right_target.position.z += 0.2

# 3. 使用dual_arms规划组，同时设置两个手臂的目标
dual_arms.set_pose_targets({"left_arm": left_target, "right_arm": right_target})
# 规划并执行同步运动
dual_arms.plan_and_execute(wait=True)
print("双臂同步上升完成！")

讲解：这里我们使用dual_arms规划组的set_pose_targets方法，同时设置左臂和右臂的目标位姿，这样MoveIt2会规划一条让两个手臂同时运动、同时到达目标的轨迹，实现同步运动。

## 7.3 双臂间避障

双臂协调运动时，最关键的是避免两个手臂互相碰撞。MoveIt2会自动处理双臂间的避障，只要我们在配置规划组时，正确设置了碰撞矩阵，MoveIt2规划轨迹时就会自动避开两个手臂之间的碰撞。

实操测试：我们设置一个双臂交叉的目标位姿，运行代码，看看MoveIt2是否会规划避障轨迹。如果规划失败，说明目标位姿过于极限，调整目标位姿即可。

## 7.4 双臂简单搬运实战

我们结合前面的内容，实现一个简单的双臂搬运demo：双臂同步下降→同时抓取物体→同步上升→同步移动到目标位置→同步下降→同时松开物体。

步骤：

1. 双臂同步移动到物体上方；
2. 双臂同步下降，靠近物体；
3. 左右夹爪同时关闭，抓取物体；
4. 双臂同步上升；
5. 双臂同步移动到目标位置；
6. 双臂同步下降；
7. 左右夹爪同时打开，放下物体。

大家可以结合前面的代码，自己编写这个demo，有问题的同学举手，我过来指导。

## 7.5 本章小结

好，本章我们掌握了双臂的基础协调控制：独立控制（分时运动）和同步运动，以及双臂间的避障，还完成了简单的双臂搬运demo。下一章我们做综合实验，跑通完整的双臂抓取-放置流程，并学习常见的调试方法。

# 第8章：Final Demo & Debug（综合实验与调试）

## 8.1 本章前言：综合实验的目标

各位同学，这一章是我们课程的最后一章，核心目标是：跑通一个完整的双臂综合demo（抓取-传递-放置），同时学习常见的错误与调试方法，确保大家学完之后，能独立解决实操中遇到的问题。

## 8.1 综合实验：双臂抓取-传递-放置完整Demo

我们整合前面所有的知识点，编写一个完整的demo，实现以下功能：

1. 左臂移动到物体位置，抓取物体；
2. 左臂上升，移动到传递位置；
3. 右臂移动到传递位置，准备接收；
4. 左臂松开物体，右臂抓取物体；
5. 右臂上升，移动到目标位置；
6. 右臂下降，松开物体，完成放置。

完整代码（大家复制到VS Code，保存为dual_arm_final_demo.py）：

import rclpy
from rclpy.node import Node
from moveitpy import MoveItPy
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool

def gripper_control(node, gripper_topic, open_flag):
    # 夹爪控制函数：open_flag=True打开，False关闭
    cli = node.create_client(SetBool, gripper_topic)
 while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("服务未就绪，等待...")
    req = SetBool.Request()
    req.data = open_flag
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().success

def main(args=None):
    # 初始化ROS2节点
    rclpy.init(args=args)
    node = Node("dual_arm_final_demo")
    # 创建MoveItPy实例
    moveit = MoveItPy(node_name="moveit_py_node", config_package_name="dual_arm_moveit_config")
    left_arm = moveit.get_planning_component("left_arm")
    right_arm = moveit.get_planning_component("right_arm")

    # 1. 左臂抓取物体
    node.get_logger().info("开始左臂抓取...")
    # 物体位置
    object_pose = Pose()
 object_pose.position.x = 0.5
    object_pose.position.y = 0.2
    object_pose.position.z = 0.8
    object_pose.orientation.w = 1.0
    # 移动到物体上方
    left_arm.set_pose_target(Pose(position=object_pose.position, orientation=object_pose.orientation, z=0.9))
    left_arm.plan_and_execute(wait=True)
    # 打开左夹爪
    gripper_control(node, "/left_gripper/command", True)
    # 下降抓取
    left_arm.set_pose_target(object_pose)
    left_arm.plan_and_execute(wait=True)
    # 关闭左夹爪
    gripper_control(node, "/left_gripper/command", False)
    node.get_logger().info("左臂抓取成功！")
    # 上升
    left_arm.set_pose_target(Pose(position=object_pose.position, orientation=object_pose.orientation, z=0.9))
    left_arm.plan_and_execute(wait=True)

 # 2. 左臂移动到传递位置
    pass_pose = Pose()
    pass_pose.position.x = 0.5
    pass_pose.position.y = 0.0
    pass_pose.position.z = 0.9
    pass_pose.orientation.w = 1.0
    left_arm.set_pose_target(pass_pose)
    left_arm.plan_and_execute(wait=True)
    node.get_logger().info("左臂到达传递位置！")

    # 3. 右臂移动到传递位置
    right_arm.set_pose_target(pass_pose)
    right_arm.plan_and_execute(wait=True)
    # 打开右夹爪
    gripper_control(node, "/right_gripper/command", True)
    node.get_logger().info("右臂到达传递位置！")

    # 4. 左臂松开，右臂抓取
    gripper_control(node, "/left_gripper/command", True)
    # 右臂微调位置，抓取物体
    right_pose = right_arm.get_current_pose().pose
    right_pose.position.z -= 0.05
    right_arm.set_pose_target(right_pose)
    right_arm.plan_and_execute(wait=True)
    gripper_control(node, "/right_gripper/command", False)
    node.get_logger().info("物体传递完成！")

    # 5. 右臂移动到目标位置，放下物体
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = -0.2
    target_pose.position.z = 0.8
    target_pose.orientation.w = 1.0
    # 上升
    right_arm.set_pose_target(Pose(position=target_pose.position, orientation=target_pose.orientation, z=0.9))
    right_arm.plan_and_execute(wait=True)
    # 移动到目标位置上方
    right_arm.set_pose_target(target_pose)
    right_arm.plan_and_execute(wait=True)
    # 松开右夹爪
    gripper_control(node, "/right_gripper/command", True)
    node.get_logger().info("物体放置完成！")

    # 6. 双臂回到初始姿态
    left_arm.set_joint_value_target({"left_arm_joint1": 0.0, "left_arm_joint2": 0.0, "left_arm_joint3": 0.0, "left_arm_joint4": 0.0, "left_arm_joint5": 0.0, "left_arm_joint6": 0.0})
    right_arm.set_joint_value_target({"right_arm_joint1": 0.0, "right_arm_joint2": 0.0, "right_arm_joint3": 0.0, "right_arm_joint4": 0.0, "right_arm_joint5": 0.0, "right_arm_joint6": 0.0})
    left_arm.plan_and_execute(wait=True)
    right_arm.plan_and_execute(wait=True)

    node.get_logger().info("综合Demo执行完成！")
    # 关闭节点
    rclpy.shutdown()

if __name__ == "__main__":
    main()

## 8.3 代码运行与测试

我们运行这个demo，步骤如下：

1. 打开终端，启动MoveIt2配置包的demo：ros2 launch dual_arm_moveit_config demo.launch.py；
2. 打开另一个终端，进入ROS2工作空间，source install/setup.bash；
3. 运行代码：ros2 run dual_arm_moveit_config dual_arm_final_demo.py；
4. 观察RViz2中机器人的运动，看看是否能完成“抓取-传递-放置”的完整流程。

如果运行成功，说明大家已经掌握了课程的核心内容；如果失败，我们接下来学习调试方法。

## 8.4 常见错误与调试方法

实操中最常见的3个错误，以及对应的调试方法，大家记好：

1. 规划失败（plan_result.success = False）
原因：目标位姿超出关节限制、碰撞、机器人模型配置错误。
调试：① 调整目标位姿，确保在关节运动范围内；② 检查RViz2中是否有障碍物，删除多余障碍物；③ 重新运行MoveIt2 Setup Assistant，检查规划组配置是否正确。

2. 夹爪控制失败（无法打开/关闭）
原因：服务话题名称错误、夹爪关节配置错误。
调试：① 用ros2 service list查看夹爪服务的正确名称，修改代码中的话题名称；② 检查MoveIt2 Setup Assistant中末端执行器的关节配置是否正确。

3. 双臂碰撞
原因：碰撞矩阵配置错误、目标位姿过于极限。
调试：① 重新运行MoveIt2 Setup Assistant，生成新的碰撞矩阵；② 调整目标位姿，避免双臂交叉过于紧密。

## 8.5 硬件对接思路（简单介绍）

最后，简单提一下硬件对接的思路：我们现在用的是仿真环境，实际对接真实双臂机器人时，只需要修改“执行器接口”——将MoveIt2的轨迹指令，通过ros_control发布到机器人的硬件接口，就能控制真实机器人运动。具体的硬件对接，需要根据机器人的型号（比如Yumi、自定义双臂）调整，但核心逻辑和我们课程中学的一致。

## 8.6 课程总结

各位同学，我们的ROS2 + MoveIt2 双臂机器人精简课程就结束了。回顾一下，我们从环境搭建、ROS2基础、机器人模型，到MoveIt2配置、Python API、单臂控制、双臂协调，最后跑通了完整的综合demo，全程围绕实操，没有复杂理论。

大家课后多练习，熟练掌握单臂和双臂的基本控制，后续可以根据自己的需求，拓展更复杂的功能（比如力控制、动态避障）。希望大家通过这门课，能快速上手双臂机器人的MoveIt2控制，祝大家学习顺利！
> （注：文档部分内容可能由 AI 生成）
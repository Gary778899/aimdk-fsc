# PICO → X2 双臂遥操：开源项目调查汇总

整理日期：2026-10-29。

当前目标：保留 X2 原厂运控，在 URS（`UPPERBODY_REMOTE_SPLIT`）模式下，将 PICO 双手柄的相对位姿映射为双臂 TCP 目标，经 IK 生成完整 14 关节目标，并接入现有 AimDK 上肢接口。视频回传与夹爪开合是相关扩展功能。

本文汇总本轮对话检索到的相关仓库和项目资料，不纳入无关搜索结果、论坛讨论及通用百科。按三类整理：表一归集 X2 Ultra / X2 平台项目，表二归集上半身遥操及其输入、IK、视频组件，表三归集后续研究拓展。项目只出现一次，X2 平台项目优先归入表一。文档仅标注 X2、未明确 Ultra 的条目保留型号待核实说明，不将其自动认定为已适配本机。文档网站、论文项目页与源码仓库分别标明。

调查深度分为“源码审阅”“文档查阅”“检索线索”。只有 X2 IK 项目完成了较深入的机器人端源码审阅；未执行依赖安装、机器人容器构建、运行或真机测试。各项目声明的支持范围不等于本项目的验收结论。

## 表一：基于 X2 Ultra / X2 平台开发的项目

优先关注本平台的模型、IK、上肢控制、视觉抓取和系统集成。明确使用 X2 Ultra 模型与仅标注 X2 的项目在平台列中区分；ILLEON 为平台文档线索，尚未确认源码开放。

| 项目及链接 | 项目范围、实现功能 | 硬件与软件平台 | 参考价值与用途 | 迁移边界与调查依据 |
| --- | --- | --- | --- | --- |
| [Agibot-X2-IK-upper-body-control](https://github.com/AgiBot-Community/Agibot-X2-IK-upper-body-control) | X2 上肢 MDI 调试与 Python MoveJ；单臂笛卡尔轨迹、双臂同步关节运动；SRS 几何候选、SEW 臂角、数值精修及连续解支跟踪；腕原点、灵巧手、夹爪与自定义 TCP | X2 Ultra 模型；机器人端 Python ≥3.10、NumPy、ROS 2/AimDK；桌面 Linux x86_64 AppImage；要求操作者预先进入 URS | **优先评估的 X2 模型和 IK 基础**。复用左右臂求解器、TCP 变换及连续跟踪，再增加双臂统一调度和 PICO 输入 | **源码审阅**：提交 `4445980`，解包 `x2ik-2.1.0` wheel。MDI 一次选择一侧；公开 `Robot` 只有 MoveJ，无双臂同步笛卡尔接口。夹爪零值发送、固定位置偏置补偿、限位裁剪与反馈检查需调整。详见 [项目原理](https://github.com/AgiBot-Community/Agibot-X2-IK-upper-body-control/blob/main/docs/PROJECT_OVERVIEW.md)、[模块说明](https://github.com/AgiBot-Community/Agibot-X2-IK-upper-body-control/blob/main/docs/MODULE_GUIDE.md)。源码可阅读不等于项目自有代码许可已明确，迁移前须核实授权 |
| [AgiBot-Community/raicom2026-agibot-X2](https://github.com/AgiBot-Community/raicom2026-agibot-X2) | 竞赛任务系统，包含语音、导航、视觉、抓取执行器及 `x2_ik_sdk` 目录说明 | 文档标注 X2，Ultra 型号适配待核实；AimDK、Ubuntu 22.04、ROS 2 Humble、Python 3.10；推荐 Orin NX/ARM64 | 可参考 X2 抓取集成；不是 PICO 遥操。尚未确认其 `x2_ik_sdk` 与 X2 IK upper-body-control 项目的关系、完整源码及连续双臂能力；SDK 许可独立于仓库原始代码 MIT 说明 | 文档查阅，未展开 SDK 核心 |
| [AgiBot-Community/agibot-X2-grasp-demo](https://github.com/AgiBot-Community/agibot-X2-grasp-demo) | ROS 2 视觉抓取：AprilTag / Grounding 目标获取、RGB-D 定位、TF 转换、多初值 Pinocchio IK、分段笛卡尔规划、机械臂与夹爪执行；Grasp Action 提供反馈与取消 | 文档标注 AgiBot X2，Ultra 型号适配待核实；Ubuntu 22.04、ROS 2 Humble、AimDK、Python 3.10；C++17/pybind11 IK 后端及 Python 回退；RGB-D 相机 | 可借鉴 X2 抓取编排、夹爪控制、IK 后端分离与 50 Hz C++ 发布器；也可作为后续视觉辅助遥操基础 | **README 查阅，未审阅源码或运行**。自动选臂不等于双臂同步；内置简化 URDF 不包含完整碰撞模型。AprilTag 本地运行，Grounding 使用远端视觉 API；不是 PICO 遥操实现，也未由本次审阅确认 URS 契约与本项目一致 |
| [TaiE64/AgibotX2-Retargeter](https://github.com/TaiE64/AgibotX2-Retargeter) | SMPL-X/BVH/Xsens 人体动作到 X2 Ultra 的全身重定向，描述含 GMR IK、碰撞约束及质量检查 | AgiBot X2 Ultra 模型；人体动作数据 | X2 模型与全身重定向线索；不是已确认的 PICO 在线双臂控制项目 | 仓库主页与检索线索 |
| [ILLEON X2 平台文档](https://docs.illeon.de/x2.html)（平台说明） | 描述 X2 笛卡尔操控与 SpaceMouse、Quest、外骨骼输入接入 | 文档标注 AGIBOT X2，Ultra 型号适配待核实；Meta Quest 等输入设备 | 说明已有 X2 外部输入到 IK 的系统路线；未确认源码开放和 PICO 支持，不能列为可直接迁移的开源代码 | 文档检索线索 |

## 表二：与上半身遥操作开发强相关的项目

包含宇树等双臂遥操实现，以及 XRoboToolkit 的客户端、通信、输入接口、示例和视频链路。强相关不代表全部需要引入；不同机器人驱动与控制模式需独立适配。

| 项目及链接 | 项目范围、实现功能 | 硬件与软件平台 | 参考价值与用途 | 迁移边界与调查依据 |
| --- | --- | --- | --- | --- |
| [XRoboToolkit-Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client) | XR 端交互应用，采集跟踪与控制输入，并显示机器人视频 | PICO/Unity/Android；组织入门说明列出的测试设备为 PICO 4 Ultra | **PICO 前端首选**。初期使用发布版 APK，界面定制阶段再修改源码 | **文档查阅**。客户端提供输入与显示，不负责 X2 IK；不同 PICO 型号、系统版本和具体功能权限需分别核实 |
| [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) | XR 数据接收服务及 SDK，负责设备连接与数据访问 | 文档涉及 Windows、Linux x86_64、Linux ARM64；提供 Ubuntu 安装包与源码构建方式 | **XR 通信基础组件**，连接头显与上层遥操程序 | **文档查阅**。不是机器人驱动；需匹配运行平台与 SDK 版本，amd64 包不能用于 Orin ARM64 |
| [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) | Python `xrobotoolkit_sdk`，读取手柄/头显位姿、扳机、Grip、按钮、摇杆和时间戳等 | Python ≥3.10；Linux x86_64/aarch64、Windows x64；运行时需要 PC Service | **Python 输入层首选**，向 X2 遥操模块提供原始 XR 状态 | **文档查阅**。时间戳需核实语义；读取位姿不等于已经完成坐标标定、相对运动映射或硬件同步 |
| [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) | XR 输入到机器人运动的应用框架；Placo QP IK、MuJoCo/可视化及 UR5、ARX R5、Galaxea R1 Lite 等硬件示例 | PICO；示例安装说明主要基于 Ubuntu 22.04；机器人模型与驱动按示例分别配置 | 参考 Grip 使能、末端目标配置和遥操应用组织；Placo 可作为 X2 专用 IK 之外的对照方案 | **文档查阅**。没有确认现成 X2 驱动；不必同时引入两套 IK。详见 [teleop_details.md](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/main/teleop_details.md) |
| [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) | 人形机器人上肢遥操，支持手柄/手部输入；G1 等双臂 IK、末端执行器及记录功能 | PICO 4 Ultra Enterprise、Quest 3、Apple Vision Pro；Unitree G1、H1/H1_2 等，具体型号见仓库 | **双臂遥操架构与 IK 对照参考**；可借鉴独立运动学模块与硬件执行层的组织方式 | **文档及部分源码审阅**：[robot_arm_ik.py](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py) 使用 Pinocchio、CasADi/IPOPT。X2 应替换模型及执行接口，不搬用宇树电机命令、增益或力矩控制 |
| [XRoboToolkit-Orin-Video-Sender](https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender) | 摄像头预览、H.264 编码、TCP/UDP 发送；提供与 Unity 视频显示的连接方式 | NVIDIA Jetson Orin；Webcam/ZED；GStreamer | **视频回传阶段的直接参考**，不属于双臂 IK 必需依赖 | **文档查阅**。X2 摄像头输入若为 ROS 图像，需要适配；Unity 接收协议与发送配置需匹配，不能将“支持 Orin”视为可直接接现有相机 |
| [XRoboToolkit-Teleop-Sample-Cpp](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Cpp) | 双 UR5 与 Dynamixel 头部遥操示例，使用 XRoboToolkit 数据链路 | PICO 4 Ultra、双 UR5、Dynamixel；Linux 主分支及 Windows 分支 | 可参考 C++ 集成；当前 Python/X2 专用 IK 路线不需要依赖 UR RTDE 或 Dynamixel 驱动 | 文档查阅 |
| [BRobotAssistantLib](https://github.com/XR-Robotics/BRobotAssistantLib) | Unity 机器人接口相关 Android 库组件 | Android/PICO 客户端侧 | 底层客户端定制时关注；使用现成 APK 时无需独立开发 | 组织主页说明 |
| [XRoboToolkit-RobotVision-PC](https://github.com/XR-Robotics/XRoboToolkit-RobotVision-PC) | PC 视频采集、H.264 编解码、TCP 传输、播放器与延迟分析工具 | PC 摄像头与视频收发环境；具体构建条件见仓库 | 可用于视频链路调试或 PC 采集备选，不参与运动学 | 文档查阅 |
| [XRoboToolkit-Unity-Client-Quest](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest) | XRoboToolkit 的 Quest 客户端 | Meta Quest/Unity | 将来更换头显时参考；PICO 路线不需要 | 组织主页说明 |
| [OpenMind/humanoid_teleoperate](https://github.com/OpenMind/humanoid_teleoperate) | 与宇树 XR 遥操代码路线相关的人形机器人控制项目，文档列出手柄输入和多种末端执行器 | Unitree G1/H1 系列；PICO 4 Ultra Enterprise、Quest 3、Apple Vision Pro | 功能与宇树官方参考重叠，优先研究上游；具体差异未审阅 | README 检索线索 |
| [YanjieZe/Humanoid-Teleoperation](https://github.com/YanjieZe/Humanoid-Teleoperation) | 人形双臂与手部遥操，属于 iDP3 相关工程 | Fourier GR1、Inspire Hands、RealSense；Apple Vision Pro | 可参考跨机器人重定向与硬件适配；不是直接的 PICO 实现 | README 查阅 |
| [Isaac ROS Unitree G1 Teleop](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_physical_ai/isaac_ros_unitree_g1_teleop_bringup/index.html)（官方文档入口） | G1 遥操 bringup，组合双臂 IK、手部控制和移动控制，支持仿真与硬件 | G1、PICO 4 Ultra/Quest 3；文档验证平台包括 Jetson AGX Thor 与 x86_64 | 完整系统参考，但依赖与版本跨度较大；不能默认兼容当前 Orin/Humble 环境 | 官方文档查阅，未审阅实现 |
| [1EastonJ/vive_g1_hfbody](https://github.com/1EastonJ/vive_g1_hfbody) | G1 上半身遥操，UDP 位姿输入、Mink QP IK 与 MuJoCo 可视化 | G1、HTC Vive/Valve Index；Windows 输入端、Ubuntu 求解端 | 可参考 Mink IK；不是已经验证的 PICO 方案，README 性能声明未独立验证 | README 检索线索 |
| [Arborseek/PiperXR](https://github.com/Arborseek/PiperXR) | 手柄相对位姿映射、Placo IK、单/双臂仿真与实机统一入口、输入录制回放 | AgileX PiPER 六轴臂、PICO 4 Ultra、XRoboToolkit、Ubuntu 22.04 | 相对映射及回放有参考价值；六轴结构不同，且仓库标注非商业使用许可，不能按通用宽松开源许可处理 | README 检索线索 |
| [Seeed-Projects/lerobot-teleoperator-rebot-vr](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-vr) | PICO 位姿映射、位置 QP 与腕部解析分解、双臂协调、MIT/POS_VEL 电机控制 | PICO 4、reBot B601-DM 机械臂 | 可参考输入映射；IK 分解依赖特定机械结构，电机模式、启动与退出行为不适用于当前 X2 契约 | README 检索线索 |

## 表三：面向后续研究与拓展开发的其他项目

主要用于全身运动跟踪、强化学习控制、上下肢协同、部署环境及动态遥操研究；不作为当前 X2 双臂遥操的必需依赖。

| 项目及链接 | 项目范围、实现功能 | 硬件与软件平台 | 参考价值与用途 | 迁移边界与调查依据 |
| --- | --- | --- | --- | --- |
| [NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl)；[PICO 设置文档](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/vr_teleop_setup.html) | 全身控制、运动跟踪与 PICO 遥操；包含 XRoboToolkit 接入资料 | Unitree G1 相关部署；PICO 头显、手柄及全身跟踪所需设备 | 将来研究全身动作时参考；当前保留原厂运控的双臂任务不需要迁移整套策略 | 官方设置文档查阅 |
| [BotRunner64/Teleopit](https://github.com/BotRunner64/Teleopit) | BVH/PICO 输入、全身重定向、MuJoCo 与 G1 部署，包含策略与记录工作流 | PICO 4、Unitree G1、MuJoCo | 偏全身策略执行；不是简单替换模型即可用于 X2 原厂运控的双臂工具 | README 检索线索 |
| [yixxuan-li/OmniClone](https://github.com/yixxuan-li/OmniClone) | Nokov/PICO 输入驱动全身 G1 控制 | Unitree G1、Unitree SDK2/DDS、ROS 2 | 全身遥操线索，尚未核实是否存在适合独立复用的双臂 IK 模块 | 检索线索 |
| [HorizonRobotics/HoloMotion](https://github.com/HorizonRobotics/HoloMotion) | 全身人形控制，检索说明提及 G1 上的 PICO/XRoboToolkit 遥操试验 | Unitree G1、PICO、板载计算环境 | 属于全身控制研究路线，不能直接替代当前双臂执行器 | 仅检索线索，未审阅源码 |
| [Roboparty/GR00T-WholeBodyControl_unitree](https://github.com/Roboparty/GR00T-WholeBodyControl_unitree) | GR00T/Unitree 相关全身控制与手部集成工程 | Unitree 相关平台，具体硬件组合需核对 | 与 NVIDIA 上游重叠，尚未比较差异；当前优先上游文档 | 仅检索线索 |
| [Roboparty/UFO](https://github.com/Roboparty/UFO) | 人形控制的强化学习、动作导入及实机遥操框架 | 检索说明涉及 G1/RP1 | 全身策略研究备选，不是已核实的 PICO 双臂 IK 组件 | 仅检索线索，未审阅源码 |
| [SIMPLE：Teleop Setup](https://psi-lab.ai/SIMPLE/docs/decoupled-wbc/teleop_decoupled_setup.html)（项目文档） | 分离式全身控制遥操设置，使用 PICO/XRoboToolkit | 文档示例为 G1 | 可参考上肢与下肢控制分工；具体源码接口未审阅 | 文档检索线索 |
| [sim2real：Onboard Orin](https://egalahad.github.io/sim2real/getting-started/teleop-onboard-orin/) / [x86_64 设置](https://egalahad.github.io/sim2real/getting-started/teleop-x86-64/)（部署文档） | PICO/XRoboToolkit 在 G1 板载 Orin 与工作站的部署资料 | G1、Orin、x86_64 PC | 可参考环境与部署问题；不是独立核实的 IK 开源实现 | 文档检索线索 |
| [HuMiT](https://humit-ral.github.io/)（研究项目页） | 以紧凑参考实现低延迟全身人形遥操 | Unitree G1 | 适合比较全身跟踪架构；本轮未确认代码可用性及可抽取的上肢 IK | 项目页检索线索 |
| [ExtremControl](https://extremcontrol.github.io/)（研究项目页） | 低延迟全身遥操，研究速度前馈 | G1；项目说明涉及 OptiTrack、Quest 2 与 VIVE 跟踪器 | 不是直接 PICO 方案；可作为未来动态遥操研究参考 | 项目页检索线索 |
| [TWIST](https://xbpeng.github.io/projects/TWIST/TWIST_2025.pdf)（论文入口） | 全身动作模仿与遥操研究 | Unitree G1 | 当前只获取论文线索，未核实该版本代码与 PICO 输入链路；不作为已验证移植方案 | 论文检索线索 |

## 当前建议与复用边界

优先组合为：XRoboToolkit 的 PICO 客户端、PC Service 与 Pybind 负责输入；X2 专用项目提供模型、TCP 与 IK 参考；本工作区负责适配 AimDK 契约、双臂同步执行和数据记录。视频回传后续独立接入。

双臂同步不必从零重写 IK：每个控制周期分别求解左右臂，再统一检查、组装并发布完整 14 关节目标即可形成基础闭环。双手共同搬运、固定相对位姿或碰撞约束等需求，才需要进一步设计耦合任务。PICO 相对位姿映射、使能、重新建立参考和失联处理仍需开发。

X2 IK 项目当前公开的“双臂同步”是 MoveJ 关节插值；MDI 笛卡尔轨迹一次选择一侧。不能把它描述为开箱即用的双臂实时笛卡尔遥操。其最终发送层还存在固定位置偏置补偿、补偿后限位裁剪和夹爪零值目标，与本工作区契约不同；复用数学求解模块不意味着接受这些控制行为。

所有真实运动仍应在匹配的机器人开发板容器中由用户同步后验证。本地文档及源码审阅不构成模型精度、运动安全性或实机遥操性能的验收。

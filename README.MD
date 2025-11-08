# Nav-Neupan: 完整的ROS2自主导航系统

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

Nav-Neupan是一个完整的ROS2自主导航系统，集成了多种先进的导航算法和仿真环境，为室内机器人导航提供完整的解决方案。

> **项目基础**: 本项目基于 [HomeBot-ROS2-Navigation](https://github.com/aimechengineer/HomeBot-ROS2-Navigation) 扩展而来，增加了NeuPAN神经网络导航算法支持。

![机器人仿真](src/bot_nav/mobile_robot_gazebo.gif)

## 🚀 项目特色

- **多算法集成**: 结合传统Nav2导航栈和先进的NeuPAN神经网络导航算法
- **完整仿真环境**: 基于Gazebo的室内房屋环境仿真
- **模块化设计**: 清晰的功能包结构，便于扩展和维护
- **一键启动**: 提供统一的launch文件，简化系统启动流程

## 📦 功能包组成

### 🔧 核心功能包

| 功能包 | 描述 | 状态 |
|--------|------|------|
| `nav_launch` | 统一的系统启动管理器 | ✅ 完成 |
| `neupan_ros2` | NeuPAN神经网络导航算法ROS2封装 | ✅ 完成 |
| `bot_nav` | 机器人平台相关功能包集合 | ✅ 完成 |

### 🤖 机器人平台功能包 (bot_nav/src/)

| 功能包 | 描述 |
|--------|------|
| `robot_description` | 机器人URDF描述、网格模型和配置 |
| `robot_simulation` | Gazebo仿真环境和导航配置 |
| `robot_patrol` | 自主巡逻行为实现 |
| `costmap_converter` | 代价地图转换工具 |
| `teb_local_planner` | 时间弹性带局部规划器 |
| `teb_msgs` | TEB规划器消息定义 |

## 🛠️ 系统要求

### 基础环境
- **操作系统**: Ubuntu 22.04 LTS
- **ROS2版本**: Humble Hawksbill
- **Python版本**: 3.10 或更高

### 依赖包
- **Nav2**: ROS2导航栈
- **SLAM Toolbox**: SLAM建图工具
- **Gazebo**: 机器人仿真环境
- **NeuPAN**: 神经网络导航算法 ([原项目](https://github.com/hanruihua/neupan))

## 📥 安装指南

### 1. 创建ROS2工作空间

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws
```

### 2. 克隆项目

```bash
git clone https://github.com/A-Keeee/neupan_navbot.git
```

### 3. 安装依赖

```bash
# 安装系统依赖
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# 安装NeuPAN (参考原项目文档)
# https://github.com/hanruihua/neupan
```

### 4. 构建工作空间

```bash
cd ~/nav_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 快速开始

### 一键启动完整系统

```bash
# 从工作空间根目录启动
ros2 launch nav_launch nav_neupan_complete.launch.py
```

这个命令将按顺序启动：
1. 🏠 **Gazebo仿真环境** - 房屋场景和机器人模型
2. 🧭 **Nav2导航栈** - 传统导航算法和定位
3. 🧠 **NeuPAN导航节点** - 神经网络智能导航

### 单独启动组件

```bash
# 仅启动仿真环境
ros2 launch robot_simulation house_sim.launch.py

# 仅启动导航系统
ros2 launch robot_simulation autonomous_navigation.launch.py

# 仅启动NeuPAN节点
ros2 launch neupan_ros2 limo_diff_launch.py
```

## 📋 使用说明

### 导航控制

系统启动后，你可以通过以下方式控制机器人：

1. **RViz界面**: 可视化导航和设置目标点
2. **命令行**: 使用ROS2 action客户端发送导航目标
3. **自动巡逻**: 启动巡逻模式进行自主导航

### 参数配置

主要配置文件位于：
- `src/bot_nav/src/robot_simulation/params/nav2_params.yaml` - Nav2导航参数
- `src/neupan_ros2/config/` - NeuPAN算法配置
- `src/bot_nav/src/robot_simulation/config/` - 仿真环境配置

## 🔧 高级配置

### 自定义地图

1. 将你的地图文件放置在 `src/bot_nav/src/robot_simulation/maps/`
2. 更新launch文件中的地图路径参数

### 调整启动时序

如果系统启动时间需要调整，编辑 `src/nav_launch/launch/nav_neupan_complete.launch.py` 中的延迟参数：

```python
# 调整Gazebo启动后的等待时间
autonomous_navigation_launch = TimerAction(period=8.0, ...)

# 调整导航系统启动后的等待时间
neupan_launch = TimerAction(period=15.0, ...)
```

## 📊 系统架构

```
nav-neupan/
├── nav_launch/           # 🚀 统一启动管理器
│   ├── launch/
│   │   ├── nav_neupan_complete.launch.py     # 完整系统启动
│   │   └── nav_neupan_sequential.launch.py   # 顺序启动版本
│   └── README.md
├── neupan_ros2/          # 🧠 NeuPAN导航算法
│   ├── neupan_node.py    # 主节点
│   ├── config/           # 算法配置
│   └── launch/
├── bot_nav/             # 🤖 机器人平台
│   ├── robot_description/# 机器人描述
│   ├── robot_simulation/ # 仿真环境
│   ├── robot_patrol/     # 巡逻功能
│   └── ...               # 其他导航组件
└── README.md            # 本文件
```

## 🐛 故障排除

### 常见问题

**Q: Nav2导航初始化失败**

**A:** 检查地图文件路径是否正确，并确保仿真时间设置一致。

### 调试模式

启用详细日志：
```bash
export ROS_LOG_LEVEL=debug
ros2 launch nav_launch nav_neupan_complete.launch.py
```

## 🤝 贡献指南

欢迎提交Issue和Pull Request！

1. Fork本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

## 📄 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

本项目基于以下开源项目和贡献：

- **HomeBot-ROS2-Navigation**: 机器人平台基础代码 ([aimechengineer/HomeBot-ROS2-Navigation](https://github.com/aimechengineer/HomeBot-ROS2-Navigation))
- **NeuPAN算法**: 神经网络导航算法 ([hanruihua/neupan](https://github.com/hanruihua/neupan))
- **ROS2社区**: 感谢ROS2和Nav2项目的贡献者
- **AgileX**: Limo机器人平台支持

特别感谢原HomeBot项目的作者为ROS2导航社区做出的贡献。

## 📞 联系方式

- **项目维护者**: A-Keeee
- **邮箱**: [邮箱地址]
- **项目主页**: [https://github.com/A-Keeee/neupan_navbot](https://github.com/A-Keeee/neupan_navbot)

---

⭐ 如果这个项目对你有帮助，请给我们一个星标！

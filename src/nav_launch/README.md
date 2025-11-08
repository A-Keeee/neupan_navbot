# nav_launch

完整的nav_neupan导航启动功能包

## 描述

这个ROS2功能包提供了一个完整的launch文件，可以一次性启动整个导航系统，包括：

1. **Gazebo仿真环境** (`house_sim.launch.py`) - 启动带有房屋环境的Gazebo仿真器
2. **自主导航系统** (`autonomous_navigation.launch.py`) - 启动Nav2导航栈
3. **Neupan导航节点** (`limo_diff_launch.py`) - 启动Neupan智能导航节点

## 依赖

- `robot_simulation` - 机器人仿真功能包
- `neupan_ros2` - Neupan导航功能包
- `launch` - ROS2 launch系统
- `launch_ros` - ROS2 launch工具

## 构建

在workspace根目录下：

```bash
cd /home/ake/nav/nav_neupan
colcon build --packages-select nav_launch
source install/setup.bash
```

## 使用方法

### 启动完整导航系统

```bash
ros2 launch nav_launch nav_neupan_complete.launch.py
```

这个命令会按顺序启动所有必需的节点：
- 首先启动Gazebo仿真环境（立即启动）
- 5秒后启动自主导航系统（等待Gazebo初始化完成）
- 10秒后启动Neupan导航节点（等待导航栈初始化完成）

### 调整启动延迟

如果你的系统较慢或需要更多时间来初始化，可以编辑launch文件中的`period`参数：

- `autonomous_navigation_launch`: 默认延迟5秒
- `neupan_launch`: 默认延迟10秒

## 文件结构

```
nav_launch/
├── launch/
│   └── nav_neupan_complete.launch.py  # 主启动文件
├── nav_launch/
│   └── __init__.py
├── resource/
│   └── nav_launch
├── package.xml                         # 功能包配置
├── setup.py                            # Python安装配置
├── setup.cfg                           # Python配置
└── README.md                           # 本文件
```

## 许可证

MIT License

## 维护者

Navigation User <user@todo.todo>

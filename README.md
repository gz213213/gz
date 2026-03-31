# chapt8_ws

## 快速启动
```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 启动 Gazebo 仿真
```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

### 启动 Nav2 导航
```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

## 文档索引
- 导航详细说明：`src/fishbot_navigation2/README.md`


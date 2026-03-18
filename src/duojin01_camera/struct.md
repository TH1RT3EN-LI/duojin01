duojin01_camera/
├── duojin01_camera/              ← Python 包模块目录（节点放这里）
│   ├── __init__.py               ← 标识这是一个 Python 包
│   └── usb_camera_node.py        ← 节点文件 ✓ 位置正确
│
├── resource/
│   └── duojin01_camera           ← ament 索引标记文件（不要改）
│
├── test/
│   ├── test_copyright.py         ← 自动生成的测试
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── package.xml                   ← 包依赖声明
├── setup.cfg                     ← 包元数据（包名、脚本入口前缀）
└── setup.py                      ← 构建配置，entry_points 在这里注册


# 在 duojin01 根目录
colcon build --packages-select duojin01_camera --symlink-install
source install/setup.bash

# 运行（默认 /dev/video0）
ros2 run duojin01_camera usb_camera_node

# 或指定设备和分辨率
ros2 run duojin01_camera usb_camera_node --ros-args \
  -p device_id:=0 -p width:=1280 -p height:=720 -p fps:=30

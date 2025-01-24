# Realsense_depth_value
for studying realsense

I published the depth screen of Realsense2 using the following repository.

깃 클론
```cpp
cd {your workspace}
git clone https://github.com/beomsuchoi/Realsense_depth.git
```

How to use This package
1. 깃 클론

```cpp
cd {your workspace/src}
git clone https://github.com/beomsuchoi/Realsense_depth.git
```

2. 빌드, 소싱

```cpp
cd {your workspace}
colcon build --packages-select realsense_depth
source install/setup.bash

```
3. run

```cpp
ros2 run realsense_depth realsense_depth_node

```

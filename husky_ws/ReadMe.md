# Husky workspace

clone [https://github.com/husky/husky/tree/foxy-devel](https://github.com/husky/husky/tree/foxy-devel)

using Makefile

```bash
make clone_husky
```

then copy `gazebo_cats.launch.py` inside `~/husky_ws/husky/husky_gazebo/launch`

```bash
cp ./husky_ws/gazebo_cats.launch.py ~/husky_ws/husky/husky_gazebo/launch
```

then `colcon build` the folder `husky_ws` then source the install and run

```bash
ros2 launch husky_gazebo gazebo_cats.launch.py
```
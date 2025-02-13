# uwb_simulation

## Architecture

## How to Install

```
$ git clone https://github.com/fleshycat/uwb_reconn.git
$ cd uwb_reconn
$ colcon build
```

## How to Run

```
# Running a Gazebo World with Anchors and Tags
$ ros2 launch uwb_sim robot_in_gazebo_world_run.launch.py
```

```
# Running UWB localization Node
$ ros2 run uwb_localization sqrrange_leastsqr_localization.py
```

## Shortcut in Vscode

```
Command Palette(Ctrl+Shift+P)
Tasks:Run Task(Set your desired shortcut keys)
Select the Model and the World
```

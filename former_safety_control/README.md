# former_safety_control

## parameters

| Data Type | Parameter Name | Default Value  |
|:---------:|:--------------:|:--------------:|
| float | safety_distance | 0.5 |
| float | max_distance | 5.0 |

## predefined topic

| Topic Name | Description |
|:---------:|:--------------:|
| l_sonar_range | get range from left sonar as an input. |
| r_sonar_range | get range from right sonar as an input. |
| cmd_vel_in | input subscribe command |
| cmd_vel_out | output to control command |

## how to run

### base

```shell
ros2 run former_safety_control main_node
ros2 run former_safety_control main_node --ros-args -r safety_distance:=0.5 -r max_distance:=5.0
```

### simulation

```shell
# with namespace
ros2 run former_safety_control main_node --ros-args -r __ns:=/acloi00 -r l_sonar_range:=l_sonar/range -r r_sonar_range:=r_sonar/range -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=base_controller/cmd_vel_unstamped

# without namespace
ros2 run former_safety_control main_node --ros-args -r l_sonar_range:=l_sonar/range -r r_sonar_range:=r_sonar/range -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=base_controller/cmd_vel_unstamped
```

### former robot

```shell
# with namespace
ros2 run former_safety_control main_node --ros-args -r __ns:=/former2_6 -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=base_controller/cmd_vel_unstamped
```

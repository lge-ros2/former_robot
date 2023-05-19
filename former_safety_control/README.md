# former_safety_control

## predefined topic

l_sonar_range: get range from left sonar as an input.

r_sonar_range: get range from right sonar as an input.

cmd_vel: output to stop command

## how to run

```shell
ros2 run former_safety_control main_node --ros-args -r __ns:=/acloi00 -r l_sonar_range:=l_sonar/range -r r_sonar_range:=r_sonar/range -r cmd_vel:=base_controller/cmd_vel_unstamped
```

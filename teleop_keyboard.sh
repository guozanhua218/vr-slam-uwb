#!  /bin/bash

rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=jackal/teleop_keyboard/cmd_vel \
  _speed:=0.2 \
  _turn:=0.4 \
  _repeat_rate:=10 \
  _key_timeout:=0.5 \


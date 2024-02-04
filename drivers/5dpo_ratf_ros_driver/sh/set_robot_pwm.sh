#!/bin/bash
echo "------------------------------------------------------------"
echo "                   SET ROBOT PWM (v|vn|w)                   "
echo "------------------------------------------------------------"

# Arguments error checks
if [[ !($# -eq 2) ]]
then
  echo "Invalid number of arguments (expected: 2 vs $#)"
  echo "./set_robot_pwm.sh <v,vn,w> <pwm value>"
  exit
fi

if [ $1 != "v" ] && [ $1 != "vn" ] && [ $1 != "w" ]
then
  echo "Invalid first argument (expected: v|vn|w vs $1)"
  echo "./set_robot_pwm.sh <v,vn,w> <pwm value>"
  exit
fi

# Set PWM depending on the motion direction desired by the user
# 0: BR  + all wheels with inverted
# 1: BL
# 2: FR
# 3: FL
if [ $1 == "v" ]
then
  echo "Setting PWM for the robot to move in the forward direction (v)..."
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($2)),$((-$2)),$(($2)),$((-$2))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($2)),$((-$2)),$(($2)),$((-$2))]

elif [ $1 == "vn" ]
then
  echo "Setting PWM for the robot to move in the ortogonal direction (vn)..."
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$2)),$((-$2)),$(($2)),$(($2))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$2)),$((-$2)),$(($2)),$(($2))]

else
  echo "Setting PWM for the robot to move with an angular motion (w)..."
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($2)),$(($2)),$(($2)),$(($2))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($2)),$(($2)),$(($2)),$(($2))]

fi

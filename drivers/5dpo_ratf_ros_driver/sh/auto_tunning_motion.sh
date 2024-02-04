#!/bin/bash
echo "------------------------------------------------------------"
echo "          AUTOMATIC TUNNING PROCEDURE (v > vn > w)          "
echo "------------------------------------------------------------"

# Arguments error checks
if [[ !($# -eq 6) ]]
then
  echo "Invalid number of arguments (expected: 6 vs $#)"
  echo "./auto_tunning_motion.sh <#runs per motion> <time per motion state (s)> <v,vn: pwm ini> <v,vn: pwm fin> <w: pwm ini> <w: pwm fin>"
  exit
fi

# Automatic tunning procedure
# - v
echo "Let's start with v motion..."
for i in $(seq 1 $(($1)))
do
  sleep $2
  echo "- run $i:"

  # forward initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($3)),$((-$3)),$(($3)),$((-$3))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($3)),$((-$3)),$(($3)),$((-$3))]
  sleep $2

  # forward final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($4)),$((-$4)),$(($4)),$((-$4))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($4)),$((-$4)),$(($4)),$((-$4))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  sleep $2

  # backward initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$3)),$(($3)),$((-$3)),$(($3))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$3)),$(($3)),$((-$3)),$(($3))]
  sleep $2

  # backward final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$4)),$(($4)),$((-$4)),$(($4))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$4)),$(($4)),$((-$4)),$(($4))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  
done
echo ""

# - vn
echo "Now, go on with a vn motion..."
for i in $(seq 1 $(($1)))
do
  sleep $2
  echo "- run $i:"

  # forward initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$3)),$((-$3)),$(($3)),$(($3))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$3)),$((-$3)),$(($3)),$(($3))]
  sleep $2

  # forward final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$4)),$((-$4)),$(($4)),$(($4))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$4)),$((-$4)),$(($4)),$(($4))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  sleep $2

  # backward initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($3)),$(($3)),$((-$3)),$((-$3))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($3)),$(($3)),$((-$3)),$((-$3))]
  sleep $2

  # backward final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($4)),$(($4)),$((-$4)),$((-$4))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($4)),$(($4)),$((-$4)),$((-$4))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
done
echo ""

# - w
echo "And, finally!!!, let's finish with a w motion!!!"
for i in $(seq 1 $(($1)))
do
  sleep $2
  echo "- run $i:"

  # positive initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($5)),$(($5)),$(($5)),$(($5))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($5)),$(($5)),$(($5)),$(($5))]
  sleep $2

  # positive final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$(($6)),$(($6)),$(($6)),$(($6))]
  rosservice call /unnamed_robot/set_motors_pwm [$(($6)),$(($6)),$(($6)),$(($6))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  sleep $2

  # negative initial state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$5)),$((-$5)),$((-$5)),$((-$5))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$5)),$((-$5)),$((-$5)),$((-$5))]
  sleep $2

  # negative final state
  echo rosservice call /unnamed_robot/set_motors_pwm [$((-$6)),$((-$6)),$((-$6)),$((-$6))]
  rosservice call /unnamed_robot/set_motors_pwm [$((-$6)),$((-$6)),$((-$6)),$((-$6))]
  sleep $2

  # stop state
  echo rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
  rosservice call /unnamed_robot/set_motors_pwm [0,0,0,0]
done
echo ""

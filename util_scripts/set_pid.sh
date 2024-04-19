P=$1
I=$2
D=$3

if [ -z $P ] || [ -z $I ] || [ -z $D ]; then
echo "Please pass PID coeffs as parameters to command (e.g: ./set_pid.sh {P} {I} {D})"
exit
fi

rosparam set /arduino_interface/motor0/pid/p $P
rosparam set /arduino_interface/motor0/pid/i $I
rosparam set /arduino_interface/motor0/pid/d $D

rosparam set /arduino_interface/motor1/pid/p $P
rosparam set /arduino_interface/motor1/pid/i $I
rosparam set /arduino_interface/motor1/pid/d $D

rosparam set /arduino_interface/motor2/pid/p $P
rosparam set /arduino_interface/motor2/pid/i $I
rosparam set /arduino_interface/motor2/pid/d $D

rostopic pub -1 /reconfigure bigbang_eurobot/Reconfigure "target_node: '/arduino_interface'
param_name: ''"
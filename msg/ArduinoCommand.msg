int8 function 
float32 x
float32 y
float32 z
int16 aux

# Functions:
##
# 0 - Motors Command:
# x(-1.0 - 1.0) / y(-1.0 - 1.0) / z(-1.0 - 1.0) (theta)
# aux - ignored
##
# 1 - Servo Command
# x (0-100.0) - Percents
# aux - Servo number
# y and z ignored
##
# 2 - Lcd Command
# x/y/z - ignored
# aux - Wanted Number
##  
##
# 3 - Enable everything
# x/y/z - ignored
# aux - ignored
##  
##
# 4 - Disable operation
# x/y/z - ignored
# aux - ignored
##  
##
# 4 - Disable operation
# x/y/z - ignored
# aux - ignored
##  
##
# 5 - Pump Control
# x/y/z - ignored
# aux > 0 ? HIGH : LOW
##  
##
# 6 - Valve Control
# x/y/z - ignored
# aux > 0 ? HIGH : LOW
##  
# 7 - Led Control
# x/y/z - r/g/b
# aux > 0 ? on : off
##  
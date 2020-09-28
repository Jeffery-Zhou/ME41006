#for potentialfield_car.py and sendmsg.py can't work directly in here(raspberry)!!!

#potentialfield_car.py need to be execute in car
#put potentialfield_car.py in  /home/xtark/ros_ws/src/xtark_ctl/scripts
#$roscore
#open a new terminal(crtl+shift+t)
#$roslaunch xtark_driver xtark_driver,launch
#open a new terminal
#$roslaunch xtark_ctl potentialfieldcar.launch
#execute sendmsg.py here(raspberrypi) or in your computer **remember to check host ip
#after seen "potential_field_planning start" on terminal (car)
#press any key to start
  
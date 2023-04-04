#!/usr/bin/env roseus
(load "package://mycoboteus/mycobot-interface.l")
(load "package://mycoboteus/mycobot-joystick.l")

(ros::roseus-add-msgs "sensor_msgs")

;; mycobot init
(mycobot-init)

;; start teleop node
(joystick-init)



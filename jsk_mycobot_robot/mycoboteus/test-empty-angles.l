#!/usr/bin/env roseus
(require "package://mycoboteus/mycobot-interface.l")

(mycobot-init)
(unix:sleep 3)

(do-until-key
  (send *mycobot* :angle-vector #f(0 0 0 0 0 0))
  (send *ri* :angle-vector (send *mycobot* :angle-vector) 7000)
  (unix:sleep 3)
  (send *mycobot* :angle-vector #f(0 0 130 0 0 0))
  (send *ri* :angle-vector (send *mycobot* :angle-vector) 7000)
  (unix:sleep 3)
  )
(send *ri* :stop-motion)

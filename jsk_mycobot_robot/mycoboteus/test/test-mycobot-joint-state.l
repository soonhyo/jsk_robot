#!/usr/bin/env roseus
(load "package://mycoboteus/mycobot-interface.l")
(load "package://mycoboteus/mycobot-utils.l")

;; Initialize MyCobot environment
(defun setup-mycobot ()
  (mycobot-init)
  (send *ri* :servo-off) ;; Set Servo to off
  (objects (list *mycobot*)))

;; Function to monitor and visualize the angle vector
(defun monitor-angle-vector ()
  (loop
    (let ((current-av (send *ri* :state :potentio-vector)))
      (format t "Current angle vector: ~A~%" current-av)
      (send *mycobot* :angle-vector current-av)
      (send *irtviewer* :draw-objects))
    (unix::usleep 100000))) ;; 100ms delay

;; Main function
(defun main ()
  (setup-mycobot)
  (monitor-angle-vector))

(main)

#!/usr/bin/env roseus
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; Initialize Sciurus17
(unless (boundp '*sciurus17*) (sciurus17-init :safe nil :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables
(defvar *head-target-coords* (make-coords))   ;; Head target coordinates
(defvar *torso-target-coords* (make-coords))  ;; Waist target coordinates

;; Callback functions
(defun head-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *head-target-coords* pose)
    (ros::ros-info "Received head target pose: ~A" pose)
    (send *sciurus17* :head :y :joint-angle (rad2deg (elt (car (rpy-angle (send pose :rot))) 0)))
    (send *sciurus17* :head :p :joint-angle (- (rad2deg (elt (car (rpy-angle (send pose :rot))) 1))))
    (update-robot-state)
    (send *ri* :angle-vector (send *sciurus17* :angle-vector) :fast :head-controller 0.0 :min-time 0.3)
    )
  )

(defun torso-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *torso-target-coords* pose)
    (ros::ros-info "Received waist target pose: ~A" pose)
    (send *sciurus17* :torso :angle-vector (float-vector (rad2deg (elt (car (rpy-angle (send pose :rot))) 0))))
    (update-robot-state)
    (send *ri* :angle-vector (send *sciurus17* :angle-vector) :fast :torso-controller 0.0 :min-time 1.0)))

;; commands

;; predefined pose
;; init pose
(defun command-pose-callback (msg)
  (let ((command-pose-data (send msg :data)))
    (cond
     ((string= command-pose-data "pose/init_pose")
        (progn
          (send *sciurus17* :reset-pose)
          (update-robot-state)
          (send *ri* :angle-vector (send *sciurus17* :angle-vector) 5000))))))

;; A function to update robot state
(defun update-robot-state ()
  (send *irtviewer* :draw-objects)
  (x::window-main-one))

;; Object visualization
(objects (list *sciurus17*))

;; Initialize teleop pose
(send *sciurus17* :reset-pose)
(send *sciurus17* :angle-vector (send *ri* :actual-vector))
(send *sciurus17* :l_arm_joint4 :joint-angle -136)
(send *sciurus17* :r_arm_joint4 :joint-angle 136)
(send *ri* :angle-vector (send *sciurus17* :angle-vector))
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; Subscribe to all topics
(ros::subscribe "/head_target_pose" geometry_msgs::PoseStamped #'head-target-pose-callback)
(ros::subscribe "/waist_target_pose" geometry_msgs::PoseStamped #'torso-target-pose-callback)
(ros::subscribe "/vr_command" std_msgs::String #'command-pose-callback)

;; Main loop
(defun ik-viewer-loop ()
  (ros::rate 30) ;; 30 Hz loop
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

(defun ik-viewer-loop-hz-debug ()
  (ros::rate 50)  ;; 10 Hz loop
  (while (ros::ok)
    (let ((start-time (ros::time-now)))
      (ros::spin-once)
      (ros::sleep)
      (let ((elapsed-time (send (ros::time- (ros::time-now) start-time) :to-sec)))
        (if (> elapsed-time 0)
            (ros::ros-info "Update-robot-state frequency: ~A Hz" (/ 1.0 elapsed-time))
            (ros::ros-info "Update-robot-state frequency: N/A (elapsed time too small)"))))))

;; Start the loop
(ros::ros-info "Starting loop...")
(ik-viewer-loop-hz-debug)

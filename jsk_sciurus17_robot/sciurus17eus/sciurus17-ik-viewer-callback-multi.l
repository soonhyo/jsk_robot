#!/usr/bin/env roseus
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; Initialize Sciurus17
(unless (boundp '*sciurus17*) (sciurus17-init :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables
(defvar *r-arm-target-coords* (make-coords))  ;; Right arm target coordinates
(defvar *l-arm-target-coords* (make-coords))  ;; Left arm target coordinates
(defvar *head-target-coords* (make-coords))   ;; Head target coordinates
(defvar *torso-target-coords* (make-coords))  ;; Waist target coordinates
(defvar *r-hand-target* 0.0)                  ;; Right hand target value
(defvar *l-hand-target* 0.0)                  ;; Left hand target value
(defvar *ik-success-r* nil)                   ;; Right arm IK success flag
(defvar *ik-success-l* nil)                   ;; Left arm IK success flag

;; Callback functions
(defun r-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *r-arm-target-coords* pose)
    (ros::ros-info "Received right wrist target pose: ~A" pose)
    (setq *ik-success-r*
          (send *sciurus17* :inverse-kinematics
                *r-arm-target-coords*
                :link-list (send *sciurus17* :rarm :links)
                :move-target (send *sciurus17* :rarm :end-coords)
                :rotation-axis t
                :stop 2
                :debug-view nil
                :revert-if-fail nil
                :periodic-time 0.5
                :check-collision nil))
    (update-robot-state)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :rarm-controller 0.0 :min-time 0.3)
    )
  )

(defun l-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *l-arm-target-coords* pose)
    (ros::ros-info "Received left wrist target pose: ~A" pose)
    (setq *ik-success-l*
          (send *sciurus17* :inverse-kinematics
                *l-arm-target-coords*
                :link-list (send *sciurus17* :larm :links)
                :move-target (send *sciurus17* :larm :end-coords)
                :rotation-axis t
                :stop 2
                :debug-view nil
                :revert-if-fail nil
                :periodic-time 0.5
                :check-collision nil))
    (update-robot-state)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :larm-controller 0.0 :min-time 0.3)
    )
  )

(defun head-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *head-target-coords* pose)
    (ros::ros-info "Received head target pose: ~A" pose)
    (send *sciurus17* :head :y :joint-angle (rad2deg (elt (car (rpy-angle (send pose :rot))) 0)))
    (send *sciurus17* :head :p :joint-angle (- (rad2deg (elt (car (rpy-angle (send pose :rot))) 1))))
    (update-robot-state)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :head-controller 0.0 :min-time 0.3)
    )
  )

(defun torso-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *torso-target-coords* pose)
    (ros::ros-info "Received waist target pose: ~A" pose)
    (send *sciurus17* :torso :angle-vector (float-vector (rad2deg (elt (car (rpy-angle (send pose :rot))) 0))))
    (update-robot-state)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :torso-controller 0.0 :min-time 1.0)))

;; right hand callback
(defun r-hand-target-value-callback (msg)
  (setq *r-hand-target* (/ (- 1 (send msg :data)) 3))
  (ros::ros-info "Received right hand target: ~A" *r-hand-target*)
  (let ((mapped-value (* *r-hand-target* 10)))
    (ros::ros-info "Mapped right hand value: ~A" mapped-value)
    (send *sciurus17* :rhand :angle-vector (float-vector mapped-value)))
  (update-robot-state)
  (send *ri* :go-grasp :rarm :pos (/ (elt (send *sciurus17* :rhand :angle-vector) 0) 10) :effort 3 :wait nil))

;; left hand callback
(defun l-hand-target-value-callback (msg)
  (setq *l-hand-target* (/ (- 1 (send msg :data)) 3))
  (ros::ros-info "Received left hand target: ~A" *l-hand-target*)
  (let ((mapped-value (* *l-hand-target* 10)))
    (ros::ros-info "Mapped left hand value: ~A" (- mapped-value))
    (send *sciurus17* :lhand :angle-vector (float-vector (- mapped-value))))
  (update-robot-state)
  (send *ri* :go-grasp :larm :pos (/ (elt (send *sciurus17* :lhand :angle-vector) 0) 10) :effort 3 :wait nil))

;; predefined pose
(defun command-pose-callback (msg)
  (let ((command-pose-data (send msg :data)))
    (if (string= command-pose-data "init_pose")
        (progn
          (send *sciurus17* :reset-pose)
          (update-robot-state)
          (send *ri* :angle-vector (send *sciurus17* :angle-vector) 5000))
        )
    )
  )

;; Helper function to update robot state
(defun update-robot-state ()
  (send *irtviewer* :draw-objects)
  (x::window-main-one))

;; Object visualization
(objects (list *sciurus17*))

;; Initialize teleop pose
(send *sciurus17* :angle-vector (send *ri* :state :potentio-vector))
(send *sciurus17* :r_arm_joint4 :joint-angle 136)
(send *sciurus17* :l_arm_joint4 :joint-angle -136)
(send *ri* :angle-vector (send *sciurus17* :angle-vector))
(send *ri* :wait-interpolation)

;; Set initial coordinates
(setq *r-arm-target-coords* (send *sciurus17* :rarm :end-coords))
(setq *l-arm-target-coords* (send *sciurus17* :larm :end-coords))
(send *irtviewer* :draw-objects)

;; Subscribe to all topics
(ros::subscribe "/right_wrist_target_pose" geometry_msgs::PoseStamped #'r-arm-target-pose-callback)
(ros::subscribe "/left_wrist_target_pose" geometry_msgs::PoseStamped #'l-arm-target-pose-callback)
(ros::subscribe "/head_target_pose" geometry_msgs::PoseStamped #'head-target-pose-callback)
(ros::subscribe "/waist_target_pose" geometry_msgs::PoseStamped #'torso-target-pose-callback)
(ros::subscribe "/right_hand" std_msgs::Float32 #'r-hand-target-value-callback)
(ros::subscribe "/left_hand" std_msgs::Float32 #'l-hand-target-value-callback)
(ros::subscribe "/command_pose" std_msgs::String #'command-pose-callback)

;; Main loop
(defun ik-viewer-loop ()
  (ros::rate 100) ;; 100 Hz loop
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

;; Start the loop
(ros::ros-info "Starting IK viewer loop...")
(ik-viewer-loop)

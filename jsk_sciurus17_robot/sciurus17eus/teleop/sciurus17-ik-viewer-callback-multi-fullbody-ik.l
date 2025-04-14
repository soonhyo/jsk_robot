#!/usr/bin/env roseus
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; Initialize Sciurus17
(unless (boundp '*sciurus17*) (sciurus17-init :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables
(defvar *r-arm-target-coords* nil)  ;; Right arm target coordinates
(defvar *l-arm-target-coords* nil)  ;; Left arm target coordinates
(defvar *head-target-coords* nil)   ;; Head target coordinates
(defvar *torso-target-coords* nil)  ;; Waist target coordinates
(defvar *r-hand-target* 0.0)        ;; Right hand target value
(defvar *l-hand-target* 0.0)        ;; Left hand target value
(defvar *robot-state-updated* nil)  ;; State update flag

;; Callback functions (목표값만 설정)
(defun r-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *r-arm-target-coords* pose)
    (ros::ros-info "Received right wrist target pose: ~A" pose)
    (setq *robot-state-updated* t)))

(defun l-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *l-arm-target-coords* pose)
    (ros::ros-info "Received left wrist target pose: ~A" pose)
    (setq *robot-state-updated* t)))

(defun head-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *head-target-coords* pose)
    (ros::ros-info "Received head target pose: ~A" pose)
    (setq *robot-state-updated* t)))

(defun torso-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *torso-target-coords* pose)
    (ros::ros-info "Received waist target pose: ~A" pose)
    (setq *robot-state-updated* t)))

(defun r-hand-target-value-callback (msg)
  (setq *r-hand-target* (/ (- 1 (send msg :data)) 3))
  (ros::ros-info "Received right hand target: ~A" *r-hand-target*)
  (setq *robot-state-updated* t))

(defun l-hand-target-value-callback (msg)
  (setq *l-hand-target* (/ (- 1 (send msg :data)) 3))
  (ros::ros-info "Received left hand target: ~A" *l-hand-target*)
  (setq *robot-state-updated* t))

(defun command-pose-callback (msg)
  (let ((command-pose-data (send msg :data)))
    (if (string= command-pose-data "init_pose")
        (progn
          (send *sciurus17* :reset-pose)
          (send *ri* :angle-vector (send *sciurus17* :angle-vector) 5000)
          (send *ri* :wait-interpolation)
          (setq *robot-state-updated* t))
        nil)))

;; Helper function to update robot state with fullbody IK
(defun update-robot-state ()
  (if *robot-state-updated*
      (progn
        ;; Ensure valid target coordinates
        (let ((r-target (if *r-arm-target-coords* *r-arm-target-coords* (send *sciurus17* :rarm :end-coords :copy-worldcoords)))
              (l-target (if *l-arm-target-coords* *l-arm-target-coords* (send *sciurus17* :larm :end-coords :copy-worldcoords))))
          ;; Fullbody IK for both arms
          (let ((success (send *sciurus17* :inverse-kinematics
                               (list r-target l-target)  ;; Target coords for both arms
                               :move-target (list (send *sciurus17* :rarm :end-coords)
                                                  (send *sciurus17* :larm :end-coords))
                               :link-list (list (send *sciurus17* :rarm :links)
                                                (send *sciurus17* :larm :links))
                               :rotation-axis (list t t)
                               :stop 2
                               :debug-view nil
                               :revert-if-fail nil
                               :periodic-time 0.5
                               :check-collision nil)))
            (if success
                (progn
                  (send *ri* :angle-vector (send *sciurus17* :angle-vector) 50 :default-controller 0.0 :min-time 0.05 :spin-once nil)
                  (ros::ros-info "Fullbody IK succeeded"))
                (ros::ros-warn "Fullbody IK failed for targets: R:~A, L:~A" r-target l-target))))

        ;; Head
        (if *head-target-coords*
            (progn
              (send *sciurus17* :head :y :joint-angle (rad2deg (elt (car (rpy-angle (send *head-target-coords* :rot))) 0)))
              (send *sciurus17* :head :p :joint-angle (- (rad2deg (elt (car (rpy-angle (send *head-target-coords* :rot))) 1))))
              (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :head-controller 0.0 :min-time 0.05))
            nil)

        ;; Torso
        (if *torso-target-coords*
            (progn
              (send *sciurus17* :torso :angle-vector (float-vector (rad2deg (elt (car (rpy-angle (send *torso-target-coords* :rot))) 0))))
              (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :torso-controller 0.0 :min-time 0.05))
            nil)

        ;; Right hand
        (if *r-hand-target*
            (progn
              (let ((mapped-value (* *r-hand-target* 10)))
                (send *sciurus17* :rhand :angle-vector (float-vector mapped-value))
                (send *ri* :go-grasp :rarm :pos (/ (elt (send *sciurus17* :rhand :angle-vector) 0) 10) :effort 3 :wait nil)))
            nil)

        ;; Left hand
        (if *l-hand-target*
            (progn
              (let ((mapped-value (* *l-hand-target* 10)))
                (send *sciurus17* :lhand :angle-vector (float-vector (- mapped-value)))
                (send *ri* :go-grasp :larm :pos (/ (elt (send *sciurus17* :lhand :angle-vector) 0) 10) :effort 3 :wait nil)))
            nil)

        ;; Visualization
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (setq *robot-state-updated* nil))
      nil))

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
  (ros::rate 200)  ;; 200 Hz loop
  (while (ros::ok)
    (ros::spin-once)
    (update-robot-state)
    (ros::sleep)))

;; Start the loop
(ros::ros-info "Starting IK viewer loop with fullbody-inverse-kinematics...")
(ik-viewer-loop)

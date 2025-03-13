#!/usr/bin/env roseus
(require :sciurus17 "package://sciuruseus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")

;; Initialize Sciurus17
(unless (boundp '*sciurus*) (sciurus-init :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables
(defvar *target-coords* (make-coords)) ;; Target coordinates for the end-effector
(defvar *ik-success* nil)              ;; Flag to check IK success

;; Callback function for target pose subscription
(defun target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *target-coords* pose)
    (ros::ros-info "Received target pose: ~A" pose)))


(objects (list *sciurus*))

;; init teleop pose

(send *sciurus* :angle-vector (send *ri* :state :potentio-vector))
(send *sciurus* :r_arm_joint4 :joint-angle 136)
(send *sciurus* :l_arm_joint4 :joint-angle -136)

(send *ri* :angle-vector (send *sciurus* :angle-vector))
(send *ri* :wait-interpolation)

(setq *target-coords* (send *sciurus* :rarm :end-coords))

(send *irtviewer* :draw-objects)

;; Subscribe to target pose topic
(ros::subscribe "/right_wrist_target_pose" geometry_msgs::PoseStamped #'target-pose-callback)

;; Main loop to solve IK and update viewer
(defun ik-viewer-loop ()
    (ros::rate 100) ;; 10 Hz loop
    (while (ros::ok)
      (ros::spin-once)
      ;; Solve IK for right arm (r_link7)
      (setq *ik-success*
            (send *sciurus* :inverse-kinematics
                  *target-coords*
                  :link-list (send *sciurus* :rarm :links)
                  :move-target (send *sciurus* :rarm :end-coords)
                  :rotation-axis t
                  :stop 2
                  :debug-view nil
                  :revert-if-fail nil
                  :periodic-time 0.5
                  :check-collision nil))
        (if *ik-success*
            (ros::ros-info "IK solved successfully")
            (ros::ros-warn "IK failed"))
        ;; Update IRT viewer
        (send *irtviewer* :draw-objects)
        (send *ri* :angle-vector (send *sciurus* :angle-vector) 10 :rarm-controller) ;; Update robot state
        (x::window-main-one) ;; Process viewer events
        (ros::sleep))))

;; Start the loop
(ros::ros-info "Starting IK viewer loop...")
(ik-viewer-loop)

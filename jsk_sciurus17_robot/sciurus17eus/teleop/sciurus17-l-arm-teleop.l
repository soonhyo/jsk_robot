#!/usr/bin/env roseus
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; Initialize Sciurus17 (Left arm only)
(unless (boundp '*sciurus17*) (sciurus17-init :safe nil :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables for left arm
(defvar *l-arm-target-coords* (make-coords))  ;; Left arm target coordinates
(defvar *l-hand-target* 0.0)                  ;; Left hand target value
(defvar *ik-success-l* nil)                   ;; Left arm IK success flag

;; Callback for left arm
(defun l-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *l-arm-target-coords* pose)
    (ros::ros-info "Received left wrist target pose: ~A" pose)
    (setq *ik-success-l*
          (progn
            (format t "ik :larm time :~%")
            (bench
             (send *sciurus17* :larm :inverse-kinematics
                   *l-arm-target-coords*
                   :rotation-axis t
                   :stop 2
                   :debug-view nil
                   :revert-if-fail nil
                   :periodic-time 0.5
                   :check-collision nil)))
          )
    (progn
      (format t "angle vector time :~%")
      (bench
    ;(send *ri* :angle-vector-raw (send *sciurus17* :angle-vector) :fast :rarm-controller 0.0 :min-time 0.1)
      (send *ri* :angle-vector (send *sciurus17* :angle-vector) 50 :larm-controller 0.0 :min-time 0.05 :spin-once nil)
    ))
    ;; (progn
    ;;   (format t "irtviewer time :~%")
    ;;   (bench
    ;;    (update-robot-state)
    ;;    ))
    ))

;; Callback for left hand
(defun l-hand-target-value-callback (msg)
  (setq *l-hand-target* (/ (- 1 (send msg :data)) 3))
  (ros::ros-info "Received left hand target: ~A" *l-hand-target*)
  (let ((mapped-value (* *l-hand-target* 30)))
    (ros::ros-info "Mapped left hand value: ~A" (- mapped-value))
    (send *sciurus17* :lhand :angle-vector (float-vector (- mapped-value))))
  (update-robot-state)
  (send *ri* :go-grasp :larm :pos (/ (elt (send *sciurus17* :lhand :angle-vector) 0) 30) :effort 3 :wait nil))

;; Helper function
(defun update-robot-state ()
  (send *irtviewer* :draw-objects)
  (x::window-main-one))

;; Object visualization
(objects (list *sciurus17*))

;; Initialize teleop pose for left arm
(send *sciurus17* :angle-vector (send *ri* :actual-vector))
(send *sciurus17* :l_arm_joint4 :joint-angle -136)
(send *ri* :angle-vector (send *sciurus17* :angle-vector))
(send *ri* :wait-interpolation)
(setq *l-arm-target-coords* (send *sciurus17* :larm :end-coords))
(send *irtviewer* :draw-objects)

;; Subscribe to left arm topics
(ros::subscribe "/left_wrist_target_pose" geometry_msgs::PoseStamped #'l-arm-target-pose-callback)
;(ros::subscribe "/left_hand" std_msgs::Float32 #'l-hand-target-value-callback)

;; Main loop
(defun ik-viewer-loop ()
  (ros::rate 200) ;; 200 Hz
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

(defun ik-viewer-loop-hz-debug ()
  (ros::rate 100)  ;; 200 Hz loop
  (while (ros::ok)
    (let ((start-time (ros::time-now)))
      (ros::spin-once)
      (ros::sleep)
      (let ((elapsed-time (send (ros::time- (ros::time-now) start-time) :to-sec)))
        (if (> elapsed-time 0)
            (ros::ros-info "Update-robot-state frequency: ~A Hz" (/ 1.0 elapsed-time))
            (ros::ros-info "Update-robot-state frequency: N/A (elapsed time too small)"))))))

;; Start the loop
(ros::ros-info "Starting IK viewer loop...")
(ik-viewer-loop-hz-debug)

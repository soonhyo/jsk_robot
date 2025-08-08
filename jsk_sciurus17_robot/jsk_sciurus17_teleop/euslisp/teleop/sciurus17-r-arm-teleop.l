#!/usr/bin/env roseus
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; Initialize Sciurus17 (Right arm only)
(unless (boundp '*sciurus17*) (sciurus17-init :safe nil :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; Global variables for right arm
(defvar *r-arm-target-coords* (make-coords))  ;; Right arm target coordinates
(defvar *r-hand-target* 0.0)                  ;; Right hand target value
(defvar *ik-success-r* nil)                   ;; Right arm IK success flag

;; Callback for right arm
(defun r-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *r-arm-target-coords* pose)
    (ros::ros-info "Received right wrist target pose: ~A" pose)
    (setq *ik-success-r*
          (progn
            (format t "ik :rarm time :~%")
            (bench
             (send *sciurus17* :rarm :inverse-kinematics
                   *r-arm-target-coords*
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
       (send *ri* :angle-vector (send *sciurus17* :angle-vector) 50 :rarm-controller 0.0 :min-time 0.05 :spin-once nil)
    ))
    ;; (progn
    ;;   (format t "irtviewer time :~%")
    ;;   (bench
    ;;    (update-robot-state)
    ;;    ))
    ))

;; Callback for right hand
(defun r-hand-target-value-callback (msg)
  (setq *r-hand-target* (/ (- 1 (send msg :data)) 1.25))
  (ros::ros-info "Received right hand target: ~A" *r-hand-target*)
  (let ((mapped-value (* *r-hand-target* 10)))
    (ros::ros-info "Mapped right hand value: ~A" mapped-value)
    (send *sciurus17* :rhand :angle-vector (float-vector mapped-value)))
  ;; (update-robot-state)
  (send *ri* :go-grasp :rarm :pos (/ (elt (send *sciurus17* :rhand :angle-vector) 0) 10) :effort 3 :wait nil))

;; init pose
(defun command-pose-callback (msg)
  (let ((command-pose-data (send msg :data)))
    (cond
     ((string= command-pose-data "pose/init_pose")
      (progn
	(ros::unsubscribe "/right_wrist_target_pose")
	(unix:sleep 6)
	(send *sciurus17* :reset-pose)
	(setq *r-arm-target-coords* (send *sciurus17* :rarm :end-coords))
	(ros::subscribe "/right_wrist_target_pose" geometry_msgs::PoseStamped #'r-arm-target-pose-callback)
	;; (update-robot-state)
	)))))

;; Helper function
(defun update-robot-state ()
  (send *irtviewer* :draw-objects)
  (x::window-main-one))

;; Object visualization
(objects (list *sciurus17*))

;; Initialize teleop pose for right arm
(send *sciurus17* :angle-vector (send *ri* :actual-vector))
(setq *r-arm-target-coords* (send *sciurus17* :rarm :end-coords))
(send *irtviewer* :draw-objects)

;; (send-all (send *sciurus17* :links) :analysis-level :coords)
;; (send-all (send *ri* :robot :links) :analysis-level :coords)
;; Subscribe to right arm topics
(ros::subscribe "/right_wrist_target_pose" geometry_msgs::PoseStamped #'r-arm-target-pose-callback)
(ros::subscribe "/right_hand" std_msgs::Float32 #'r-hand-target-value-callback)
(ros::subscribe "/vr_command" std_msgs::String #'command-pose-callback)

;; (send *ri* :update-robot-state-alone)
;; (send *ri* :update-robot-state)

;; Main loop
(defun ik-viewer-loop ()
  (ros::rate 200) ;; 200 Hz
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

(defun ik-viewer-loop-hz-debug ()
  (ros::rate 50)  ;; 50 Hz loop
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

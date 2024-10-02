#!/usr/bin/env roseus

(require "package://pr2eus/robot-interface.l")
(require "package://pr2eus_moveit/euslisp/robot-moveit.l")
(load "package://mycoboteus/mycobot-utils.l")
(ros::roseus-add-srvs "std_srvs")

(defclass mycobot-moveit-environment
  :super moveit-environment)

(defmethod mycobot-moveit-environment
  (:init (&key args)
         (send-super* :init :robot (instance mycobot-robot :init) :frame-id "link1" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "arm_group")
               (cons :target-link
                     (send self :search-link-from-name "link7"))
               (cons :joint-list (send robot :rarm :joint-list)))))
  )


(defclass mycobot-interface
  :super robot-interface
  :slots ()
  )


(defmethod mycobot-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot mycobot-robot :joint-states-topic "joint_states" :groupname "mycobot_interface" args)
      (send self :add-controller :rarm-controller)
      (setq gripper-action
            (instance ros::simple-action-client :init
                      "/gripper_controller/gripper_command"
                      control_msgs::GripperCommandAction
                      :groupname groupname))
      (send self :set-moveit-environment (instance mycobot-moveit-environment :init))
    ))
  (:default-controller ()
    (append
      (send self :rarm-controller)))
  (:rarm-controller ()
    (list
      (list
        (cons :controller-action "/arm_controller/follow_joint_trajectory")
        (cons :controller-state "/arm_controller/follow_joint_trajectory/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names
              (list "joint1" "joint2" "joint3" "joint4" "joint5" "joint6")))))

;;   (:angle-vector-raw (av &optional (tm :fast) (ctype controller-type) (start-time 0)
;;                          &key (scale 2.2) (min-time 0.05) (end-coords-interpolation nil)
;;                          &allow-other-keys)
;;     (send self :angle-vector-sequence-raw (list av) (list tm) ctype start-time :scale scale :min-time min-time
;;           :end-coords-interpolation end-coords-interpolation))
;;   (:angle-vector-sequence-raw (avs &optional (tms :fast) (ctype controller-type) (start-time 0)
;;                                    &key (scale 2.2) (min-time 0.05) (end-coords-interpolation nil)
;;                                    &allow-other-keys)
;;     ;; force add current position to the top of avs
;;     (if (atom tms) (setq tms (list tms)))
;;     (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
;;     (send-super :angle-vector-sequence avs tms ctype start-time :scale scale :min-time min-time
;;                 :end-coords-interpolation end-coords-interpolation))
;;   (:angle-vector
;;    (av &optional tm (ctype controller-type) (start-time 0) &rest args
;;        &key (move-arm :arms) start-offset-time (clear-velocities t) &allow-other-keys)
;;    "Send joind angle to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
;; - av : joint angle vector [rad]
;; - tm : time to goal in [msec]
;; "
;;    (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
;;    ;; for simulation mode
;;    (when (send self :simulation-modep)
;;      (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
;;    (if (and (get self :moveit-environment)
;;             (send (get self :moveit-environment) :robot))
;;      (progn
;;        (unless tm (setq tm 3000))
;;        (send-super* :angle-vector-motion-plan av :ctype ctype :move-arm move-arm :total-time tm
;;                     :start-offset-time (if start-offset-time start-offset-time start-time)
;;                     :clear-velocities clear-velocities :use-torso nil args))
;;      (progn
;;        (warning-message 3 "moveit environment is not correctly set, execute :angle-vector-raw instead~%")
;;        (unless tm (setq tm :fast))
;;        (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))))
;;   (:angle-vector-sequence
;;    (avs &optional tms (ctype controller-type) (start-time 0) &rest args
;;         &key (move-arm :rarm) start-offset-time (clear-velocities t) &allow-other-keys)
;;    "Send joind angle sequence to robot with self-collision motion planning, this method returns immediately, so use :wait-interpolation to block until the motion stops.
;; - avs : sequence of joint angle vector [rad]
;; - tms : list of time to goal from previous angle-vector point in [msec]
;; "
;;    (setq ctype (or ctype controller-type))  ;; use default if ctype is nil
;;    ;; for simulation mode
;;    (when (send self :simulation-modep)
;;      (return-from :angle-vector-sequence
;;                   (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))
;;    (if (and (get self :moveit-environment)
;;             (send (get self :moveit-environment) :robot))
;;      (progn
;;        (setq tms
;;              (if tms
;;                (if (and (listp tms) (every #'numberp tms)) (apply #'+ tms) tms)
;;                3000))
;;        (send-super* :angle-vector-motion-plan avs :ctype ctype :move-arm move-arm :total-time tms
;;                     :start-offset-time (if start-offset-time start-offset-time start-time)
;;                     :clear-velocities clear-velocities :use-torso nil args))
;;      (progn
;;        (warning-message 3 "moveit environment is not correctly set, execute :angle-vector-sequence-raw instead~%")
;;        (unless tms (setq tms :fast))
;;        (return-from :angle-vector-sequence
;;                     (send* self :angle-vector-sequence-raw avs tms ctype start-time args)))))

  (:go-grasp (&key (pos 0) (wait t))
    (when (send self :simulation-modep)
      ;; TODO: add mimic joint
      (return-from :go-grasp t))
    ;; we believe the low-level algorithm of mycobot can perform graspping with proper effort.
    ;; so we do not need to assign a specific posiiton for gripper.
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 1 args)) ;; 1 denote to grasp
  (:stop-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 0 args)) ;; 0 denote to grasp
  (:servo-on
   "Servo On all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data t)))
  (:servo-off
   "Servo Off all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data nil)))
  )


(defun mycobot-init ()
  (setq *mycobot* (instance mycobot-robot :init))
  (setq *robot* *mycobot*)
  (setq *ri* (instance mycobot-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*)))


;; send *mycobot* :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil) :move-target (send *mycobot* :rarm :end-coords)
;; send *mycobot* :rarm :inverse-kinematics (make-coords :pos #f(240 0 100)) :rotation-axis nil)

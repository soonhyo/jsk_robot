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
               (cons :joint-list (send robot :rarm :joint-list))))))

(defclass mycobot-interface
  :super robot-interface
  :slots (gripper-sequence-id gripper-type moveit-robot))

(defmethod mycobot-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot mycobot-robot :joint-states-topic "joint_states" :groupname "mycobot_interface" args)
      (send self :add-controller :rarm-controller)
      (setq gripper-sequence-id 0)
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

  (:start-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 1 args))

  (:stop-grasp
   (&rest args &key &allow-other-keys)
   (send* self :go-grasp :pos 0 args))

  (:go-grasp (&key (pos 0) (wait t))
    (when (send self :simulation-modep)
      (return-from :go-grasp t))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))

  (:angle-vector
   (av &optional tm (ctype controller-type) (start-time 0) &rest args
       &key (move-arm :rarm) start-offset-time (clear-velocities t) &allow-other-keys)
   (setq ctype (or ctype controller-type))
   (when (send self :simulation-modep)
     (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))
   (if (and (get self :moveit-environment)
            (send (get self :moveit-environment) :robot))
       (progn
         (unless tm (setq tm 3000))
         (send-super* :angle-vector-motion-plan av :ctype ctype :move-arm move-arm :total-time tm
                     :start-offset-time (if start-offset-time start-offset-time start-time)
                     :clear-velocities clear-velocities :use-torso nil args))
       (progn
         (warning-message 3 "moveit environment is not correctly set, execute :angle-vector-raw instead~%")
         (unless tm (setq tm :fast))
         (return-from :angle-vector (send* self :angle-vector-raw av tm ctype start-time args)))))

  (:servo-on
   "Servo On all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data t)))

  (:servo-off
   "Servo Off all joints."
   (ros::service-call
    "/set_servo" (instance std_srvs::SetBoolRequest :init :data nil)))
  )


(defun mycobot-init (&key (type :default-controller) (moveit t))
  (let (mvit-env mvit-rb)
    (when moveit
      (setq mvit-env (instance mycobot-moveit-environment))
      (setq mvit-rb (instance mycobot-robot :init)))
    (if (not (boundp '*ri*))
        (setq *ri* (instance mycobot-interface :init :type type
                            :moveit-environment mvit-env
                            :moveit-robot mvit-rb)))
    (if (not (boundp '*mycobot*))
        (setq *mycobot* (instance mycobot-robot :init)))))

(provide :mycobot-interface)

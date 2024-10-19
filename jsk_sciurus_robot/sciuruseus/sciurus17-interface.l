(require "package://pr2eus_moveit/euslisp/robot-moveit.l")
(require :sciurus17 "package://sciuruseus/sciurus17-utils.l")
(ros::load-ros-package "control_msgs")

(defclass sciurus17-interface
  :super robot-interface
  :slots (gripper-sequence-id right-gripper-action left-gripper-action
          right-gripper-type left-gripper-type moveit-robot hard-coded-pose-table))

(defmethod sciurus17-interface
  (:init (&rest args &key ((:moveit-environment mvit-env) (instance sciurus17-moveit-environment))
                ((:moveit-robot mvit-rb) (instance sciurus17-robot :init))
                ((:robot robot) sciurus17-robot)
                &allow-other-keys)
   (prog1 (send-super* :init :robot robot :joint-states-topic "/joint_states" :groupname "sciurus17_interface" args)
     (send self :add-controller :larm-controller)
     (send self :add-controller :rarm-controller)
     (send self :add-controller :head-controller)
     (send self :add-controller :torso-controller)

     (setq right-gripper-action (instance ros::simple-action-client :init
                                          "/sciurus17/controller1/right_hand_controller/gripper_cmd"
                                          control_msgs::GripperCommandAction
                                          :groupname groupname))
     (setq left-gripper-action (instance ros::simple-action-client :init
                                         "/sciurus17/controller2/left_hand_controller/gripper_cmd"
                                         control_msgs::GripperCommandAction
                                         :groupname groupname))

     (dolist (action (list right-gripper-action left-gripper-action))
       (unless (and joint-action-enable (send action :wait-for-server 3))
         (ros::ros-warn "~A is not respond" action)
         (ros::ros-info "*** if you do not have gripper, you can ignore this message ***")
         (return)))

     (setq gripper-sequence-id 0)
     (ros::spin-once)
     (if mvit-rb (setq moveit-robot mvit-rb))
     (if mvit-env (send self :set-moveit-environment (send mvit-env :init :robot moveit-robot)))
     (if *enable-roseus-resume*
       (progn
         (warning-message 3 "Installing interruption handler...~%")
         (roseus_resume::install-interruption-handler self
            right-gripper-action
            left-gripper-action)
         (roseus_resume::install-default-intervention self)
         (warning-message 3 "...done.~%")))
     ))

  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :head-controller)
    (send self :torso-controller)
    ))

  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/sciurus17/controller2/left_arm_controller/follow_joint_trajectory")
     (cons :controller-state "/sciurus17/controller2/left_arm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "l_arm_joint1" "l_arm_joint2" "l_arm_joint3" "l_arm_joint4" "l_arm_joint5" "l_arm_joint6" "l_arm_joint7")))))

  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/sciurus17/controller1/right_arm_controller/follow_joint_trajectory")
     (cons :controller-state "/sciurus17/controller1/right_arm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "r_arm_joint1" "r_arm_joint2" "r_arm_joint3" "r_arm_joint4" "r_arm_joint5" "r_arm_joint6" "r_arm_joint7")))))

  (:head-controller ()
   (list
    (list
     (cons :controller-action "/sciurus17/controller3/neck_controller/follow_joint_trajectory")
     (cons :controller-state "/sciurus17/controller3/neck_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "neck_yaw_joint" "neck_pitch_joint")))))

  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/sciurus17/controller3/waist_yaw_controller/follow_joint_trajectory")
     (cons :controller-state "/sciurus17/controller3/waist_yaw_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "waist_yaw_joint")))))


  (:start-grasp
   (&optional (arm :arms) &key (effort 50) (wait t))
   (send self :go-grasp arm :pos 0 :effort effort :wait wait))

  (:stop-grasp
   (&optional (arm :arms) &key (effort 50) (wait t))
   (send self :go-grasp arm :pos 100 :effort effort :wait wait))

  (:go-grasp
   (&optional (arm :arms) &key (pos 0) (effort 50) (wait t))
   (let (goal
         (clients (case arm
                    (:larm (list left-gripper-action))
                    (:rarm (list right-gripper-action))
                    (:arms (list right-gripper-action left-gripper-action))
                    (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
         (results (list)))
     (dolist (client clients)
       (ros::ros-info "send-goal to ~A" client)
       (setq goal  (instance control_msgs::GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal))

     (if wait (send-all clients :wait-for-result))
     (dolist (client clients)
       (push (send client :get-result) results))
     results))

  (:ros-state-callback
   (msg)
   (send-super :ros-state-callback msg))
  ) ;; defclass sciurus17-interface

(defclass sciurus17-moveit-environment
  :super moveit-environment)

(defmethod sciurus17-moveit-environment
  (:init (&key ((:robot rb) *sciurus*) &rest args)
         (send-super* :init :robot rb :frame-id "base_link" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "r_arm_group")
               (cons :target-link
                     (send self :search-link-from-name "r_link7"))
               (cons :joint-list (send robot :rarm :joint-list)))
         (list :larm
               (cons :group-name "l_arm_group")
               (cons :target-link
                     (send self :search-link-from-name "l_link7"))
               (cons :joint-list (send robot :larm :joint-list)))
         (list :arms
               (cons :group-name "two_arm_group")
               (cons :target-link
                     (list
                       (send self :search-link-from-name "l_link7")
                       (send self :search-link-from-name "r_link7")))
               (cons :joint-list (append (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list))))
         (list :wrarm
               (cons :group-name "r_arm_waist_group")
               (cons :target-link
                     (send self :search-link-from-name "r_link7"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :rarm :joint-list))))
         (list :wlarm
               (cons :group-name "l_arm_waist_group")
               (cons :target-link
                     (send self :search-link-from-name "l_link7"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :larm :joint-list))))
         )))

(defun sciurus-init (&key (type :default-controller) (moveit t))
  (let (mvit-env mvit-rb)
    (when moveit
      (setq mvit-env (instance sciurus17-moveit-environment))
      (setq mvit-rb (instance sciurus17-robot :init)))
    (if (not (boundp '*ri*))
        (setq *ri* (instance sciurus17-interface :init :type type
                             :moveit-environment mvit-env
                             :moveit-robot mvit-rb)))
    (if (not (boundp '*sciurus*)) ;; changed the robot name for convenience
        (setq *sciurus* (instance sciurus17-robot :init)))))

(provide :sciurus17-interface)

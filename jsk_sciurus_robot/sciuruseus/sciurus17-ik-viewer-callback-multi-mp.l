#!/usr/bin/env roseus
(require :sciurus17 "package://sciuruseus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")

;; 스레드 초기화
(sys:make-thread 8) ;; 최대 8개 스레드 풀 생성

(unless (boundp '*sciurus*) (sciurus-init :moveit nil))
(unless (boundp '*irtviewer*) (make-irtviewer))

;; 전역 변수
(defvar *r-arm-target-coords* (make-coords))
(defvar *l-arm-target-coords* (make-coords))
(defvar *head-target-coords* (make-coords))
(defvar *waist-target-coords* (make-coords))
(defvar *r-hand-target* 0.0)
(defvar *l-hand-target* 0.0)
(defvar *ik-success-r* nil)
(defvar *ik-success-l* nil)
(defvar *last-angle-vector* nil)
(defvar *draw-counter* 0)
(defvar *state-mutex* (sys:make-mutex-lock))
(defvar *task-queue* nil) ;; 작업 큐
(defvar *queue-mutex* (sys:make-mutex-lock))

;; 좌표 비교 유틸리티
(defun coords-nearly-equal (c1 c2 threshold)
  (let ((diff (v- (send c1 :worldpos) (send c2 :worldpos))))
    (< (norm diff) threshold)))

;; 작업 큐에 추가하는 함수
(defun add-task-to-queue (task)
  (sys:mutex *queue-mutex*
    (push task *task-queue*)))

;; 콜백 함수들 (작업 큐에 추가만 함)
(defun r-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (unless (coords-nearly-equal *r-arm-target-coords* pose 0.01)
      (setq *r-arm-target-coords* pose)
      (add-task-to-queue
       #'(lambda ()
           (setq *ik-success-r*
                 (send *sciurus* :inverse-kinematics
                       *r-arm-target-coords*
                       :link-list (send *sciurus* :rarm :links)
                       :move-target (send *sciurus* :rarm :end-coords)
                       :rotation-axis t
                       :stop 1
                       :periodic-time 0.5
                       :debug-view nil
                       :revert-if-fail nil
                       :check-collision nil))
           (update-robot-state))))))

(defun l-arm-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (unless (coords-nearly-equal *l-arm-target-coords* pose 0.01)
      (setq *l-arm-target-coords* pose)
      (add-task-to-queue
       #'(lambda ()
           (setq *ik-success-l*
                 (send *sciurus* :inverse-kinematics
                       *l-arm-target-coords*
                       :link-list (send *sciurus* :larm :links)
                       :move-target (send *sciurus* :larm :end-coords)
                       :rotation-axis t
                       :stop 1
                       :periodic-time 0.5
                       :debug-view nil
                       :revert-if-fail nil
                       :check-collision nil))
           (update-robot-state))))))

(defun neck-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *head-target-coords* pose)
    (add-task-to-queue
     #'(lambda ()
         (send *sciurus* :head :y :joint-angle (rad2deg (elt (car (rpy-angle (send pose :rot))) 0)))
         (send *sciurus* :head :p :joint-angle (- (rad2deg (elt (car (rpy-angle (send pose :rot))) 1))))
         (update-robot-state)))))

(defun waist-yaw-target-pose-callback (msg)
  (let ((pose (ros::tf-pose->coords (send msg :pose))))
    (setq *waist-target-coords* pose)
    (add-task-to-queue
     #'(lambda ()
         (send *sciurus* :torso :angle-vector (float-vector (rad2deg (elt (car (rpy-angle (send pose :rot))) 0))))
         (update-robot-state)))))

(defun r-hand-target-value-callback (msg)
  (setq *r-hand-target* (- 1 (send msg :data)))
  (add-task-to-queue
   #'(lambda ()
       (let ((mapped-value (* *r-hand-target* 50)))
         (send *sciurus* :rhand :angle-vector (float-vector mapped-value)))
       (update-robot-state))))

(defun l-hand-target-value-callback (msg)
  (setq *l-hand-target* (- 1 (send msg :data)))
  (add-task-to-queue
   #'(lambda ()
       (let ((mapped-value (* *l-hand-target* 50)))
         (send *sciurus* :lhand :angle-vector (float-vector (- mapped-value))))
       (update-robot-state))))

;; 상태 업데이트 함수
(defun update-robot-state ()
  (sys:mutex *state-mutex*
    (let ((current-av (send *sciurus* :angle-vector)))
      (unless (equal current-av *last-angle-vector*)
        (send *ri* :angle-vector-raw current-av :fast :default-controller 0.0 :min-time 0.5)
        (send *ri* :go-grasp :larm :pos (/ (elt (send *sciurus* :lhand :angle-vector) 0) 50) :effort 1 :wait nil)
        (send *ri* :go-grasp :rarm :pos (/ (elt (send *sciurus* :rhand :angle-vector) 0) 50) :effort 1 :wait nil)
        (setq *last-angle-vector* current-av)))
    (incf *draw-counter*)
    (when (zerop (mod *draw-counter* 10))
      (send *irtviewer* :draw-objects))
    (x::window-main-one)))

;; 워커 스레드 함수
(defun worker-thread ()
  (while t
    (let (task)
      (sys:mutex *queue-mutex*
        (when *task-queue*
          (setq task (pop *task-queue*))))
      (when task
        (funcall task)))
    (unix:usleep 10000))) ;; 10ms 대기

;; 초기화 및 워커 스레드 시작
(objects (list *sciurus*))
(send *sciurus* :angle-vector (send *ri* :state :potentio-vector))
(send *sciurus* :r_arm_joint4 :joint-angle 136)
(send *sciurus* :l_arm_joint4 :joint-angle -136)
(send *ri* :angle-vector (send *sciurus* :angle-vector))
(send *ri* :wait-interpolation)
(setq *r-arm-target-coords* (send *sciurus* :rarm :end-coords))
(setq *l-arm-target-coords* (send *sciurus* :larm :end-coords))
(send *irtviewer* :draw-objects)

;; 워커 스레드 실행 (최대 6개 부위에 맞춰 6개 스레드)
(dotimes (i 6)
  (sys:thread #'worker-thread))

;; ROS 구독자 설정
(ros::subscribe "/right_wrist_target_pose" geometry_msgs::PoseStamped #'r-arm-target-pose-callback :queue-size 1)
(ros::subscribe "/left_wrist_target_pose" geometry_msgs::PoseStamped #'l-arm-target-pose-callback :queue-size 1)
(ros::subscribe "/head_target_pose" geometry_msgs::PoseStamped #'neck-target-pose-callback :queue-size 1)
(ros::subscribe "/waist_target_pose" geometry_msgs::PoseStamped #'waist-yaw-target-pose-callback :queue-size 1)
(ros::subscribe "/right_hand" std_msgs::Float32 #'r-hand-target-value-callback :queue-size 1)
(ros::subscribe "/left_hand" std_msgs::Float32 #'l-hand-target-value-callback :queue-size 1)

;; 메인 루프
(defun ik-viewer-loop ()
  (ros::rate 50)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

;; 실행
(ros::ros-info "Starting IK viewer loop with task queue and worker threads...")
(ik-viewer-loop)

;; ROS 및 Sciurus17 패키지 로드
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")

;; ROS 노드 초기화
(ros::roseus "jacobian_force_estimator")

;; Sciurus17 로봇 객체 및 인터페이스 생성 및 초기화 확인
(unless (sciurus17-init)
  (error "Failed to initialize Sciurus17"))
(defvar *sciurus17* *sciurus17*)  ;; 로봇 모델
(defvar *ri* *ri*)                ;; 로봇 인터페이스

;; 기준 프레임: r_link7
(setq *end-coords* (send *sciurus17* :link "r_link7"))
(unless *end-coords* (error "Failed to set end-coords"))

;; 오른팔 링크 리스트
(setq *rarm-link-list* (send *sciurus17* :rarm :links))
(unless *rarm-link-list* (error "Failed to set rarm-link-list"))

;; 전류 → 토크 변환 계수
(setq *Kt* '(-3.60 -3.60 -3.60 -3.60 -1.75 -1.75 -1.75))  ;; [Nm/A], 7개 관절

;; 관절 전류 저장 변수
(setq *tau* (make-list 7 :initial-element 0.0))

;; Jacobian 계산 함수
(defun calc-jacobian ()
  (send *sciurus17* :calc-jacobian-from-link-list
        *rarm-link-list*
        :move-target *end-coords*
        :rotation-axis t
        :translation-axis t))

;; JointState 콜백 함수
(defun joint-state-callback (msg)
  (let ((efforts (send msg :effort)))
    (if (and efforts (>= (length efforts) 7))
        (progn
          (dotimes (i 7)
            (setf (nth i *tau*) (float (aref efforts i))))
          (format t "Right arm efforts: ~A~%" *tau*)
          (estimate-force)))))

;; JointState 토픽 구독
(ros::subscribe "/sciurus17/controller1/joint_states"
                sensor_msgs::JointState
                #'joint-state-callback)

;; 힘 추정 및 퍼블리시 함수
(defun estimate-force ()
  ;; 실시간으로 로봇 모델의 관절 각도 갱신
  (send *ri* :update-robot-state)
  (send *sciurus17* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((J (calc-jacobian));; 6x7 야코비안
         (ttau (matrix (coerce *tau* float-vector)))
         (F (transpose (m* (send *sciurus17* :calc-inverse-jacobian (transpose J)) (transpose ttau)))))

    (unless J (error "Jacobian calculation failed"))
    (format t "Angle vector: ~A~%" (send *sciurus17* :angle-vector))
    (format t "J dims: ~A~%" (array-dimensions J))
    (format t "tau: ~A~%" *tau*)
    (format t "F dims: ~A~%" (array-dimensions F))
    (let ((F-vec (scale -1 (array-entity F))))
      (format t "F-vec: ~A~%" F-vec)
      (format t "Magnitude F-vec: ~A~%" (norm F-vec))
      (let ((msg (instance geometry_msgs::WrenchStamped :init)))
        (send msg :header :stamp (ros::time-now))
        (send msg :header :frame_id "r_link7")
        (let ((force-vec (subseq F-vec 0 3))
              (torque-vec (subseq F-vec 3 6)))
          (send msg :wrench :force :x (float (aref force-vec 0)))
          (send msg :wrench :force :y (float (aref force-vec 1)))
          (send msg :wrench :force :z (float (aref force-vec 2)))
          (send msg :wrench :torque :x (float (aref torque-vec 0)))
          (send msg :wrench :torque :y (float (aref torque-vec 1)))
          (send msg :wrench :torque :z (float (aref torque-vec 2))))
        (ros::publish "/force_estimation" msg)))))

;; ROS 퍼블리셔 생성
(ros::advertise "/force_estimation" geometry_msgs::WrenchStamped 10)

;; 노드 실행
(ros::rate 100)  ;; 100 Hz
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

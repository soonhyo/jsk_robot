;; ROS 및 Sciurus17 패키지 로드
(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "sensor_msgs")

;; ROS 노드 초기화
(ros::roseus "gravity_torque_calculator")

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

;; 중력 가속도 벡터 (m/s^2, z축 방향)
(setq *gravity* (float-vector 0 0 -9.8))

;; 중력 토크 계산 함수
(defun calc-gravity-torque ()
  ;; 실시간 관절 각도 갱신
  (send *sciurus17* :angle-vector (send *ri* :state :potentio-vector))
  (let ((tau (make-array 7 :element-type 'float :initial-element 0.0))
        (force-acc (make-array 7 :element-type 'float-vector :initial-element #f(0 0 0))))
    ;; 링크를 역순으로 순회 (엔드 이펙터에서 베이스 방향)
    (let ((links (reverse *rarm-link-list*)))
      (dotimes (i (length links))
        (let* ((link (nth i links))
               (mass (send link :weight))  ;; 질량 (kg 단위로 가정)
               (com (scale 0.001 (send link :centroid)))  ;; 질량 중심 (mm -> m)
               (com-world (send link :transform-vector com))  ;; 월드 좌표계 위치
               (force (scale mass *gravity*))  ;; 중력에 의한 힘
               (joint-index (- 6 i)))  ;; 관절 인덱스 (0~6)
          ;; 누적 힘 계산
          (when (< joint-index 7)
            (setf (aref force-acc joint-index) (v+ force (aref force-acc joint-index))))
          ;; 각 관절에 대해 토크 계산
          (when (>= joint-index 0)
            (let* ((parent-link (if (> i 0) (nth (1- i) links) nil))
                   (joint-pos (if parent-link
                                  (send parent-link :worldpos)  ;; 상위 링크 위치
                                  (send link :worldpos)))  ;; 첫 링크는 자체 위치
                   (r (v- com-world joint-pos))  ;; 관절에서 질량 중심까지 벡터
                   (torque (v* r (aref force-acc joint-index))))  ;; 토크 = r x F
              ;; z축 방향 토크만 사용 (Sciurus17 관절이 z축 회전 가정)
              (setf (aref tau joint-index) (aref torque 2)))))))
    tau))

;; JointState 콜백 함수
(defun joint-state-callback (msg)
  (let ((tau-gravity (calc-gravity-torque)))
    (format t "Angle vector: ~A~%" (send *sciurus17* :angle-vector))
    (format t "Gravity torque: ~A~%" tau-gravity)))

;; JointState 토픽 구독
(ros::subscribe "/sciurus17/controller1/joint_states"
                sensor_msgs::JointState
                #'joint-state-callback)

;; 노드 실행
(ros::rate 100)  ;; 100 Hz
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

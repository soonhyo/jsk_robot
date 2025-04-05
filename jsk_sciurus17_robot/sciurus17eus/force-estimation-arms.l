(require :sciurus17 "package://sciurus17eus/sciurus17-interface.l")
(ros::load-ros-package "geometry_msgs")

(ros::roseus "force_estimator_with_gravity_compensation")

(sciurus17-init)
(unless (boundp '*sciurus17*) (error "Robot *sciurus17* is not initialized"))
(unless (boundp '*ri*) (error "Robot interface *ri* is not initialized"))

;; current → torque reconfigured coefficients
(setq *Kt* '(1.79 1.79 1.79 1.79 1.79 2.70 2.10 2.60 2.70 1.79 1.79 1.79 2.70 2.10 2.50 2.70 1.79 1.79 1.79))  ;; [Nm/A], all of joints
(setq *Kt-re* (mapcar #'(lambda (x) (/ x 1.79)) *Kt*))

;; Force Estimator class
(defclass force-estimator
  :super propertied-object
  :slots (rarm-end-coords larm-end-coords rarm-link-list larm-link-list rarm-link-index-list larm-link-index-list
          joint-list-all rarm-topic-name larm-topic-name robot robot-interface debug)
  :documentation "Class for estimating external force on both arms")

(defmethod force-estimator
  (:init (&key (robot *sciurus17*) (interface *ri*) (debug nil))
    "Initialize the force estimator for both arms"
    (send self :set-val 'robot robot)
    (send self :set-val 'robot-interface interface)
    (send self :set-val 'debug debug)
    ;; Right arm 설정
    (send self :set-val 'rarm-end-coords (send robot :link "r_link7"))
    (send self :set-val 'rarm-link-list (send robot :rarm :links))
    (send self :set-val 'rarm-link-index-list
          (mapcar #'(lambda (joint-name) (position joint-name (send-all (send robot :joint-list) :name)))
                  (send-all (send robot :rarm :joint-list) :name)))
    (send self :set-val 'rarm-topic-name "/rarm_force_estimation")
    ;; Left arm 설정
    (send self :set-val 'larm-end-coords (send robot :link "l_link7"))
    (send self :set-val 'larm-link-list (send robot :larm :links))
    (send self :set-val 'larm-link-index-list
          (mapcar #'(lambda (joint-name) (position joint-name (send-all (send robot :joint-list) :name)))
                  (send-all (send robot :larm :joint-list) :name)))
    (send self :set-val 'larm-topic-name "/larm_force_estimation")
    ;; 전체 joint list
    (send self :set-val 'joint-list-all (send robot :joint-list))
    ;; ROS 퍼블리셔 초기화
    (ros::advertise rarm-topic-name geometry_msgs::WrenchStamped 10)
    (ros::advertise larm-topic-name geometry_msgs::WrenchStamped 10)
    self)

  (:calc-gravity-torque ()
    "Calculate gravity-induced torque for entire robot"
    (if debug (format t "Calculating Gravity Torque:~%"))
    (send robot-interface :update-robot-state)
    (send robot :angle-vector (send robot-interface :potentio-vector))
    (let* ((torque-vector (send robot :calc-torque
                                :calc-statics-p t
                                :debug-view nil
                                :dt 0.005)))
      (if debug
          (format t "Final Gravity Torque Vector: ~A Nm~%" torque-vector))
      torque-vector))

  (:get-real-torque ()
    "Get real torque with reconfiguration for entire robot"
    (let (tau tau-re)
      (setq tau (send robot-interface :torque-vector))
      (setq tau-re (map float-vector #'* (coerce tau cons) *Kt-re*))
      (if debug
          (format t "Real Torque vector reconfigured: ~A~%" tau-re))
      tau-re))

  (:calc-external-force (ext-tau limb link-index-list)
    "Calculate external force from external torque for specified limb"
    (if debug
        (format t "Overall ext-tau for ~A: ~A~%" limb
                (mapcar #'(lambda (x) (aref ext-tau x)) link-index-list)))
    (let ((FM-vec (send robot :calc-force-from-joint-torque limb ext-tau)))
      (if (eq FM-vec nil)
          (return-from :calc-external-force nil)
          (progn
            (setq FM-vec (scale -0.01 FM-vec))
            (if debug
                (format t "FM-vec for ~A: ~A~%" limb FM-vec))
            FM-vec))))

  (:publish-force (FM limb topic-name frame-id)
    "Publish the estimated force and torque for specified limb"
    (let ((msg (instance geometry_msgs::WrenchStamped :init)))
      (send msg :header :stamp (ros::time-now))
      (send msg :header :frame_id frame-id)
      (let ((force-vec (subseq FM 0 3))
            (torque-vec (subseq FM 3 6)))
        (send msg :wrench :force :x (float (aref force-vec 0)))
        (send msg :wrench :force :y (float (aref force-vec 1)))
        (send msg :wrench :force :z (float (aref force-vec 2)))
        (send msg :wrench :torque :x (float (aref torque-vec 0)))
        (send msg :wrench :torque :y (float (aref torque-vec 1)))
        (send msg :wrench :torque :z (float (aref torque-vec 2))))
      (ros::publish topic-name msg)))

  (:estimate-external-force ()
    "Estimate and publish external force for both arms"
    (let (real-tau gravity-tau ext-tau rarm-ext-force larm-ext-force)
      (setq real-tau (send self :get-real-torque))
      (setq gravity-tau (send self :calc-gravity-torque))
      (setq ext-tau (v- real-tau gravity-tau))
      ;; Right arm
      (setq rarm-ext-force (send self :calc-external-force ext-tau :rarm rarm-link-index-list))
      (if rarm-ext-force
          (send self :publish-force rarm-ext-force :rarm rarm-topic-name "r_link7"))
      ;; Left arm
      (setq larm-ext-force (send self :calc-external-force ext-tau :larm larm-link-index-list))
      (if larm-ext-force
          (send self :publish-force larm-ext-force :larm larm-topic-name "l_link7"))
      (if debug (format t "--------------------------------~%"))))

  (:set-debug (val)
    "Set debug mode on or off"
    (send self :set-val 'debug val)
    (format t "Debug mode set to ~A~%" debug))
)

;; Force Estimator instance
(setq *estimator* (instance force-estimator :init :robot *sciurus17* :interface *ri* :debug t))

(ros::rate 100)
(while (ros::ok)
  (ros::spin-once)
  (send *estimator* :estimate-external-force)
  (ros::sleep))

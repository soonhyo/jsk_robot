(require :baxter "package://baxtereus/baxter-interface.l")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "baxter_core_msgs")

(ros::roseus "force_estimator_with_gravity_compensation")

(baxter-init)
(unless (boundp '*baxter*) (error "Robot *baxter* is not initialized"))
(unless (boundp '*ri*) (error "Robot interface *ri* is not initialized"))

;; current → torque reconfigured coefficients
(setq *Kt* '(1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1))  ;; [Nm/A], all of joints

;; Coordinate transformation functions
(defun transform-euslisp-to-ros-baxter (force-vec torque-vec limb)
  "Transform force/torque from EusLisp coordinate system to ROS Baxter coordinate system"
  ;; EusLisp: X=forward, Y=left, Z=up
  ;; ROS Baxter: Z=forward, X=right(for right)/left(for left), Y=up
  (cond
    ((eq limb :rarm)
     ;; Right hand transformation: X->-Z, Y->Y, Z->X
     (list (float-vector (- (aref force-vec 2))
                         (aref force-vec 1)
                         (aref force-vec 0))
           (float-vector (- (aref torque-vec 2))
                         (aref torque-vec 1)
                         (aref torque-vec 0))))
    ((eq limb :larm)
     ;; Left hand transformation: X->Z, Y->X, Z->Y
     (list (float-vector (- (aref force-vec 2))
                         (aref force-vec 1)
                         (aref force-vec 0))
           (float-vector (- (aref torque-vec 2))
                         (aref torque-vec 1)
                         (aref torque-vec 0))))
    (t
     ;; No transformation (identity)
     (list force-vec torque-vec))))

(defun transform-euslisp-to-standard (force-vec torque-vec limb)
  "Transform force/torque from EusLisp coordinate system to standard robotics coordinate system"
  ;; EusLisp: X=forward, Y=left, Z=up
  ;; Standard: X=forward, Y=left, Z=up (same as EusLisp)
  (list force-vec torque-vec))

;; Generic Force Estimator class
(defclass force-estimator
  :super propertied-object
  :slots (rarm-end-coords larm-end-coords rarm-link-list larm-link-list rarm-link-index-list larm-link-index-list
          joint-list-all rarm-topic-name larm-topic-name robot robot-interface gravity real debug
          use-topic-gravity rarm-gravity-topic larm-gravity-topic
          rarm-gravity-torque larm-gravity-torque gravity-torque-received
          rarm-frame-id larm-frame-id coordinate-transform-function)
  :documentation "Generic class for estimating external force on robot arms")

(defmethod force-estimator
  (:init (&key (robot *baxter*) (interface *ri*) (gravity nil) (real nil) (debug nil)
               (use-topic-gravity nil) (output-frame nil) (coordinate-transform :ros-baxter))
    "Initialize the force estimator"
    (send self :set-val 'robot robot)
    (send self :set-val 'robot-interface interface)
    (send self :set-val 'gravity gravity)
    (send self :set-val 'real real)
    (send self :set-val 'debug debug)
    (send self :set-val 'use-topic-gravity use-topic-gravity)
    (send self :set-val 'gravity-torque-received nil)

    ;; Set coordinate transformation function
    (case coordinate-transform
      (:ros-baxter (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-ros-baxter))
      (:standard (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-standard))
      (t (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-standard)))

    ;; Output frame setting: default to end-effector frames
    (if output-frame
        (progn
          (send self :set-val 'rarm-frame-id output-frame)
          (send self :set-val 'larm-frame-id output-frame))
        (progn
          (send self :set-val 'rarm-frame-id "right_hand")  ;; End-effector frame
          (send self :set-val 'larm-frame-id "left_hand")))  ;; End-effector frame

    ;; Robot-specific initialization
    (send self :init-robot-specific robot)

    ;; ROS publishers
    (send self :set-val 'rarm-topic-name "/rarm_force_estimation")
    (send self :set-val 'larm-topic-name "/larm_force_estimation")
    (ros::advertise rarm-topic-name geometry_msgs::WrenchStamped 10)
    (ros::advertise larm-topic-name geometry_msgs::WrenchStamped 10)

    ;; ROS subscribers for gravity compensation torques (if using topic gravity)
    (send self :init-gravity-subscribers)

    self)

  (:init-robot-specific (robot)
    "Initialize robot-specific parameters"
    ;; This can be overridden for different robots
    ;; For Baxter:
    (send self :set-val 'rarm-end-coords (send robot :link "right_hand"))
    (send self :set-val 'rarm-link-list (send robot :rarm :links))
    (send self :set-val 'rarm-link-index-list
          (mapcar #'(lambda (joint-name) (position joint-name (send-all (send robot :joint-list) :name)))
                  (send-all (send robot :rarm :joint-list) :name)))

    (send self :set-val 'larm-end-coords (send robot :link "left_hand"))
    (send self :set-val 'larm-link-list (send robot :larm :links))
    (send self :set-val 'larm-link-index-list
          (mapcar #'(lambda (joint-name) (position joint-name (send-all (send robot :joint-list) :name)))
                  (send-all (send robot :larm :joint-list) :name)))

    (send self :set-val 'joint-list-all (send robot :joint-list))

    ;; Initialize gravity torque vectors
    (send self :set-val 'rarm-gravity-torque (instantiate float-vector 7))
    (send self :set-val 'larm-gravity-torque (instantiate float-vector 7)))

  (:init-gravity-subscribers ()
    "Initialize gravity compensation subscribers"
    (if use-topic-gravity
        (progn
          ;; For Baxter - can be overridden for other robots
          (send self :set-val 'rarm-gravity-topic "/robot/limb/right/gravity_compensation_torques")
          (send self :set-val 'larm-gravity-topic "/robot/limb/left/gravity_compensation_torques")
          (ros::subscribe rarm-gravity-topic baxter_core_msgs::SEAJointState
                          #'send self :rarm-gravity-callback)
          (ros::subscribe larm-gravity-topic baxter_core_msgs::SEAJointState
                          #'send self :larm-gravity-callback)
          (if debug (format t "Subscribed to gravity compensation torque topics~%")))))

  (:rarm-gravity-callback (msg)
    "Callback function for right arm gravity compensation torque"
    (let ((gravity-effort (send msg :gravity_model_effort)))
      (send self :set-val 'rarm-gravity-torque (coerce gravity-effort float-vector))
      (send self :set-val 'gravity-torque-received t)
      (if debug
          (format t "Received right arm gravity torque: ~A~%" rarm-gravity-torque))))

  (:larm-gravity-callback (msg)
    "Callback function for left arm gravity compensation torque"
    (let ((gravity-effort (send msg :gravity_model_effort)))
      (send self :set-val 'larm-gravity-torque (coerce gravity-effort float-vector))
      (send self :set-val 'gravity-torque-received t)
      (if debug
          (format t "Received left arm gravity torque: ~A~%" larm-gravity-torque))))

  (:calc-gravity-torque ()
    "Calculate gravity-induced torque for entire robot"
    (if debug (format t "Calculating Gravity Torque:~%"))
    (let* ((torque-vector (send robot :calc-torque
                                :calc-statics-p t
                                :debug-view nil
                                :dt 0.005)))
      (if debug
          (format t "Final Gravity Torque Vector: ~A Nm~%" torque-vector))
      torque-vector))

  (:get-topic-gravity-torque ()
    "Get gravity torque from ROS topics for entire robot"
    (let ((full-gravity-torque (instantiate float-vector (length (send robot :angle-vector)))))
      ;; Fill right arm gravity torque
      (dotimes (i (length rarm-link-index-list))
        (setf (aref full-gravity-torque (nth i rarm-link-index-list))
              (aref rarm-gravity-torque i)))
      ;; Fill left arm gravity torque
      (dotimes (i (length larm-link-index-list))
        (setf (aref full-gravity-torque (nth i larm-link-index-list))
              (aref larm-gravity-torque i)))
      (if debug
          (format t "Topic-based Gravity Torque Vector: ~A Nm~%" full-gravity-torque))
      full-gravity-torque))

  (:get-real-torque ()
    "Get real torque with reconfiguration for entire robot"
    (let (tau tau-re)
      (setq tau (send robot-interface :torque-vector))
      (setq tau-re (map float-vector #'* (coerce tau cons) *Kt*))
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
        ;; Apply coordinate transformation
        (let ((transformed (funcall coordinate-transform-function force-vec torque-vec limb)))
          (let ((trans-force (first transformed))
                (trans-torque (second transformed)))
            (send msg :wrench :force :x (float (aref trans-force 0)))
            (send msg :wrench :force :y (float (aref trans-force 1)))
            (send msg :wrench :force :z (float (aref trans-force 2)))
            (send msg :wrench :torque :x (float (aref trans-torque 0)))
            (send msg :wrench :torque :y (float (aref trans-torque 1)))
            (send msg :wrench :torque :z (float (aref trans-torque 2))))))
      (if debug
          (format t "Published force for ~A in ~A frame (coordinate transform applied)~%" limb frame-id))
      (ros::publish topic-name msg)))

  (:estimate-external-force ()
    "Estimate and publish external force for both arms"
    (let (real-tau gravity-tau ext-tau rarm-ext-force larm-ext-force)
      (send robot-interface :update-robot-state)
      (send robot :angle-vector (send robot-interface :state :potentio-vector))

      (if real
	        (setq real-tau (send self :get-real-torque))
	        (setq real-tau (instantiate float-vector (length (send robot :angle-vector)))))

      (if gravity
          (progn
            ;; Choose gravity calculation method based on use-topic-gravity option
            (if (and use-topic-gravity gravity-torque-received)
                (progn
                  (setq gravity-tau (send self :get-topic-gravity-torque))
                  (if debug (format t "Using topic-based gravity compensation~%")))
                (progn
                  (setq gravity-tau (send self :calc-gravity-torque))
                  (if debug (format t "Using internal gravity calculation~%"))))
	          (if real
		            (setq ext-tau (v- real-tau gravity-tau))
	              (setq ext-tau gravity-tau)))
          (setq ext-tau real-tau))

      ;; Right arm
      (setq rarm-ext-force (send self :calc-external-force ext-tau :rarm rarm-link-index-list))
      (if rarm-ext-force
          (send self :publish-force rarm-ext-force :rarm rarm-topic-name rarm-frame-id))
      ;; Left arm
      (setq larm-ext-force (send self :calc-external-force ext-tau :larm larm-link-index-list))
      (if larm-ext-force
          (send self :publish-force larm-ext-force :larm larm-topic-name larm-frame-id))
      (if debug (format t "--------------------------------~%"))))

  (:set-debug (val)
    "Set debug mode on or off"
    (send self :set-val 'debug val)
    (format t "Debug mode set to ~A~%" debug))

  (:set-use-topic-gravity (val)
    "Set whether to use topic-based gravity compensation"
    (send self :set-val 'use-topic-gravity val)
    (format t "Use topic gravity set to ~A~%" use-topic-gravity))

  (:set-coordinate-transform (transform-type)
    "Set coordinate transformation type"
    (case transform-type
      (:ros-baxter (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-ros-baxter))
      (:standard (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-standard))
      (t (send self :set-val 'coordinate-transform-function #'transform-euslisp-to-standard)))
    (format t "Coordinate transform set to ~A~%" transform-type))
)

;; Force Estimator instance examples:

;; For Baxter with ROS coordinate system
(setq *estimator* (instance force-estimator :init
                            :robot *baxter*
                            :interface *ri*
                            :gravity t
                            :real t
                            :debug t
                            :use-topic-gravity t
                            :coordinate-transform :ros-baxter))

(defun loop-hz-debug ()
  (ros::rate 200)  ;; 200 Hz loop
  (while (ros::ok)
    (let ((start-time (ros::time-now)))
      (ros::spin-once)
      (send *estimator* :estimate-external-force)
      (ros::sleep)
      (let ((elapsed-time (send (ros::time- (ros::time-now) start-time) :to-sec)))
        (if (> elapsed-time 0)
            (ros::ros-info "Update-robot-state frequency: ~A Hz" (/ 1.0 elapsed-time))
            (ros::ros-info "Update-robot-state frequency: N/A (elapsed time too small)"))))))

(defun main ()
  (ros::rate 200)
  (while (ros::ok)
    (ros::spin-once)
    (send *estimator* :estimate-external-force)
    (ros::sleep)))

;; main
(loop-hz-debug)

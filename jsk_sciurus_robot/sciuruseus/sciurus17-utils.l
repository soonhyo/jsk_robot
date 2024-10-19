(require :robot-interface "package://pr2eus/robot-interface.l")
(require :sciurus17 "package://sciuruseus/sciurus17.l")

(defmethod sciurus17-robot
  (:ik-prepared-poses
    (&optional (poses :null))
    "(:ik-prepared-poses) returns (get self :ik-prepared-poses), unless it is :null or (:reset-pose)
    (:ik-prepared-poses nil) set :null to (get self :ik-prepared-poses)
    (:ik-prepared-poses :any-posture) set :any-posutre to (get self :ik-prepared-poses)
    "
    (let ((ik-prepared-poses (or (get self :ik-prepared-poses) '(:reset-pose))))
      (ros::ros-debug "[:ik-prepared-poses] called with ~A" poses)
      (cond ((eq poses :null)
             (ros::ros-debug "[:ik-prepared-poses] returns ~A" (if (eq ik-prepared-poses :null) nil ik-prepared-poses))
             (return-from :ik-prepared-poses (if (eq ik-prepared-poses :null) nil ik-prepared-poses)))
            ((eq poses nil)
             (setf (get self :ik-prepared-poses) :null)
             (ros::ros-debug "[:ik-prepared-poses] returns nil")
             (return-from :ik-prepared-poses nil))
            (t
              (setf (get self :ik-prepared-poses) poses)
              (ros::ros-debug "[:ik-prepared-poses] returns ~A" poses)
              (return-from :ik-prepared-poses poses))
            )))
  (:inverse-kinematics-raw (&rest args)
    (send-super* :inverse-kinematics args))
  (:inverse-kinematics
   (target-coords &rest args &key (avoid-collision-distance 5) (warnp) (dump-command) &allow-other-keys)
   "Compute Inverse Kinematics with some strategies.
    The computation runs until the IK is solved as below:

      1. From current pose.
      2. Split points between current and target coords from each.
      3. From prepared poses. Poses in `:ik-prepared-poses` methods are used,
          and if it is not defined `:reset-pose` is used."
   (ros::ros-debug "call ik of ~A" sciurus17-robot)
   (let ((r) (prev-av (send self :angle-vector)))
     (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
     (unless r
       (format *error-output* "; failed for normal ik, starting from relaxed position~%")
       (let* ((move-target (cadr (memq :move-target args)))
              (current-coords (if (listp move-target)
                                (mapcar #'(lambda (x) (send x :copy-worldcoords)) move-target)
                                (send move-target :copy-worldcoords)))
              (rotation-axis (if (listp current-coords) (make-list (length current-coords)) nil)))
         (send-super* :inverse-kinematics current-coords :move-target move-target :rotation-axis rotation-axis :avoid-nspace-gain 0.1 :avoid-weight-gain 0.1 :stop 200
                      :avoid-collision-distance avoid-collision-distance args)
         (setq r (send-super* :inverse-kinematics target-coords :warnp warnp :dump-command dump-command args))
         (if (and (null r) (or (null (memq :revert-if-fail args)) (cadr (memq :revert-if-fail args))))
             (send self :angle-vector prev-av))
         ))
     (unless r
       (format *error-output* "; failed for normal ik, try to move arms very slowly~%")
       (let* ((step 0.0)
              (move-target (cadr (memq :move-target args)))
              (current-coords (if (listp move-target)
                                (mapcar #'(lambda (x) (send x :copy-worldcoords)) move-target)
                                (send move-target :copy-worldcoords))))
         (setq r t)
         (while (and r (<= step 1.0))
           (let ((mid-coords (if (listp current-coords)
                               (mapcar #'(lambda (x) (midcoords step (car x) (cadr x)))
                                       (mapcar #'list current-coords target-coords))
                               (midcoords step current-coords target-coords))))
             (setq r (send-super* :inverse-kinematics mid-coords
                                  :move-target move-target :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
             (incf step 0.01)))
         (unless r (send self :angle-vector prev-av))))
     (unless r
       (format *error-output* "; failed for slow ik, try to start from prepared poses~%")
       (let* ((link-list (cadr (memq :link-list args)))
              (move-joints
                (if (listp link-list)
                  (apply #'append (mapcar #'(lambda (x) (send-all x :joint)) link-list))
                  (send-all link-list :joint)))
              (av (mapcar #'(lambda (j) (send j :joint-angle)) (send self :joint-list)))
              (ik-prepared-poses (send self :ik-prepared-poses)))
         (dolist (pose ik-prepared-poses)
           (unless r
             (format *error-output* "; starting from prepared pose '~A'~%" pose)
             (send self pose)
             (mapcar #'(lambda (j a)
                         (if (not (memq j move-joints))
                           (send j :joint-angle a)))
                     (send self :joint-list) av)
             (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
             )
           )
         (when (and (null r) (or (null (memq :revert-if-fail args)) (cadr (memq :revert-if-fail args))))
             (when dump-command
               (format *error-output* "; failed all ik attempts, dump all data to /tmp/irtmodel-ik-~A~%" (unix::getpid))
               (format *error-output* ";      base coords ~A~%" (send self :worldcoords))
               (format *error-output* ";    target coords ~A~%" target-coords))
             (send self :angle-vector prev-av))
         ))
     r))
  (:l/r-reverse
    (av)
    "return the other arm's angle vector"
    (let ((av-tmp (copy-list av)) index)
      (dotimes (i 7)
        (setq index (* i 2))
        (setf (elt av-tmp index) (- (elt av-tmp index))))
      av-tmp))
  )

(defclass sciurus17-robot-safe
  :super sciurus17-robot
  :slots ())
(defmethod sciurus17-robot-safe
  (:init (&rest args)
    (send-super* :init args)
    ;; add chest and remove base from all links
    (setq collision-avoidance-links (remove-if #'(lambda (x) (string= (send x :name) "base_link")) (append (list (send self :chest_link)) (send self :links))))
    (setq larm-collision-avoidance-links collision-avoidance-links)
    (setq rarm-collision-avoidance-links collision-avoidance-links)
    ;; Set max joint velocities (you may need to adjust these values)
    (send (send self :l_arm_joint1) :max-joint-velocity 1)
    (send (send self :l_arm_joint2) :max-joint-velocity 1)
    (send (send self :l_arm_joint3) :max-joint-velocity 1)
    (send (send self :l_arm_joint4) :max-joint-velocity 1)
    (send (send self :l_arm_joint5) :max-joint-velocity 1)
    (send (send self :l_arm_joint6) :max-joint-velocity 1)
    (send (send self :l_arm_joint7) :max-joint-velocity 1)

    (send (send self :r_arm_joint1) :max-joint-velocity 1)
    (send (send self :r_arm_joint2) :max-joint-velocity 1)
    (send (send self :r_arm_joint3) :max-joint-velocity 1)
    (send (send self :r_arm_joint4) :max-joint-velocity 1)
    (send (send self :r_arm_joint5) :max-joint-velocity 1)
    (send (send self :r_arm_joint6) :max-joint-velocity 1)
    (send (send self :r_arm_joint7) :max-joint-velocity 1)
    )
  ;; collision check methods
  (:collision-check-pairs
   (&rest args)
   (let (pairs)
     (if args (warning-message 1 ":collision-check-pairs is re-defined in sciurus17-robot-safe class and not intend to take any arguments ~A" args))
     (setq pairs (send-super :collision-check-pairs :links collision-avoidance-links))
     ;; You may need to remove specific collision pairs for Sciurus17 if necessary
     pairs))
  )

(provide :sciurus17-utils)

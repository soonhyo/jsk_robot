(require :robot-interface "package://pr2eus/robot-interface.l")
(require :sciurus17 "package://sciurus17eus/sciurus17.l")

(defmethod sciurus17-robot
  )

(defclass sciurus17-robot-safe
  :super sciurus17-robot
  :slots ())
(defmethod sciurus17-robot-safe
  (:init (&rest args)
    (send-super* :init args)
    ;; add chest and remove base from all links
    (setq collision-avoidance-links (remove-if #'(lambda (x) (string= (send x :name) "world")) (send self :links)))

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

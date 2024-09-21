(require :mycobot "package://mycoboteus/mycobot.l")


(unless (assoc :init-orig (send firefighter-robot :methods))
  (rplaca (assoc :init (send firefighter-robot :methods)) :init-orig))


(defmethod firefighter-robot
  (:init
   (&rest args)
   ;; grey
   (dolist (b (list
                :_make_instance_joint1_geom0
                :_make_instance_joint6_flange_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.5 0.5 0.5 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send  (class self) :methods))))))
   (send* self :init-orig args))


(provide :mycobot-utils "package://mycoboteus/mycobot-utils.l")

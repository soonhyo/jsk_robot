#!/usr/bin/env roseus
(load "package://mycoboteus/mycobot-interface.l")
(load "models/arrow-object.l")

(setq *arrow-mycobot-rarm* (arrow))
(setq *arrow-mycobot-target* (arrow))

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "hair_flow_detect")

(mycobot-init)


(send *arrow-mycobot-rarm* :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
(send *robot* :rarm :end-coords :assoc *arrow-mycobot-rarm*)


(objects (list *robot* *arrow-mycobot-rarm* *arrow-mycobot-target*))

(defun joint-state-update ()
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :draw-objects)
  )

(defun comb-from-pose (pose frame-id)
  (let
      (comb-path-coords (a nil) camera->base (stamp (ros::time 0)) )

    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))

    (send *tfl* :wait-for-transform
          "link1" frame-id stamp 10.0)

    (setq camera->base
          (send *tfl* :lookup-transform
                "link1" frame-id stamp))

    (setq comb-path-coords
          (send (send camera->base :copy-worldcoords)
                :transform (ros::tf-pose->coords pose)))


    (send comb-path-coords :rotate (/ -pi 2) :y)
    (send comb-path-coords :rotate (/ -pi 2) :x)

    ;; (send comb-path-coords :rotate -pi :y)
    ;; (send comb-path-coords :translate (float-vector *offset-x* *offset-y* *offset-z*) :local)
    (send *arrow-mycobot-target* :newcoords (send comb-path-coords :copy-worldcoords))

    (print comb-path-coords)

    (objects (list *robot* *arrow-mycobot-rarm* *arrow-mycobot-target*))

    ;; inverse-kinematics
    (setq a (send *robot* :rarm :inverse-kinematics comb-path-coords :rotation-axis nil))
    ;; (send *robot* :rarm :inverse-kinematics comb-path-coords :rotation-axis nil)
    ;; send to real robot
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;; move end pose
    (if (not (eq a nil))
        (progn
          (send *robot* :rarm :move-end-pos #f(0 0 -50) :world)

          ;; send to real robot
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)

          (send *robot* :rarm :move-end-pos #f(-40 0 0) :local)

          ;; send to real robot
          (send *ri* :angle-vector (send *robot* :angle-vector) 300)
          (send *ri* :wait-interpolation)

        )
    )
  )
;; get pose array from topic
(defun pose-array-cb (pose-array-msg)
  (print pose-array-msg)
  (let*
      ((poses (send pose-array-msg :result :pose_array :poses))
       (header (send pose-array-msg :result :pose_array :header))
       )

    ;; (pose (if poses (car poses))))
    (if poses
        (progn
          (print "pose")
          (comb-from-pose (car poses) (send header :frame_id))
            )
        )
    )
  )



(ros::roseus "combloop")

(do-until-key
  ;; (joint-state-update)
  (setq path-once (one-shot-subscribe "/pathpcd/result"
                                      hair_flow_detect::PathPcdActionResult
                                      :timeout (* 10 1000)))
  (pose-array-cb path-once)
)



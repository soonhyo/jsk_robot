#!/usr/bin/env roseus
(load "package://mycoboteus/mycobot-interface.l")
(load "models/arrow-object.l")

(setq *arrow-mycobot-rarm* (arrow))
(setq *arrow-mycobot-target* (arrow))

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "hair_flow_detect")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(mycobot-init)


(send *arrow-mycobot-rarm* :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
(send *robot* :rarm :end-coords :assoc *arrow-mycobot-rarm*)

(setq *target-cube* nil)

(objects (list *robot* *arrow-mycobot-rarm* *arrow-mycobot-target*))

(defun joint-state-update ()
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :draw-objects)
  )

(defun grip-cb (box)
  (let (target-coords grip-coords)
    (if box
        (progn
          (setq target-coords (ros::tf-pose->coords (send box :pose)))
          (setq *target-cube* (make-cube (* 1000 (send box :dimensions :x))
                                         (* 1000 (send box :dimensions :y))
                                         (* 1000 (send box :dimensions :z))))
          (send *target-cube* :newcoords (send target-coords :copy-worldcoords))
          (objects (list *robot* *arrow-mycobot-rarm* *arrow-mycobot-target* *target-cube*))

          (setq grip-coords target-coords)
          (send grip-coords :translate #f(0 10 -40) :local)

          (send *robot* :rarm :inverse-kinematics grip-coords :rotation-axis :z)
          (send *ri* :angle-vector (send *robot* :angle-vector))
          (send *ri* :wait-interpolation)

          ;;grip
          ;; (send *ri* :start-grip)

          )
        )
    )
  )

;; bbox-msg callback function
(defun bbox-cb (bbox-msg)
  (let*
      ((boxes (send bbox-msg :boxes)) box)
    (dolist (b boxes)
      (if (> (send b :pose :position :x) 0)
          (setq box b)))
    (if box
        (progn
          (print (send box :pose :position :x))
          (grip-cb box)
          )
        )
    )
)

(ros::roseus "bbox_grip")
(do-until-key
    (joint-state-update)
  (bbox-cb (one-shot-subscribe "/docker/detic_segmentor/output/boxes"
                               jsk_recognition_msgs::BoundingBoxArray
                               :timeout (* 10 1000)))
  )



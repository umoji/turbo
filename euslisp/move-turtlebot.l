#!/usr/bin/env roseus
;; author: Tomoya Yoshizawa

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")

(ros::load-ros-manifest "roseus")

(ros::roseus "move-turtlebot")
(ros::rate 100)
(ros::subscribe "/pro/move_turtlebot" std_msgs::String #'move-turtlebot)


(defun init-robot ()
  ;; init robot
  (dxl-armed-turtlebot-init)

      (send *irtviewer* :objects (list *dxl-armed-turtlebot*))
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
      (unix::sleep 1)
    nil)

(defun move-turtlebot (msg)
  (case msg  (1 (send *ri* :go-velocity 0.0 0 10))
              (2 (send *ri* :go-velocity 0.0 0 -10))
              (3 (send *ri* :go-velocity 1.0 0 10))
              (4 (send *ri* :go-velocity -1.0 0 10))
  )

(init-robot)

(do-until-key
  (ros::spin-once)
  )
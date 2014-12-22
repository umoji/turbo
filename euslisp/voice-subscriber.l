#!/usr/bin/env roseus


(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_gui_msgs")

(ros::roseus "voice-subscriber")

; (load "package://pr2eus/pr2-interface.l")
; (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")

; (ros::advertise "robotsound" sound_play::SoundRequest 5)
; (unix:sleep 1)

; (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
; (unix:sleep 1)


; (dxl-armed-turtlebot-init)
; (when (not (boundp '*irtviewer*))
;   (make-irtviewer)
;   (objects (list *dxl-armed-turtlebot*))
;   (send *irtviewer* :draw-objects)
; )

; ; (setq *ri* (instance pr2-interface :init))
; (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 5000)


; (setq LOADED_PR2 t)
(load "voice-detector.l")

(defclass voice-listener
  :super propertied-object
  :slots ()
)
(defmethod voice-listener
  (:init ()
	 (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'send self :listen)
  )
  (:listen (msg)
      (setq data (send msg :texts))
      (print data)
      (dialogue data)
      ; (name data)
  )
)


(setq l (instance voice-listener :init))

(print "voice operation: start")

(do-until-key
 (ros::spin-once)
)

(print "voice operation: stop")

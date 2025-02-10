;; Auto-generated. Do not edit!


(when (boundp 'flapper::Stabilizer)
  (if (not (find-package "FLAPPER"))
    (make-package "FLAPPER"))
  (shadow 'Stabilizer (find-package "FLAPPER")))
(unless (find-package "FLAPPER::STABILIZER")
  (make-package "FLAPPER::STABILIZER"))

(in-package "ROS")
;;//! \htmlinclude Stabilizer.msg.html


(defclass flapper::Stabilizer
  :super ros::object
  :slots (_roll _pitch _yaw _thrust ))

(defmethod flapper::Stabilizer
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:thrust __thrust) 0.0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _thrust (float __thrust))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:serialization-length
   ()
   (+
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; float64 _thrust
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get flapper::Stabilizer :md5sum-) "028032831aa76f2200dc4a9de4c3b78b")
(setf (get flapper::Stabilizer :datatype-) "flapper/Stabilizer")
(setf (get flapper::Stabilizer :definition-)
      "float64 roll
float64 pitch
float64 yaw
float64 thrust
")



(provide :flapper/Stabilizer "028032831aa76f2200dc4a9de4c3b78b")



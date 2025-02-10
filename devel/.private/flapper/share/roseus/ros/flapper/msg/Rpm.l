;; Auto-generated. Do not edit!


(when (boundp 'flapper::Rpm)
  (if (not (find-package "FLAPPER"))
    (make-package "FLAPPER"))
  (shadow 'Rpm (find-package "FLAPPER")))
(unless (find-package "FLAPPER::RPM")
  (make-package "FLAPPER::RPM"))

(in-package "ROS")
;;//! \htmlinclude Rpm.msg.html


(defclass flapper::Rpm
  :super ros::object
  :slots (_m1 _m2 _m3 _m4 ))

(defmethod flapper::Rpm
  (:init
   (&key
    ((:m1 __m1) 0)
    ((:m2 __m2) 0)
    ((:m3 __m3) 0)
    ((:m4 __m4) 0)
    )
   (send-super :init)
   (setq _m1 (round __m1))
   (setq _m2 (round __m2))
   (setq _m3 (round __m3))
   (setq _m4 (round __m4))
   self)
  (:m1
   (&optional __m1)
   (if __m1 (setq _m1 __m1)) _m1)
  (:m2
   (&optional __m2)
   (if __m2 (setq _m2 __m2)) _m2)
  (:m3
   (&optional __m3)
   (if __m3 (setq _m3 __m3)) _m3)
  (:m4
   (&optional __m4)
   (if __m4 (setq _m4 __m4)) _m4)
  (:serialization-length
   ()
   (+
    ;; uint16 _m1
    2
    ;; uint16 _m2
    2
    ;; uint16 _m3
    2
    ;; uint16 _m4
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _m1
       (write-word _m1 s)
     ;; uint16 _m2
       (write-word _m2 s)
     ;; uint16 _m3
       (write-word _m3 s)
     ;; uint16 _m4
       (write-word _m4 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _m1
     (setq _m1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _m2
     (setq _m2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _m3
     (setq _m3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _m4
     (setq _m4 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get flapper::Rpm :md5sum-) "29ed5a63544f3887c01e281980c939e1")
(setf (get flapper::Rpm :datatype-) "flapper/Rpm")
(setf (get flapper::Rpm :definition-)
      "uint16 m1
uint16 m2
uint16 m3
uint16 m4
")



(provide :flapper/Rpm "29ed5a63544f3887c01e281980c939e1")



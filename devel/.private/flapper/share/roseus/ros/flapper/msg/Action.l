;; Auto-generated. Do not edit!


(when (boundp 'flapper::Action)
  (if (not (find-package "FLAPPER"))
    (make-package "FLAPPER"))
  (shadow 'Action (find-package "FLAPPER")))
(unless (find-package "FLAPPER::ACTION")
  (make-package "FLAPPER::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass flapper::Action
  :super ros::object
  :slots (_name _height ))

(defmethod flapper::Action
  (:init
   (&key
    ((:name __name) "")
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _height (float __height))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _height
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get flapper::Action :md5sum-) "70ac77d41564567d694a44436c064b02")
(setf (get flapper::Action :datatype-) "flapper/Action")
(setf (get flapper::Action :definition-)
      "string name
float64 height


")



(provide :flapper/Action "70ac77d41564567d694a44436c064b02")



; Auto-generated. Do not edit!


(cl:in-package flapper-msg)


;//! \htmlinclude Rpm.msg.html

(cl:defclass <Rpm> (roslisp-msg-protocol:ros-message)
  ((m1
    :reader m1
    :initarg :m1
    :type cl:fixnum
    :initform 0)
   (m2
    :reader m2
    :initarg :m2
    :type cl:fixnum
    :initform 0)
   (m3
    :reader m3
    :initarg :m3
    :type cl:fixnum
    :initform 0)
   (m4
    :reader m4
    :initarg :m4
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Rpm (<Rpm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rpm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rpm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flapper-msg:<Rpm> is deprecated: use flapper-msg:Rpm instead.")))

(cl:ensure-generic-function 'm1-val :lambda-list '(m))
(cl:defmethod m1-val ((m <Rpm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flapper-msg:m1-val is deprecated.  Use flapper-msg:m1 instead.")
  (m1 m))

(cl:ensure-generic-function 'm2-val :lambda-list '(m))
(cl:defmethod m2-val ((m <Rpm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flapper-msg:m2-val is deprecated.  Use flapper-msg:m2 instead.")
  (m2 m))

(cl:ensure-generic-function 'm3-val :lambda-list '(m))
(cl:defmethod m3-val ((m <Rpm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flapper-msg:m3-val is deprecated.  Use flapper-msg:m3 instead.")
  (m3 m))

(cl:ensure-generic-function 'm4-val :lambda-list '(m))
(cl:defmethod m4-val ((m <Rpm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flapper-msg:m4-val is deprecated.  Use flapper-msg:m4 instead.")
  (m4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rpm>) ostream)
  "Serializes a message object of type '<Rpm>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm4)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rpm>) istream)
  "Deserializes a message object of type '<Rpm>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'm4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'm4)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rpm>)))
  "Returns string type for a message object of type '<Rpm>"
  "flapper/Rpm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rpm)))
  "Returns string type for a message object of type 'Rpm"
  "flapper/Rpm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rpm>)))
  "Returns md5sum for a message object of type '<Rpm>"
  "29ed5a63544f3887c01e281980c939e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rpm)))
  "Returns md5sum for a message object of type 'Rpm"
  "29ed5a63544f3887c01e281980c939e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rpm>)))
  "Returns full string definition for message of type '<Rpm>"
  (cl:format cl:nil "uint16 m1~%uint16 m2~%uint16 m3~%uint16 m4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rpm)))
  "Returns full string definition for message of type 'Rpm"
  (cl:format cl:nil "uint16 m1~%uint16 m2~%uint16 m3~%uint16 m4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rpm>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rpm>))
  "Converts a ROS message object to a list"
  (cl:list 'Rpm
    (cl:cons ':m1 (m1 msg))
    (cl:cons ':m2 (m2 msg))
    (cl:cons ':m3 (m3 msg))
    (cl:cons ':m4 (m4 msg))
))

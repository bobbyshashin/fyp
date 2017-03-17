; Auto-generated. Do not edit!


(cl:in-package dji_pid-msg)


;//! \htmlinclude move_target.msg.html

(cl:defclass <move_target> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (stick_state
    :reader stick_state
    :initarg :stick_state
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass move_target (<move_target>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_target>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_target)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_pid-msg:<move_target> is deprecated: use dji_pid-msg:move_target instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <move_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:header-val is deprecated.  Use dji_pid-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'stick_state-val :lambda-list '(m))
(cl:defmethod stick_state-val ((m <move_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:stick_state-val is deprecated.  Use dji_pid-msg:stick_state instead.")
  (stick_state m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <move_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:x-val is deprecated.  Use dji_pid-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <move_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:y-val is deprecated.  Use dji_pid-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <move_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:z-val is deprecated.  Use dji_pid-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_target>) ostream)
  "Serializes a message object of type '<move_target>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stick_state)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_target>) istream)
  "Deserializes a message object of type '<move_target>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stick_state)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_target>)))
  "Returns string type for a message object of type '<move_target>"
  "dji_pid/move_target")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_target)))
  "Returns string type for a message object of type 'move_target"
  "dji_pid/move_target")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_target>)))
  "Returns md5sum for a message object of type '<move_target>"
  "9c07253b6b2b88818f5453c7b18d44a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_target)))
  "Returns md5sum for a message object of type 'move_target"
  "9c07253b6b2b88818f5453c7b18d44a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_target>)))
  "Returns full string definition for message of type '<move_target>"
  (cl:format cl:nil "Header header~%uint8 stick_state~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_target)))
  "Returns full string definition for message of type 'move_target"
  (cl:format cl:nil "Header header~%uint8 stick_state~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_target>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_target>))
  "Converts a ROS message object to a list"
  (cl:list 'move_target
    (cl:cons ':header (header msg))
    (cl:cons ':stick_state (stick_state msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))

; Auto-generated. Do not edit!


(cl:in-package dji_pid-msg)


;//! \htmlinclude pid_ctrl_data.msg.html

(cl:defclass <pid_ctrl_data> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ctrl_mode
    :reader ctrl_mode
    :initarg :ctrl_mode
    :type cl:fixnum
    :initform 0)
   (position_error
    :reader position_error
    :initarg :position_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position_target
    :reader position_target
    :initarg :position_target
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (status
    :reader status
    :initarg :status
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
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass pid_ctrl_data (<pid_ctrl_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pid_ctrl_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pid_ctrl_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_pid-msg:<pid_ctrl_data> is deprecated: use dji_pid-msg:pid_ctrl_data instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:header-val is deprecated.  Use dji_pid-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ctrl_mode-val :lambda-list '(m))
(cl:defmethod ctrl_mode-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:ctrl_mode-val is deprecated.  Use dji_pid-msg:ctrl_mode instead.")
  (ctrl_mode m))

(cl:ensure-generic-function 'position_error-val :lambda-list '(m))
(cl:defmethod position_error-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:position_error-val is deprecated.  Use dji_pid-msg:position_error instead.")
  (position_error m))

(cl:ensure-generic-function 'position_target-val :lambda-list '(m))
(cl:defmethod position_target-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:position_target-val is deprecated.  Use dji_pid-msg:position_target instead.")
  (position_target m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:status-val is deprecated.  Use dji_pid-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:x-val is deprecated.  Use dji_pid-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:y-val is deprecated.  Use dji_pid-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:z-val is deprecated.  Use dji_pid-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <pid_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_pid-msg:yaw-val is deprecated.  Use dji_pid-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pid_ctrl_data>) ostream)
  "Serializes a message object of type '<pid_ctrl_data>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ctrl_mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_target) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pid_ctrl_data>) istream)
  "Deserializes a message object of type '<pid_ctrl_data>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ctrl_mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_target) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pid_ctrl_data>)))
  "Returns string type for a message object of type '<pid_ctrl_data>"
  "dji_pid/pid_ctrl_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pid_ctrl_data)))
  "Returns string type for a message object of type 'pid_ctrl_data"
  "dji_pid/pid_ctrl_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pid_ctrl_data>)))
  "Returns md5sum for a message object of type '<pid_ctrl_data>"
  "398b87ab466e55b265ab53a2f032f659")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pid_ctrl_data)))
  "Returns md5sum for a message object of type 'pid_ctrl_data"
  "398b87ab466e55b265ab53a2f032f659")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pid_ctrl_data>)))
  "Returns full string definition for message of type '<pid_ctrl_data>"
  (cl:format cl:nil "Header header~%~%uint8 ctrl_mode~%~%geometry_msgs/Vector3 position_error~%geometry_msgs/Vector3 position_target~%uint8 status~%~%float32 x~%float32 y~%float32 z~%float32 yaw~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pid_ctrl_data)))
  "Returns full string definition for message of type 'pid_ctrl_data"
  (cl:format cl:nil "Header header~%~%uint8 ctrl_mode~%~%geometry_msgs/Vector3 position_error~%geometry_msgs/Vector3 position_target~%uint8 status~%~%float32 x~%float32 y~%float32 z~%float32 yaw~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pid_ctrl_data>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_target))
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pid_ctrl_data>))
  "Converts a ROS message object to a list"
  (cl:list 'pid_ctrl_data
    (cl:cons ':header (header msg))
    (cl:cons ':ctrl_mode (ctrl_mode msg))
    (cl:cons ':position_error (position_error msg))
    (cl:cons ':position_target (position_target msg))
    (cl:cons ':status (status msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
))

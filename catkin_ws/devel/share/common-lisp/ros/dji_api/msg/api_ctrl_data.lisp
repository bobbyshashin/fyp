; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_ctrl_data.msg.html

(cl:defclass <api_ctrl_data> (roslisp-msg-protocol:ros-message)
  ((ctrl_flag
    :reader ctrl_flag
    :initarg :ctrl_flag
    :type cl:float
    :initform 0.0)
   (horiz_mode
    :reader horiz_mode
    :initarg :horiz_mode
    :type cl:fixnum
    :initform 0)
   (vert_mode
    :reader vert_mode
    :initarg :vert_mode
    :type cl:fixnum
    :initform 0)
   (yaw_mode
    :reader yaw_mode
    :initarg :yaw_mode
    :type cl:fixnum
    :initform 0)
   (level_frame
    :reader level_frame
    :initarg :level_frame
    :type cl:fixnum
    :initform 0)
   (torsion_frame
    :reader torsion_frame
    :initarg :torsion_frame
    :type cl:fixnum
    :initform 0)
   (ctrl_data
    :reader ctrl_data
    :initarg :ctrl_data
    :type geometry_msgs-msg:QuaternionStamped
    :initform (cl:make-instance 'geometry_msgs-msg:QuaternionStamped)))
)

(cl:defclass api_ctrl_data (<api_ctrl_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_ctrl_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_ctrl_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_ctrl_data> is deprecated: use dji_api-msg:api_ctrl_data instead.")))

(cl:ensure-generic-function 'ctrl_flag-val :lambda-list '(m))
(cl:defmethod ctrl_flag-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:ctrl_flag-val is deprecated.  Use dji_api-msg:ctrl_flag instead.")
  (ctrl_flag m))

(cl:ensure-generic-function 'horiz_mode-val :lambda-list '(m))
(cl:defmethod horiz_mode-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:horiz_mode-val is deprecated.  Use dji_api-msg:horiz_mode instead.")
  (horiz_mode m))

(cl:ensure-generic-function 'vert_mode-val :lambda-list '(m))
(cl:defmethod vert_mode-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:vert_mode-val is deprecated.  Use dji_api-msg:vert_mode instead.")
  (vert_mode m))

(cl:ensure-generic-function 'yaw_mode-val :lambda-list '(m))
(cl:defmethod yaw_mode-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:yaw_mode-val is deprecated.  Use dji_api-msg:yaw_mode instead.")
  (yaw_mode m))

(cl:ensure-generic-function 'level_frame-val :lambda-list '(m))
(cl:defmethod level_frame-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:level_frame-val is deprecated.  Use dji_api-msg:level_frame instead.")
  (level_frame m))

(cl:ensure-generic-function 'torsion_frame-val :lambda-list '(m))
(cl:defmethod torsion_frame-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:torsion_frame-val is deprecated.  Use dji_api-msg:torsion_frame instead.")
  (torsion_frame m))

(cl:ensure-generic-function 'ctrl_data-val :lambda-list '(m))
(cl:defmethod ctrl_data-val ((m <api_ctrl_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:ctrl_data-val is deprecated.  Use dji_api-msg:ctrl_data instead.")
  (ctrl_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_ctrl_data>) ostream)
  "Serializes a message object of type '<api_ctrl_data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ctrl_flag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'horiz_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vert_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'yaw_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'level_frame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'torsion_frame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ctrl_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_ctrl_data>) istream)
  "Deserializes a message object of type '<api_ctrl_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ctrl_flag) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'horiz_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vert_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yaw_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level_frame) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'torsion_frame) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ctrl_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_ctrl_data>)))
  "Returns string type for a message object of type '<api_ctrl_data>"
  "dji_api/api_ctrl_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_ctrl_data)))
  "Returns string type for a message object of type 'api_ctrl_data"
  "dji_api/api_ctrl_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_ctrl_data>)))
  "Returns md5sum for a message object of type '<api_ctrl_data>"
  "17a4210b32e15b6ae5ea3f62362227fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_ctrl_data)))
  "Returns md5sum for a message object of type 'api_ctrl_data"
  "17a4210b32e15b6ae5ea3f62362227fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_ctrl_data>)))
  "Returns full string definition for message of type '<api_ctrl_data>"
  (cl:format cl:nil "float32 ctrl_flag~%int8    horiz_mode~%int8    vert_mode~%int8    yaw_mode~%int8    level_frame~%int8    torsion_frame~%geometry_msgs/QuaternionStamped ctrl_data~%~%~%~%~%================================================================================~%MSG: geometry_msgs/QuaternionStamped~%# This represents an orientation with reference coordinate frame and timestamp.~%~%Header header~%Quaternion quaternion~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_ctrl_data)))
  "Returns full string definition for message of type 'api_ctrl_data"
  (cl:format cl:nil "float32 ctrl_flag~%int8    horiz_mode~%int8    vert_mode~%int8    yaw_mode~%int8    level_frame~%int8    torsion_frame~%geometry_msgs/QuaternionStamped ctrl_data~%~%~%~%~%================================================================================~%MSG: geometry_msgs/QuaternionStamped~%# This represents an orientation with reference coordinate frame and timestamp.~%~%Header header~%Quaternion quaternion~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_ctrl_data>))
  (cl:+ 0
     4
     1
     1
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ctrl_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_ctrl_data>))
  "Converts a ROS message object to a list"
  (cl:list 'api_ctrl_data
    (cl:cons ':ctrl_flag (ctrl_flag msg))
    (cl:cons ':horiz_mode (horiz_mode msg))
    (cl:cons ':vert_mode (vert_mode msg))
    (cl:cons ':yaw_mode (yaw_mode msg))
    (cl:cons ':level_frame (level_frame msg))
    (cl:cons ':torsion_frame (torsion_frame msg))
    (cl:cons ':ctrl_data (ctrl_data msg))
))

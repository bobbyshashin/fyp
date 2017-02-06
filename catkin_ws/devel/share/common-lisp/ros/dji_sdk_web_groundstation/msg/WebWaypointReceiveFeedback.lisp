; Auto-generated. Do not edit!


(cl:in-package dji_sdk_web_groundstation-msg)


;//! \htmlinclude WebWaypointReceiveFeedback.msg.html

(cl:defclass <WebWaypointReceiveFeedback> (roslisp-msg-protocol:ros-message)
  ((latitude_progress
    :reader latitude_progress
    :initarg :latitude_progress
    :type cl:fixnum
    :initform 0)
   (longitude_progress
    :reader longitude_progress
    :initarg :longitude_progress
    :type cl:fixnum
    :initform 0)
   (altitude_progress
    :reader altitude_progress
    :initarg :altitude_progress
    :type cl:fixnum
    :initform 0)
   (index_progress
    :reader index_progress
    :initarg :index_progress
    :type cl:fixnum
    :initform 0)
   (stage
    :reader stage
    :initarg :stage
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WebWaypointReceiveFeedback (<WebWaypointReceiveFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WebWaypointReceiveFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WebWaypointReceiveFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_sdk_web_groundstation-msg:<WebWaypointReceiveFeedback> is deprecated: use dji_sdk_web_groundstation-msg:WebWaypointReceiveFeedback instead.")))

(cl:ensure-generic-function 'latitude_progress-val :lambda-list '(m))
(cl:defmethod latitude_progress-val ((m <WebWaypointReceiveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:latitude_progress-val is deprecated.  Use dji_sdk_web_groundstation-msg:latitude_progress instead.")
  (latitude_progress m))

(cl:ensure-generic-function 'longitude_progress-val :lambda-list '(m))
(cl:defmethod longitude_progress-val ((m <WebWaypointReceiveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:longitude_progress-val is deprecated.  Use dji_sdk_web_groundstation-msg:longitude_progress instead.")
  (longitude_progress m))

(cl:ensure-generic-function 'altitude_progress-val :lambda-list '(m))
(cl:defmethod altitude_progress-val ((m <WebWaypointReceiveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:altitude_progress-val is deprecated.  Use dji_sdk_web_groundstation-msg:altitude_progress instead.")
  (altitude_progress m))

(cl:ensure-generic-function 'index_progress-val :lambda-list '(m))
(cl:defmethod index_progress-val ((m <WebWaypointReceiveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:index_progress-val is deprecated.  Use dji_sdk_web_groundstation-msg:index_progress instead.")
  (index_progress m))

(cl:ensure-generic-function 'stage-val :lambda-list '(m))
(cl:defmethod stage-val ((m <WebWaypointReceiveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:stage-val is deprecated.  Use dji_sdk_web_groundstation-msg:stage instead.")
  (stage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WebWaypointReceiveFeedback>) ostream)
  "Serializes a message object of type '<WebWaypointReceiveFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latitude_progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'longitude_progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'altitude_progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stage)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WebWaypointReceiveFeedback>) istream)
  "Deserializes a message object of type '<WebWaypointReceiveFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latitude_progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'longitude_progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'altitude_progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stage)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WebWaypointReceiveFeedback>)))
  "Returns string type for a message object of type '<WebWaypointReceiveFeedback>"
  "dji_sdk_web_groundstation/WebWaypointReceiveFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WebWaypointReceiveFeedback)))
  "Returns string type for a message object of type 'WebWaypointReceiveFeedback"
  "dji_sdk_web_groundstation/WebWaypointReceiveFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WebWaypointReceiveFeedback>)))
  "Returns md5sum for a message object of type '<WebWaypointReceiveFeedback>"
  "0895e5924b05e8720f48006b778b54ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WebWaypointReceiveFeedback)))
  "Returns md5sum for a message object of type 'WebWaypointReceiveFeedback"
  "0895e5924b05e8720f48006b778b54ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WebWaypointReceiveFeedback>)))
  "Returns full string definition for message of type '<WebWaypointReceiveFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#progress is in percent~%uint8 latitude_progress~%uint8 longitude_progress~%uint8 altitude_progress~%uint8 index_progress~%#stage code:~%#  0: waiting for waypoint_list~%#  1: waiting for start~%#  2: in progress~%#  3: paused~%#  4: canceled~%uint8 stage~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WebWaypointReceiveFeedback)))
  "Returns full string definition for message of type 'WebWaypointReceiveFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#progress is in percent~%uint8 latitude_progress~%uint8 longitude_progress~%uint8 altitude_progress~%uint8 index_progress~%#stage code:~%#  0: waiting for waypoint_list~%#  1: waiting for start~%#  2: in progress~%#  3: paused~%#  4: canceled~%uint8 stage~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WebWaypointReceiveFeedback>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WebWaypointReceiveFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'WebWaypointReceiveFeedback
    (cl:cons ':latitude_progress (latitude_progress msg))
    (cl:cons ':longitude_progress (longitude_progress msg))
    (cl:cons ':altitude_progress (altitude_progress msg))
    (cl:cons ':index_progress (index_progress msg))
    (cl:cons ':stage (stage msg))
))

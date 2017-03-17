; Auto-generated. Do not edit!


(cl:in-package bluefox2-srv)


;//! \htmlinclude SetExposeSrv-request.msg.html

(cl:defclass <SetExposeSrv-request> (roslisp-msg-protocol:ros-message)
  ((expose_us
    :reader expose_us
    :initarg :expose_us
    :type cl:integer
    :initform 0))
)

(cl:defclass SetExposeSrv-request (<SetExposeSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExposeSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExposeSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluefox2-srv:<SetExposeSrv-request> is deprecated: use bluefox2-srv:SetExposeSrv-request instead.")))

(cl:ensure-generic-function 'expose_us-val :lambda-list '(m))
(cl:defmethod expose_us-val ((m <SetExposeSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluefox2-srv:expose_us-val is deprecated.  Use bluefox2-srv:expose_us instead.")
  (expose_us m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExposeSrv-request>) ostream)
  "Serializes a message object of type '<SetExposeSrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'expose_us)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExposeSrv-request>) istream)
  "Deserializes a message object of type '<SetExposeSrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'expose_us) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExposeSrv-request>)))
  "Returns string type for a service object of type '<SetExposeSrv-request>"
  "bluefox2/SetExposeSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposeSrv-request)))
  "Returns string type for a service object of type 'SetExposeSrv-request"
  "bluefox2/SetExposeSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExposeSrv-request>)))
  "Returns md5sum for a message object of type '<SetExposeSrv-request>"
  "75265de6308041cab01352a6d59078cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExposeSrv-request)))
  "Returns md5sum for a message object of type 'SetExposeSrv-request"
  "75265de6308041cab01352a6d59078cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExposeSrv-request>)))
  "Returns full string definition for message of type '<SetExposeSrv-request>"
  (cl:format cl:nil "int32 expose_us~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExposeSrv-request)))
  "Returns full string definition for message of type 'SetExposeSrv-request"
  (cl:format cl:nil "int32 expose_us~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExposeSrv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExposeSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExposeSrv-request
    (cl:cons ':expose_us (expose_us msg))
))
;//! \htmlinclude SetExposeSrv-response.msg.html

(cl:defclass <SetExposeSrv-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetExposeSrv-response (<SetExposeSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExposeSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExposeSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bluefox2-srv:<SetExposeSrv-response> is deprecated: use bluefox2-srv:SetExposeSrv-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetExposeSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bluefox2-srv:status-val is deprecated.  Use bluefox2-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExposeSrv-response>) ostream)
  "Serializes a message object of type '<SetExposeSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExposeSrv-response>) istream)
  "Deserializes a message object of type '<SetExposeSrv-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExposeSrv-response>)))
  "Returns string type for a service object of type '<SetExposeSrv-response>"
  "bluefox2/SetExposeSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposeSrv-response)))
  "Returns string type for a service object of type 'SetExposeSrv-response"
  "bluefox2/SetExposeSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExposeSrv-response>)))
  "Returns md5sum for a message object of type '<SetExposeSrv-response>"
  "75265de6308041cab01352a6d59078cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExposeSrv-response)))
  "Returns md5sum for a message object of type 'SetExposeSrv-response"
  "75265de6308041cab01352a6d59078cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExposeSrv-response>)))
  "Returns full string definition for message of type '<SetExposeSrv-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExposeSrv-response)))
  "Returns full string definition for message of type 'SetExposeSrv-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExposeSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExposeSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExposeSrv-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetExposeSrv)))
  'SetExposeSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetExposeSrv)))
  'SetExposeSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposeSrv)))
  "Returns string type for a service object of type '<SetExposeSrv>"
  "bluefox2/SetExposeSrv")
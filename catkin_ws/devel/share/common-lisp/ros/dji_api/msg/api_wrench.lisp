; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_wrench.msg.html

(cl:defclass <api_wrench> (roslisp-msg-protocol:ros-message)
  ((wx
    :reader wx
    :initarg :wx
    :type cl:float
    :initform 0.0)
   (wy
    :reader wy
    :initarg :wy
    :type cl:float
    :initform 0.0)
   (wz
    :reader wz
    :initarg :wz
    :type cl:float
    :initform 0.0))
)

(cl:defclass api_wrench (<api_wrench>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_wrench>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_wrench)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_wrench> is deprecated: use dji_api-msg:api_wrench instead.")))

(cl:ensure-generic-function 'wx-val :lambda-list '(m))
(cl:defmethod wx-val ((m <api_wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:wx-val is deprecated.  Use dji_api-msg:wx instead.")
  (wx m))

(cl:ensure-generic-function 'wy-val :lambda-list '(m))
(cl:defmethod wy-val ((m <api_wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:wy-val is deprecated.  Use dji_api-msg:wy instead.")
  (wy m))

(cl:ensure-generic-function 'wz-val :lambda-list '(m))
(cl:defmethod wz-val ((m <api_wrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:wz-val is deprecated.  Use dji_api-msg:wz instead.")
  (wz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_wrench>) ostream)
  "Serializes a message object of type '<api_wrench>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_wrench>) istream)
  "Deserializes a message object of type '<api_wrench>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_wrench>)))
  "Returns string type for a message object of type '<api_wrench>"
  "dji_api/api_wrench")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_wrench)))
  "Returns string type for a message object of type 'api_wrench"
  "dji_api/api_wrench")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_wrench>)))
  "Returns md5sum for a message object of type '<api_wrench>"
  "48d31eabbcefc0c5c4bd8b442da24d53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_wrench)))
  "Returns md5sum for a message object of type 'api_wrench"
  "48d31eabbcefc0c5c4bd8b442da24d53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_wrench>)))
  "Returns full string definition for message of type '<api_wrench>"
  (cl:format cl:nil "float32 wx~%float32 wy~%float32 wz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_wrench)))
  "Returns full string definition for message of type 'api_wrench"
  (cl:format cl:nil "float32 wx~%float32 wy~%float32 wz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_wrench>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_wrench>))
  "Converts a ROS message object to a list"
  (cl:list 'api_wrench
    (cl:cons ':wx (wx msg))
    (cl:cons ':wy (wy msg))
    (cl:cons ':wz (wz msg))
))

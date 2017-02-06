; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_velocity.msg.html

(cl:defclass <api_velocity> (roslisp-msg-protocol:ros-message)
  ((vgx
    :reader vgx
    :initarg :vgx
    :type cl:float
    :initform 0.0)
   (vgy
    :reader vgy
    :initarg :vgy
    :type cl:float
    :initform 0.0)
   (vgz
    :reader vgz
    :initarg :vgz
    :type cl:float
    :initform 0.0))
)

(cl:defclass api_velocity (<api_velocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_velocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_velocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_velocity> is deprecated: use dji_api-msg:api_velocity instead.")))

(cl:ensure-generic-function 'vgx-val :lambda-list '(m))
(cl:defmethod vgx-val ((m <api_velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:vgx-val is deprecated.  Use dji_api-msg:vgx instead.")
  (vgx m))

(cl:ensure-generic-function 'vgy-val :lambda-list '(m))
(cl:defmethod vgy-val ((m <api_velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:vgy-val is deprecated.  Use dji_api-msg:vgy instead.")
  (vgy m))

(cl:ensure-generic-function 'vgz-val :lambda-list '(m))
(cl:defmethod vgz-val ((m <api_velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:vgz-val is deprecated.  Use dji_api-msg:vgz instead.")
  (vgz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_velocity>) ostream)
  "Serializes a message object of type '<api_velocity>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vgx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vgy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vgz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_velocity>) istream)
  "Deserializes a message object of type '<api_velocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vgx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vgy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vgz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_velocity>)))
  "Returns string type for a message object of type '<api_velocity>"
  "dji_api/api_velocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_velocity)))
  "Returns string type for a message object of type 'api_velocity"
  "dji_api/api_velocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_velocity>)))
  "Returns md5sum for a message object of type '<api_velocity>"
  "cf5f9e1e9a93c3b5ada672ea4e8faf92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_velocity)))
  "Returns md5sum for a message object of type 'api_velocity"
  "cf5f9e1e9a93c3b5ada672ea4e8faf92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_velocity>)))
  "Returns full string definition for message of type '<api_velocity>"
  (cl:format cl:nil "float32 vgx~%float32 vgy~%float32 vgz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_velocity)))
  "Returns full string definition for message of type 'api_velocity"
  (cl:format cl:nil "float32 vgx~%float32 vgy~%float32 vgz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_velocity>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_velocity>))
  "Converts a ROS message object to a list"
  (cl:list 'api_velocity
    (cl:cons ':vgx (vgx msg))
    (cl:cons ':vgy (vgy msg))
    (cl:cons ':vgz (vgz msg))
))

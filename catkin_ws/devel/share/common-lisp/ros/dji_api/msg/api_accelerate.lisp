; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_accelerate.msg.html

(cl:defclass <api_accelerate> (roslisp-msg-protocol:ros-message)
  ((agx
    :reader agx
    :initarg :agx
    :type cl:float
    :initform 0.0)
   (agy
    :reader agy
    :initarg :agy
    :type cl:float
    :initform 0.0)
   (agz
    :reader agz
    :initarg :agz
    :type cl:float
    :initform 0.0))
)

(cl:defclass api_accelerate (<api_accelerate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_accelerate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_accelerate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_accelerate> is deprecated: use dji_api-msg:api_accelerate instead.")))

(cl:ensure-generic-function 'agx-val :lambda-list '(m))
(cl:defmethod agx-val ((m <api_accelerate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:agx-val is deprecated.  Use dji_api-msg:agx instead.")
  (agx m))

(cl:ensure-generic-function 'agy-val :lambda-list '(m))
(cl:defmethod agy-val ((m <api_accelerate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:agy-val is deprecated.  Use dji_api-msg:agy instead.")
  (agy m))

(cl:ensure-generic-function 'agz-val :lambda-list '(m))
(cl:defmethod agz-val ((m <api_accelerate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:agz-val is deprecated.  Use dji_api-msg:agz instead.")
  (agz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_accelerate>) ostream)
  "Serializes a message object of type '<api_accelerate>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_accelerate>) istream)
  "Deserializes a message object of type '<api_accelerate>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_accelerate>)))
  "Returns string type for a message object of type '<api_accelerate>"
  "dji_api/api_accelerate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_accelerate)))
  "Returns string type for a message object of type 'api_accelerate"
  "dji_api/api_accelerate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_accelerate>)))
  "Returns md5sum for a message object of type '<api_accelerate>"
  "03df7652de8ef3fd3e268967730bab29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_accelerate)))
  "Returns md5sum for a message object of type 'api_accelerate"
  "03df7652de8ef3fd3e268967730bab29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_accelerate>)))
  "Returns full string definition for message of type '<api_accelerate>"
  (cl:format cl:nil "float32 agx~%float32 agy~%float32 agz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_accelerate)))
  "Returns full string definition for message of type 'api_accelerate"
  (cl:format cl:nil "float32 agx~%float32 agy~%float32 agz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_accelerate>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_accelerate>))
  "Converts a ROS message object to a list"
  (cl:list 'api_accelerate
    (cl:cons ':agx (agx msg))
    (cl:cons ':agy (agy msg))
    (cl:cons ':agz (agz msg))
))

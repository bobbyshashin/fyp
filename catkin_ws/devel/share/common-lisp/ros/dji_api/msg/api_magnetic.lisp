; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_magnetic.msg.html

(cl:defclass <api_magnetic> (roslisp-msg-protocol:ros-message)
  ((mx
    :reader mx
    :initarg :mx
    :type cl:float
    :initform 0.0)
   (my
    :reader my
    :initarg :my
    :type cl:float
    :initform 0.0)
   (mz
    :reader mz
    :initarg :mz
    :type cl:float
    :initform 0.0))
)

(cl:defclass api_magnetic (<api_magnetic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_magnetic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_magnetic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_magnetic> is deprecated: use dji_api-msg:api_magnetic instead.")))

(cl:ensure-generic-function 'mx-val :lambda-list '(m))
(cl:defmethod mx-val ((m <api_magnetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:mx-val is deprecated.  Use dji_api-msg:mx instead.")
  (mx m))

(cl:ensure-generic-function 'my-val :lambda-list '(m))
(cl:defmethod my-val ((m <api_magnetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:my-val is deprecated.  Use dji_api-msg:my instead.")
  (my m))

(cl:ensure-generic-function 'mz-val :lambda-list '(m))
(cl:defmethod mz-val ((m <api_magnetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:mz-val is deprecated.  Use dji_api-msg:mz instead.")
  (mz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_magnetic>) ostream)
  "Serializes a message object of type '<api_magnetic>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'my))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_magnetic>) istream)
  "Deserializes a message object of type '<api_magnetic>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'my) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_magnetic>)))
  "Returns string type for a message object of type '<api_magnetic>"
  "dji_api/api_magnetic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_magnetic)))
  "Returns string type for a message object of type 'api_magnetic"
  "dji_api/api_magnetic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_magnetic>)))
  "Returns md5sum for a message object of type '<api_magnetic>"
  "64ac045c4cbeb58b42caa30834be56a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_magnetic)))
  "Returns md5sum for a message object of type 'api_magnetic"
  "64ac045c4cbeb58b42caa30834be56a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_magnetic>)))
  "Returns full string definition for message of type '<api_magnetic>"
  (cl:format cl:nil "float32 mx~%float32 my~%float32 mz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_magnetic)))
  "Returns full string definition for message of type 'api_magnetic"
  (cl:format cl:nil "float32 mx~%float32 my~%float32 mz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_magnetic>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_magnetic>))
  "Converts a ROS message object to a list"
  (cl:list 'api_magnetic
    (cl:cons ':mx (mx msg))
    (cl:cons ':my (my msg))
    (cl:cons ':mz (mz msg))
))

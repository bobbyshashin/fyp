; Auto-generated. Do not edit!


(cl:in-package dji_api-msg)


;//! \htmlinclude api_gps.msg.html

(cl:defclass <api_gps> (roslisp-msg-protocol:ros-message)
  ((longti
    :reader longti
    :initarg :longti
    :type cl:float
    :initform 0.0)
   (lati
    :reader lati
    :initarg :lati
    :type cl:float
    :initform 0.0)
   (alti
    :reader alti
    :initarg :alti
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (health_flag
    :reader health_flag
    :initarg :health_flag
    :type cl:fixnum
    :initform 0))
)

(cl:defclass api_gps (<api_gps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <api_gps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'api_gps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_api-msg:<api_gps> is deprecated: use dji_api-msg:api_gps instead.")))

(cl:ensure-generic-function 'longti-val :lambda-list '(m))
(cl:defmethod longti-val ((m <api_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:longti-val is deprecated.  Use dji_api-msg:longti instead.")
  (longti m))

(cl:ensure-generic-function 'lati-val :lambda-list '(m))
(cl:defmethod lati-val ((m <api_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:lati-val is deprecated.  Use dji_api-msg:lati instead.")
  (lati m))

(cl:ensure-generic-function 'alti-val :lambda-list '(m))
(cl:defmethod alti-val ((m <api_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:alti-val is deprecated.  Use dji_api-msg:alti instead.")
  (alti m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <api_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:height-val is deprecated.  Use dji_api-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'health_flag-val :lambda-list '(m))
(cl:defmethod health_flag-val ((m <api_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_api-msg:health_flag-val is deprecated.  Use dji_api-msg:health_flag instead.")
  (health_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <api_gps>) ostream)
  "Serializes a message object of type '<api_gps>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longti))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lati))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'alti))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'health_flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <api_gps>) istream)
  "Deserializes a message object of type '<api_gps>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longti) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lati) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alti) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'health_flag) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<api_gps>)))
  "Returns string type for a message object of type '<api_gps>"
  "dji_api/api_gps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'api_gps)))
  "Returns string type for a message object of type 'api_gps"
  "dji_api/api_gps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<api_gps>)))
  "Returns md5sum for a message object of type '<api_gps>"
  "782f87212da4c55448495f3a43afd880")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'api_gps)))
  "Returns md5sum for a message object of type 'api_gps"
  "782f87212da4c55448495f3a43afd880")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<api_gps>)))
  "Returns full string definition for message of type '<api_gps>"
  (cl:format cl:nil "float64 longti~%float64 lati~%float32 alti~%float32 height~%int8 health_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'api_gps)))
  "Returns full string definition for message of type 'api_gps"
  (cl:format cl:nil "float64 longti~%float64 lati~%float32 alti~%float32 height~%int8 health_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <api_gps>))
  (cl:+ 0
     8
     8
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <api_gps>))
  "Converts a ROS message object to a list"
  (cl:list 'api_gps
    (cl:cons ':longti (longti msg))
    (cl:cons ':lati (lati msg))
    (cl:cons ':alti (alti msg))
    (cl:cons ':height (height msg))
    (cl:cons ':health_flag (health_flag msg))
))

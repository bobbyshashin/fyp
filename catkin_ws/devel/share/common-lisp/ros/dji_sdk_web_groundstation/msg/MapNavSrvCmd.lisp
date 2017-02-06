; Auto-generated. Do not edit!


(cl:in-package dji_sdk_web_groundstation-msg)


;//! \htmlinclude MapNavSrvCmd.msg.html

(cl:defclass <MapNavSrvCmd> (roslisp-msg-protocol:ros-message)
  ((cmdCode
    :reader cmdCode
    :initarg :cmdCode
    :type cl:fixnum
    :initform 0)
   (tid
    :reader tid
    :initarg :tid
    :type cl:integer
    :initform 0))
)

(cl:defclass MapNavSrvCmd (<MapNavSrvCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapNavSrvCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapNavSrvCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_sdk_web_groundstation-msg:<MapNavSrvCmd> is deprecated: use dji_sdk_web_groundstation-msg:MapNavSrvCmd instead.")))

(cl:ensure-generic-function 'cmdCode-val :lambda-list '(m))
(cl:defmethod cmdCode-val ((m <MapNavSrvCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:cmdCode-val is deprecated.  Use dji_sdk_web_groundstation-msg:cmdCode instead.")
  (cmdCode m))

(cl:ensure-generic-function 'tid-val :lambda-list '(m))
(cl:defmethod tid-val ((m <MapNavSrvCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk_web_groundstation-msg:tid-val is deprecated.  Use dji_sdk_web_groundstation-msg:tid instead.")
  (tid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapNavSrvCmd>) ostream)
  "Serializes a message object of type '<MapNavSrvCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmdCode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'tid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'tid)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapNavSrvCmd>) istream)
  "Deserializes a message object of type '<MapNavSrvCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmdCode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'tid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'tid)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapNavSrvCmd>)))
  "Returns string type for a message object of type '<MapNavSrvCmd>"
  "dji_sdk_web_groundstation/MapNavSrvCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapNavSrvCmd)))
  "Returns string type for a message object of type 'MapNavSrvCmd"
  "dji_sdk_web_groundstation/MapNavSrvCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapNavSrvCmd>)))
  "Returns md5sum for a message object of type '<MapNavSrvCmd>"
  "321f9fe469695036c44374febd41879e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapNavSrvCmd)))
  "Returns md5sum for a message object of type 'MapNavSrvCmd"
  "321f9fe469695036c44374febd41879e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapNavSrvCmd>)))
  "Returns full string definition for message of type '<MapNavSrvCmd>"
  (cl:format cl:nil "#command code~%uint8 cmdCode~%#task id~%uint64 tid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapNavSrvCmd)))
  "Returns full string definition for message of type 'MapNavSrvCmd"
  (cl:format cl:nil "#command code~%uint8 cmdCode~%#task id~%uint64 tid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapNavSrvCmd>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapNavSrvCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'MapNavSrvCmd
    (cl:cons ':cmdCode (cmdCode msg))
    (cl:cons ':tid (tid msg))
))

; Auto-generated. Do not edit!


(cl:in-package n3_sdk-srv)


;//! \htmlinclude SendDataToRemoteDevice-request.msg.html

(cl:defclass <SendDataToRemoteDevice-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SendDataToRemoteDevice-request (<SendDataToRemoteDevice-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendDataToRemoteDevice-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendDataToRemoteDevice-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name n3_sdk-srv:<SendDataToRemoteDevice-request> is deprecated: use n3_sdk-srv:SendDataToRemoteDevice-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SendDataToRemoteDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader n3_sdk-srv:data-val is deprecated.  Use n3_sdk-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendDataToRemoteDevice-request>) ostream)
  "Serializes a message object of type '<SendDataToRemoteDevice-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendDataToRemoteDevice-request>) istream)
  "Deserializes a message object of type '<SendDataToRemoteDevice-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendDataToRemoteDevice-request>)))
  "Returns string type for a service object of type '<SendDataToRemoteDevice-request>"
  "n3_sdk/SendDataToRemoteDeviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendDataToRemoteDevice-request)))
  "Returns string type for a service object of type 'SendDataToRemoteDevice-request"
  "n3_sdk/SendDataToRemoteDeviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendDataToRemoteDevice-request>)))
  "Returns md5sum for a message object of type '<SendDataToRemoteDevice-request>"
  "1c25dff3462ed6a8f6df1c148c7b6a1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendDataToRemoteDevice-request)))
  "Returns md5sum for a message object of type 'SendDataToRemoteDevice-request"
  "1c25dff3462ed6a8f6df1c148c7b6a1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendDataToRemoteDevice-request>)))
  "Returns full string definition for message of type '<SendDataToRemoteDevice-request>"
  (cl:format cl:nil "~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendDataToRemoteDevice-request)))
  "Returns full string definition for message of type 'SendDataToRemoteDevice-request"
  (cl:format cl:nil "~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendDataToRemoteDevice-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendDataToRemoteDevice-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendDataToRemoteDevice-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SendDataToRemoteDevice-response.msg.html

(cl:defclass <SendDataToRemoteDevice-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SendDataToRemoteDevice-response (<SendDataToRemoteDevice-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendDataToRemoteDevice-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendDataToRemoteDevice-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name n3_sdk-srv:<SendDataToRemoteDevice-response> is deprecated: use n3_sdk-srv:SendDataToRemoteDevice-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SendDataToRemoteDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader n3_sdk-srv:result-val is deprecated.  Use n3_sdk-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendDataToRemoteDevice-response>) ostream)
  "Serializes a message object of type '<SendDataToRemoteDevice-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendDataToRemoteDevice-response>) istream)
  "Deserializes a message object of type '<SendDataToRemoteDevice-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendDataToRemoteDevice-response>)))
  "Returns string type for a service object of type '<SendDataToRemoteDevice-response>"
  "n3_sdk/SendDataToRemoteDeviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendDataToRemoteDevice-response)))
  "Returns string type for a service object of type 'SendDataToRemoteDevice-response"
  "n3_sdk/SendDataToRemoteDeviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendDataToRemoteDevice-response>)))
  "Returns md5sum for a message object of type '<SendDataToRemoteDevice-response>"
  "1c25dff3462ed6a8f6df1c148c7b6a1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendDataToRemoteDevice-response)))
  "Returns md5sum for a message object of type 'SendDataToRemoteDevice-response"
  "1c25dff3462ed6a8f6df1c148c7b6a1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendDataToRemoteDevice-response>)))
  "Returns full string definition for message of type '<SendDataToRemoteDevice-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendDataToRemoteDevice-response)))
  "Returns full string definition for message of type 'SendDataToRemoteDevice-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendDataToRemoteDevice-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendDataToRemoteDevice-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendDataToRemoteDevice-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendDataToRemoteDevice)))
  'SendDataToRemoteDevice-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendDataToRemoteDevice)))
  'SendDataToRemoteDevice-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendDataToRemoteDevice)))
  "Returns string type for a service object of type '<SendDataToRemoteDevice>"
  "n3_sdk/SendDataToRemoteDevice")
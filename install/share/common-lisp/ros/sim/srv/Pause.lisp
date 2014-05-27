; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Pause-request.msg.html

(cl:defclass <Pause-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Pause-request (<Pause-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pause-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pause-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Pause-request> is deprecated: use sim-srv:Pause-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pause-request>) ostream)
  "Serializes a message object of type '<Pause-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pause-request>) istream)
  "Deserializes a message object of type '<Pause-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pause-request>)))
  "Returns string type for a service object of type '<Pause-request>"
  "sim/PauseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pause-request)))
  "Returns string type for a service object of type 'Pause-request"
  "sim/PauseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pause-request>)))
  "Returns md5sum for a message object of type '<Pause-request>"
  "2ec6f3eff0161f4257b808b12bc830c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pause-request)))
  "Returns md5sum for a message object of type 'Pause-request"
  "2ec6f3eff0161f4257b808b12bc830c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pause-request>)))
  "Returns full string definition for message of type '<Pause-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pause-request)))
  "Returns full string definition for message of type 'Pause-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pause-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pause-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Pause-request
))
;//! \htmlinclude Pause-response.msg.html

(cl:defclass <Pause-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform ""))
)

(cl:defclass Pause-response (<Pause-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pause-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pause-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Pause-response> is deprecated: use sim-srv:Pause-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Pause-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Pause-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pause-response>) ostream)
  "Serializes a message object of type '<Pause-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pause-response>) istream)
  "Deserializes a message object of type '<Pause-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pause-response>)))
  "Returns string type for a service object of type '<Pause-response>"
  "sim/PauseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pause-response)))
  "Returns string type for a service object of type 'Pause-response"
  "sim/PauseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pause-response>)))
  "Returns md5sum for a message object of type '<Pause-response>"
  "2ec6f3eff0161f4257b808b12bc830c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pause-response)))
  "Returns md5sum for a message object of type 'Pause-response"
  "2ec6f3eff0161f4257b808b12bc830c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pause-response>)))
  "Returns full string definition for message of type '<Pause-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pause-response)))
  "Returns full string definition for message of type 'Pause-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pause-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pause-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Pause-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Pause)))
  'Pause-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Pause)))
  'Pause-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pause)))
  "Returns string type for a service object of type '<Pause>"
  "sim/Pause")
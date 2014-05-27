; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Noise-request.msg.html

(cl:defclass <Noise-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Noise-request (<Noise-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Noise-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Noise-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Noise-request> is deprecated: use sim-srv:Noise-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <Noise-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:enable-val is deprecated.  Use sim-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Noise-request>) ostream)
  "Serializes a message object of type '<Noise-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Noise-request>) istream)
  "Deserializes a message object of type '<Noise-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Noise-request>)))
  "Returns string type for a service object of type '<Noise-request>"
  "sim/NoiseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Noise-request)))
  "Returns string type for a service object of type 'Noise-request"
  "sim/NoiseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Noise-request>)))
  "Returns md5sum for a message object of type '<Noise-request>"
  "e38f6bbb98c07c037693fa4c74b95ce5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Noise-request)))
  "Returns md5sum for a message object of type 'Noise-request"
  "e38f6bbb98c07c037693fa4c74b95ce5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Noise-request>)))
  "Returns full string definition for message of type '<Noise-request>"
  (cl:format cl:nil "bool   enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Noise-request)))
  "Returns full string definition for message of type 'Noise-request"
  (cl:format cl:nil "bool   enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Noise-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Noise-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Noise-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude Noise-response.msg.html

(cl:defclass <Noise-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Noise-response (<Noise-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Noise-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Noise-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Noise-response> is deprecated: use sim-srv:Noise-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Noise-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Noise-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Noise-response>) ostream)
  "Serializes a message object of type '<Noise-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Noise-response>) istream)
  "Deserializes a message object of type '<Noise-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Noise-response>)))
  "Returns string type for a service object of type '<Noise-response>"
  "sim/NoiseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Noise-response)))
  "Returns string type for a service object of type 'Noise-response"
  "sim/NoiseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Noise-response>)))
  "Returns md5sum for a message object of type '<Noise-response>"
  "e38f6bbb98c07c037693fa4c74b95ce5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Noise-response)))
  "Returns md5sum for a message object of type 'Noise-response"
  "e38f6bbb98c07c037693fa4c74b95ce5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Noise-response>)))
  "Returns full string definition for message of type '<Noise-response>"
  (cl:format cl:nil "bool   success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Noise-response)))
  "Returns full string definition for message of type 'Noise-response"
  (cl:format cl:nil "bool   success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Noise-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Noise-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Noise-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Noise)))
  'Noise-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Noise)))
  'Noise-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Noise)))
  "Returns string type for a service object of type '<Noise>"
  "sim/Noise")
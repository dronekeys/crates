; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Step-request.msg.html

(cl:defclass <Step-request> (roslisp-msg-protocol:ros-message)
  ((num_steps
    :reader num_steps
    :initarg :num_steps
    :type cl:integer
    :initform 0))
)

(cl:defclass Step-request (<Step-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Step-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Step-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Step-request> is deprecated: use sim-srv:Step-request instead.")))

(cl:ensure-generic-function 'num_steps-val :lambda-list '(m))
(cl:defmethod num_steps-val ((m <Step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:num_steps-val is deprecated.  Use sim-srv:num_steps instead.")
  (num_steps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Step-request>) ostream)
  "Serializes a message object of type '<Step-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_steps)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Step-request>) istream)
  "Deserializes a message object of type '<Step-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_steps)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Step-request>)))
  "Returns string type for a service object of type '<Step-request>"
  "sim/StepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Step-request)))
  "Returns string type for a service object of type 'Step-request"
  "sim/StepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Step-request>)))
  "Returns md5sum for a message object of type '<Step-request>"
  "e93a400b6dd513793b97ffb4a0ce25a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Step-request)))
  "Returns md5sum for a message object of type 'Step-request"
  "e93a400b6dd513793b97ffb4a0ce25a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Step-request>)))
  "Returns full string definition for message of type '<Step-request>"
  (cl:format cl:nil "uint32 num_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Step-request)))
  "Returns full string definition for message of type 'Step-request"
  (cl:format cl:nil "uint32 num_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Step-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Step-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Step-request
    (cl:cons ':num_steps (num_steps msg))
))
;//! \htmlinclude Step-response.msg.html

(cl:defclass <Step-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Step-response (<Step-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Step-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Step-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Step-response> is deprecated: use sim-srv:Step-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Step-response>) ostream)
  "Serializes a message object of type '<Step-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Step-response>) istream)
  "Deserializes a message object of type '<Step-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Step-response>)))
  "Returns string type for a service object of type '<Step-response>"
  "sim/StepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Step-response)))
  "Returns string type for a service object of type 'Step-response"
  "sim/StepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Step-response>)))
  "Returns md5sum for a message object of type '<Step-response>"
  "e93a400b6dd513793b97ffb4a0ce25a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Step-response)))
  "Returns md5sum for a message object of type 'Step-response"
  "e93a400b6dd513793b97ffb4a0ce25a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Step-response>)))
  "Returns full string definition for message of type '<Step-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Step-response)))
  "Returns full string definition for message of type 'Step-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Step-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Step-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Step-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Step)))
  'Step-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Step)))
  'Step-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Step)))
  "Returns string type for a service object of type '<Step>"
  "sim/Step")
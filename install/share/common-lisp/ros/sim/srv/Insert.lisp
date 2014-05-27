; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Insert-request.msg.html

(cl:defclass <Insert-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (model_type
    :reader model_type
    :initarg :model_type
    :type cl:string
    :initform ""))
)

(cl:defclass Insert-request (<Insert-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Insert-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Insert-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Insert-request> is deprecated: use sim-srv:Insert-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <Insert-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:model_name-val is deprecated.  Use sim-srv:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'model_type-val :lambda-list '(m))
(cl:defmethod model_type-val ((m <Insert-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:model_type-val is deprecated.  Use sim-srv:model_type instead.")
  (model_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Insert-request>) ostream)
  "Serializes a message object of type '<Insert-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Insert-request>) istream)
  "Deserializes a message object of type '<Insert-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Insert-request>)))
  "Returns string type for a service object of type '<Insert-request>"
  "sim/InsertRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Insert-request)))
  "Returns string type for a service object of type 'Insert-request"
  "sim/InsertRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Insert-request>)))
  "Returns md5sum for a message object of type '<Insert-request>"
  "ff0b31d7564436277c8fb44cc8249578")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Insert-request)))
  "Returns md5sum for a message object of type 'Insert-request"
  "ff0b31d7564436277c8fb44cc8249578")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Insert-request>)))
  "Returns full string definition for message of type '<Insert-request>"
  (cl:format cl:nil "string model_name~%string model_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Insert-request)))
  "Returns full string definition for message of type 'Insert-request"
  (cl:format cl:nil "string model_name~%string model_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Insert-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'model_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Insert-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Insert-request
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':model_type (model_type msg))
))
;//! \htmlinclude Insert-response.msg.html

(cl:defclass <Insert-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Insert-response (<Insert-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Insert-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Insert-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Insert-response> is deprecated: use sim-srv:Insert-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Insert-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Insert-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Insert-response>) ostream)
  "Serializes a message object of type '<Insert-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Insert-response>) istream)
  "Deserializes a message object of type '<Insert-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Insert-response>)))
  "Returns string type for a service object of type '<Insert-response>"
  "sim/InsertResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Insert-response)))
  "Returns string type for a service object of type 'Insert-response"
  "sim/InsertResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Insert-response>)))
  "Returns md5sum for a message object of type '<Insert-response>"
  "ff0b31d7564436277c8fb44cc8249578")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Insert-response)))
  "Returns md5sum for a message object of type 'Insert-response"
  "ff0b31d7564436277c8fb44cc8249578")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Insert-response>)))
  "Returns full string definition for message of type '<Insert-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Insert-response)))
  "Returns full string definition for message of type 'Insert-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Insert-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Insert-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Insert-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Insert)))
  'Insert-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Insert)))
  'Insert-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Insert)))
  "Returns string type for a service object of type '<Insert>"
  "sim/Insert")
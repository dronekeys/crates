; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Delete-request.msg.html

(cl:defclass <Delete-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform ""))
)

(cl:defclass Delete-request (<Delete-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Delete-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Delete-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Delete-request> is deprecated: use sim-srv:Delete-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <Delete-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:model_name-val is deprecated.  Use sim-srv:model_name instead.")
  (model_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Delete-request>) ostream)
  "Serializes a message object of type '<Delete-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Delete-request>) istream)
  "Deserializes a message object of type '<Delete-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Delete-request>)))
  "Returns string type for a service object of type '<Delete-request>"
  "sim/DeleteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete-request)))
  "Returns string type for a service object of type 'Delete-request"
  "sim/DeleteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Delete-request>)))
  "Returns md5sum for a message object of type '<Delete-request>"
  "9ce56b4e9e54616de25d796dc972a262")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Delete-request)))
  "Returns md5sum for a message object of type 'Delete-request"
  "9ce56b4e9e54616de25d796dc972a262")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Delete-request>)))
  "Returns full string definition for message of type '<Delete-request>"
  (cl:format cl:nil "string model_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Delete-request)))
  "Returns full string definition for message of type 'Delete-request"
  (cl:format cl:nil "string model_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Delete-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Delete-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Delete-request
    (cl:cons ':model_name (model_name msg))
))
;//! \htmlinclude Delete-response.msg.html

(cl:defclass <Delete-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Delete-response (<Delete-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Delete-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Delete-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Delete-response> is deprecated: use sim-srv:Delete-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Delete-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Delete-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Delete-response>) ostream)
  "Serializes a message object of type '<Delete-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Delete-response>) istream)
  "Deserializes a message object of type '<Delete-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Delete-response>)))
  "Returns string type for a service object of type '<Delete-response>"
  "sim/DeleteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete-response)))
  "Returns string type for a service object of type 'Delete-response"
  "sim/DeleteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Delete-response>)))
  "Returns md5sum for a message object of type '<Delete-response>"
  "9ce56b4e9e54616de25d796dc972a262")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Delete-response)))
  "Returns md5sum for a message object of type 'Delete-response"
  "9ce56b4e9e54616de25d796dc972a262")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Delete-response>)))
  "Returns full string definition for message of type '<Delete-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Delete-response)))
  "Returns full string definition for message of type 'Delete-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Delete-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Delete-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Delete-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Delete)))
  'Delete-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Delete)))
  'Delete-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete)))
  "Returns string type for a service object of type '<Delete>"
  "sim/Delete")
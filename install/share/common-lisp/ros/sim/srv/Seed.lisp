; Auto-generated. Do not edit!


(cl:in-package sim-srv)


;//! \htmlinclude Seed-request.msg.html

(cl:defclass <Seed-request> (roslisp-msg-protocol:ros-message)
  ((seed
    :reader seed
    :initarg :seed
    :type cl:integer
    :initform 0))
)

(cl:defclass Seed-request (<Seed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Seed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Seed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Seed-request> is deprecated: use sim-srv:Seed-request instead.")))

(cl:ensure-generic-function 'seed-val :lambda-list '(m))
(cl:defmethod seed-val ((m <Seed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:seed-val is deprecated.  Use sim-srv:seed instead.")
  (seed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Seed-request>) ostream)
  "Serializes a message object of type '<Seed-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Seed-request>) istream)
  "Deserializes a message object of type '<Seed-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Seed-request>)))
  "Returns string type for a service object of type '<Seed-request>"
  "sim/SeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed-request)))
  "Returns string type for a service object of type 'Seed-request"
  "sim/SeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Seed-request>)))
  "Returns md5sum for a message object of type '<Seed-request>"
  "50a810a05cc873df672f01c90fb2ef4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Seed-request)))
  "Returns md5sum for a message object of type 'Seed-request"
  "50a810a05cc873df672f01c90fb2ef4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Seed-request>)))
  "Returns full string definition for message of type '<Seed-request>"
  (cl:format cl:nil "uint32 seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Seed-request)))
  "Returns full string definition for message of type 'Seed-request"
  (cl:format cl:nil "uint32 seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Seed-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Seed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Seed-request
    (cl:cons ':seed (seed msg))
))
;//! \htmlinclude Seed-response.msg.html

(cl:defclass <Seed-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Seed-response (<Seed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Seed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Seed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-srv:<Seed-response> is deprecated: use sim-srv:Seed-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Seed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:success-val is deprecated.  Use sim-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <Seed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-srv:status_message-val is deprecated.  Use sim-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Seed-response>) ostream)
  "Serializes a message object of type '<Seed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Seed-response>) istream)
  "Deserializes a message object of type '<Seed-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Seed-response>)))
  "Returns string type for a service object of type '<Seed-response>"
  "sim/SeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed-response)))
  "Returns string type for a service object of type 'Seed-response"
  "sim/SeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Seed-response>)))
  "Returns md5sum for a message object of type '<Seed-response>"
  "50a810a05cc873df672f01c90fb2ef4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Seed-response)))
  "Returns md5sum for a message object of type 'Seed-response"
  "50a810a05cc873df672f01c90fb2ef4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Seed-response>)))
  "Returns full string definition for message of type '<Seed-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Seed-response)))
  "Returns full string definition for message of type 'Seed-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Seed-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Seed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Seed-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Seed)))
  'Seed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Seed)))
  'Seed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed)))
  "Returns string type for a service object of type '<Seed>"
  "sim/Seed")
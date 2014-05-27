; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude Takeoff-request.msg.html

(cl:defclass <Takeoff-request> (roslisp-msg-protocol:ros-message)
  ((altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass Takeoff-request (<Takeoff-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Takeoff-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Takeoff-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Takeoff-request> is deprecated: use hal_quadrotor-srv:Takeoff-request instead.")))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <Takeoff-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:altitude-val is deprecated.  Use hal_quadrotor-srv:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Takeoff-request>) ostream)
  "Serializes a message object of type '<Takeoff-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Takeoff-request>) istream)
  "Deserializes a message object of type '<Takeoff-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Takeoff-request>)))
  "Returns string type for a service object of type '<Takeoff-request>"
  "hal_quadrotor/TakeoffRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Takeoff-request)))
  "Returns string type for a service object of type 'Takeoff-request"
  "hal_quadrotor/TakeoffRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Takeoff-request>)))
  "Returns md5sum for a message object of type '<Takeoff-request>"
  "fd827683f3053f974df19d13a876d4e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Takeoff-request)))
  "Returns md5sum for a message object of type 'Takeoff-request"
  "fd827683f3053f974df19d13a876d4e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Takeoff-request>)))
  "Returns full string definition for message of type '<Takeoff-request>"
  (cl:format cl:nil "~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Takeoff-request)))
  "Returns full string definition for message of type 'Takeoff-request"
  (cl:format cl:nil "~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Takeoff-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Takeoff-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Takeoff-request
    (cl:cons ':altitude (altitude msg))
))
;//! \htmlinclude Takeoff-response.msg.html

(cl:defclass <Takeoff-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass Takeoff-response (<Takeoff-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Takeoff-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Takeoff-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Takeoff-response> is deprecated: use hal_quadrotor-srv:Takeoff-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Takeoff-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Takeoff-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Takeoff-response>) ostream)
  "Serializes a message object of type '<Takeoff-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Takeoff-response>) istream)
  "Deserializes a message object of type '<Takeoff-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Takeoff-response>)))
  "Returns string type for a service object of type '<Takeoff-response>"
  "hal_quadrotor/TakeoffResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Takeoff-response)))
  "Returns string type for a service object of type 'Takeoff-response"
  "hal_quadrotor/TakeoffResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Takeoff-response>)))
  "Returns md5sum for a message object of type '<Takeoff-response>"
  "fd827683f3053f974df19d13a876d4e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Takeoff-response)))
  "Returns md5sum for a message object of type 'Takeoff-response"
  "fd827683f3053f974df19d13a876d4e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Takeoff-response>)))
  "Returns full string definition for message of type '<Takeoff-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Takeoff-response)))
  "Returns full string definition for message of type 'Takeoff-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Takeoff-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Takeoff-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Takeoff-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Takeoff)))
  'Takeoff-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Takeoff)))
  'Takeoff-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Takeoff)))
  "Returns string type for a service object of type '<Takeoff>"
  "hal_quadrotor/Takeoff")
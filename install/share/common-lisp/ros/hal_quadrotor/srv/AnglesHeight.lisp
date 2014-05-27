; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude AnglesHeight-request.msg.html

(cl:defclass <AnglesHeight-request> (roslisp-msg-protocol:ros-message)
  ((roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass AnglesHeight-request (<AnglesHeight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnglesHeight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnglesHeight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<AnglesHeight-request> is deprecated: use hal_quadrotor-srv:AnglesHeight-request instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <AnglesHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:roll-val is deprecated.  Use hal_quadrotor-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <AnglesHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:pitch-val is deprecated.  Use hal_quadrotor-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <AnglesHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:yaw-val is deprecated.  Use hal_quadrotor-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <AnglesHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:z-val is deprecated.  Use hal_quadrotor-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnglesHeight-request>) ostream)
  "Serializes a message object of type '<AnglesHeight-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnglesHeight-request>) istream)
  "Deserializes a message object of type '<AnglesHeight-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnglesHeight-request>)))
  "Returns string type for a service object of type '<AnglesHeight-request>"
  "hal_quadrotor/AnglesHeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnglesHeight-request)))
  "Returns string type for a service object of type 'AnglesHeight-request"
  "hal_quadrotor/AnglesHeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnglesHeight-request>)))
  "Returns md5sum for a message object of type '<AnglesHeight-request>"
  "f969e0cc88facd6aa5ee523f9a20adf8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnglesHeight-request)))
  "Returns md5sum for a message object of type 'AnglesHeight-request"
  "f969e0cc88facd6aa5ee523f9a20adf8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnglesHeight-request>)))
  "Returns full string definition for message of type '<AnglesHeight-request>"
  (cl:format cl:nil "~%float64 roll~%float64 pitch~%float64 yaw~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnglesHeight-request)))
  "Returns full string definition for message of type 'AnglesHeight-request"
  (cl:format cl:nil "~%float64 roll~%float64 pitch~%float64 yaw~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnglesHeight-request>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnglesHeight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnglesHeight-request
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude AnglesHeight-response.msg.html

(cl:defclass <AnglesHeight-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass AnglesHeight-response (<AnglesHeight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnglesHeight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnglesHeight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<AnglesHeight-response> is deprecated: use hal_quadrotor-srv:AnglesHeight-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AnglesHeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AnglesHeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnglesHeight-response>) ostream)
  "Serializes a message object of type '<AnglesHeight-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnglesHeight-response>) istream)
  "Deserializes a message object of type '<AnglesHeight-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnglesHeight-response>)))
  "Returns string type for a service object of type '<AnglesHeight-response>"
  "hal_quadrotor/AnglesHeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnglesHeight-response)))
  "Returns string type for a service object of type 'AnglesHeight-response"
  "hal_quadrotor/AnglesHeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnglesHeight-response>)))
  "Returns md5sum for a message object of type '<AnglesHeight-response>"
  "f969e0cc88facd6aa5ee523f9a20adf8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnglesHeight-response)))
  "Returns md5sum for a message object of type 'AnglesHeight-response"
  "f969e0cc88facd6aa5ee523f9a20adf8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnglesHeight-response>)))
  "Returns full string definition for message of type '<AnglesHeight-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnglesHeight-response)))
  "Returns full string definition for message of type 'AnglesHeight-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnglesHeight-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnglesHeight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnglesHeight-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnglesHeight)))
  'AnglesHeight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnglesHeight)))
  'AnglesHeight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnglesHeight)))
  "Returns string type for a service object of type '<AnglesHeight>"
  "hal_quadrotor/AnglesHeight")
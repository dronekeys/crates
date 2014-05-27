; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude VelocityHeight-request.msg.html

(cl:defclass <VelocityHeight-request> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass VelocityHeight-request (<VelocityHeight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelocityHeight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelocityHeight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<VelocityHeight-request> is deprecated: use hal_quadrotor-srv:VelocityHeight-request instead.")))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <VelocityHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:u-val is deprecated.  Use hal_quadrotor-srv:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <VelocityHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:v-val is deprecated.  Use hal_quadrotor-srv:v instead.")
  (v m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <VelocityHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:z-val is deprecated.  Use hal_quadrotor-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <VelocityHeight-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:yaw-val is deprecated.  Use hal_quadrotor-srv:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelocityHeight-request>) ostream)
  "Serializes a message object of type '<VelocityHeight-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v))))
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelocityHeight-request>) istream)
  "Deserializes a message object of type '<VelocityHeight-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-double-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelocityHeight-request>)))
  "Returns string type for a service object of type '<VelocityHeight-request>"
  "hal_quadrotor/VelocityHeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelocityHeight-request)))
  "Returns string type for a service object of type 'VelocityHeight-request"
  "hal_quadrotor/VelocityHeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelocityHeight-request>)))
  "Returns md5sum for a message object of type '<VelocityHeight-request>"
  "fb48d94e6b22b20ce438ebb8e6622de0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelocityHeight-request)))
  "Returns md5sum for a message object of type 'VelocityHeight-request"
  "fb48d94e6b22b20ce438ebb8e6622de0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelocityHeight-request>)))
  "Returns full string definition for message of type '<VelocityHeight-request>"
  (cl:format cl:nil "~%float64 u~%float64 v~%float64 z~%float64 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelocityHeight-request)))
  "Returns full string definition for message of type 'VelocityHeight-request"
  (cl:format cl:nil "~%float64 u~%float64 v~%float64 z~%float64 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelocityHeight-request>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelocityHeight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VelocityHeight-request
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
))
;//! \htmlinclude VelocityHeight-response.msg.html

(cl:defclass <VelocityHeight-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass VelocityHeight-response (<VelocityHeight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelocityHeight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelocityHeight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<VelocityHeight-response> is deprecated: use hal_quadrotor-srv:VelocityHeight-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VelocityHeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <VelocityHeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelocityHeight-response>) ostream)
  "Serializes a message object of type '<VelocityHeight-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelocityHeight-response>) istream)
  "Deserializes a message object of type '<VelocityHeight-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelocityHeight-response>)))
  "Returns string type for a service object of type '<VelocityHeight-response>"
  "hal_quadrotor/VelocityHeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelocityHeight-response)))
  "Returns string type for a service object of type 'VelocityHeight-response"
  "hal_quadrotor/VelocityHeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelocityHeight-response>)))
  "Returns md5sum for a message object of type '<VelocityHeight-response>"
  "fb48d94e6b22b20ce438ebb8e6622de0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelocityHeight-response)))
  "Returns md5sum for a message object of type 'VelocityHeight-response"
  "fb48d94e6b22b20ce438ebb8e6622de0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelocityHeight-response>)))
  "Returns full string definition for message of type '<VelocityHeight-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelocityHeight-response)))
  "Returns full string definition for message of type 'VelocityHeight-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelocityHeight-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelocityHeight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VelocityHeight-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VelocityHeight)))
  'VelocityHeight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VelocityHeight)))
  'VelocityHeight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelocityHeight)))
  "Returns string type for a service object of type '<VelocityHeight>"
  "hal_quadrotor/VelocityHeight")
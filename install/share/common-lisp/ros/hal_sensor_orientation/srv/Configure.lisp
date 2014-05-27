; Auto-generated. Do not edit!


(cl:in-package hal_sensor_orientation-srv)


;//! \htmlinclude Configure-request.msg.html

(cl:defclass <Configure-request> (roslisp-msg-protocol:ros-message)
  ((samprate
    :reader samprate
    :initarg :samprate
    :type cl:float
    :initform 0.0)
   (sendrate
    :reader sendrate
    :initarg :sendrate
    :type cl:float
    :initform 0.0))
)

(cl:defclass Configure-request (<Configure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Configure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Configure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_sensor_orientation-srv:<Configure-request> is deprecated: use hal_sensor_orientation-srv:Configure-request instead.")))

(cl:ensure-generic-function 'samprate-val :lambda-list '(m))
(cl:defmethod samprate-val ((m <Configure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_sensor_orientation-srv:samprate-val is deprecated.  Use hal_sensor_orientation-srv:samprate instead.")
  (samprate m))

(cl:ensure-generic-function 'sendrate-val :lambda-list '(m))
(cl:defmethod sendrate-val ((m <Configure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_sensor_orientation-srv:sendrate-val is deprecated.  Use hal_sensor_orientation-srv:sendrate instead.")
  (sendrate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Configure-request>) ostream)
  "Serializes a message object of type '<Configure-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'samprate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sendrate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Configure-request>) istream)
  "Deserializes a message object of type '<Configure-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'samprate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sendrate) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Configure-request>)))
  "Returns string type for a service object of type '<Configure-request>"
  "hal_sensor_orientation/ConfigureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Configure-request)))
  "Returns string type for a service object of type 'Configure-request"
  "hal_sensor_orientation/ConfigureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Configure-request>)))
  "Returns md5sum for a message object of type '<Configure-request>"
  "c138d47fe339d97761e69b77ee1c1cd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Configure-request)))
  "Returns md5sum for a message object of type 'Configure-request"
  "c138d47fe339d97761e69b77ee1c1cd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Configure-request>)))
  "Returns full string definition for message of type '<Configure-request>"
  (cl:format cl:nil "float64 samprate~%float64 sendrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Configure-request)))
  "Returns full string definition for message of type 'Configure-request"
  (cl:format cl:nil "float64 samprate~%float64 sendrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Configure-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Configure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Configure-request
    (cl:cons ':samprate (samprate msg))
    (cl:cons ':sendrate (sendrate msg))
))
;//! \htmlinclude Configure-response.msg.html

(cl:defclass <Configure-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Configure-response (<Configure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Configure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Configure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_sensor_orientation-srv:<Configure-response> is deprecated: use hal_sensor_orientation-srv:Configure-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Configure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_sensor_orientation-srv:success-val is deprecated.  Use hal_sensor_orientation-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Configure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_sensor_orientation-srv:status-val is deprecated.  Use hal_sensor_orientation-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Configure-response>) ostream)
  "Serializes a message object of type '<Configure-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Configure-response>) istream)
  "Deserializes a message object of type '<Configure-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Configure-response>)))
  "Returns string type for a service object of type '<Configure-response>"
  "hal_sensor_orientation/ConfigureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Configure-response)))
  "Returns string type for a service object of type 'Configure-response"
  "hal_sensor_orientation/ConfigureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Configure-response>)))
  "Returns md5sum for a message object of type '<Configure-response>"
  "c138d47fe339d97761e69b77ee1c1cd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Configure-response)))
  "Returns md5sum for a message object of type 'Configure-response"
  "c138d47fe339d97761e69b77ee1c1cd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Configure-response>)))
  "Returns full string definition for message of type '<Configure-response>"
  (cl:format cl:nil "bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Configure-response)))
  "Returns full string definition for message of type 'Configure-response"
  (cl:format cl:nil "bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Configure-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Configure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Configure-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Configure)))
  'Configure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Configure)))
  'Configure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Configure)))
  "Returns string type for a service object of type '<Configure>"
  "hal_sensor_orientation/Configure")
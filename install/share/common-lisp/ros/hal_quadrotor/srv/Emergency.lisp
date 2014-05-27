; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude Emergency-request.msg.html

(cl:defclass <Emergency-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Emergency-request (<Emergency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Emergency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Emergency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Emergency-request> is deprecated: use hal_quadrotor-srv:Emergency-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Emergency-request>) ostream)
  "Serializes a message object of type '<Emergency-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Emergency-request>) istream)
  "Deserializes a message object of type '<Emergency-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Emergency-request>)))
  "Returns string type for a service object of type '<Emergency-request>"
  "hal_quadrotor/EmergencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency-request)))
  "Returns string type for a service object of type 'Emergency-request"
  "hal_quadrotor/EmergencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Emergency-request>)))
  "Returns md5sum for a message object of type '<Emergency-request>"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Emergency-request)))
  "Returns md5sum for a message object of type 'Emergency-request"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Emergency-request>)))
  "Returns full string definition for message of type '<Emergency-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Emergency-request)))
  "Returns full string definition for message of type 'Emergency-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Emergency-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Emergency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Emergency-request
))
;//! \htmlinclude Emergency-response.msg.html

(cl:defclass <Emergency-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Emergency-response (<Emergency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Emergency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Emergency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Emergency-response> is deprecated: use hal_quadrotor-srv:Emergency-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Emergency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Emergency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Emergency-response>) ostream)
  "Serializes a message object of type '<Emergency-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Emergency-response>) istream)
  "Deserializes a message object of type '<Emergency-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Emergency-response>)))
  "Returns string type for a service object of type '<Emergency-response>"
  "hal_quadrotor/EmergencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency-response)))
  "Returns string type for a service object of type 'Emergency-response"
  "hal_quadrotor/EmergencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Emergency-response>)))
  "Returns md5sum for a message object of type '<Emergency-response>"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Emergency-response)))
  "Returns md5sum for a message object of type 'Emergency-response"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Emergency-response>)))
  "Returns full string definition for message of type '<Emergency-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Emergency-response)))
  "Returns full string definition for message of type 'Emergency-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Emergency-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Emergency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Emergency-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Emergency)))
  'Emergency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Emergency)))
  'Emergency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency)))
  "Returns string type for a service object of type '<Emergency>"
  "hal_quadrotor/Emergency")
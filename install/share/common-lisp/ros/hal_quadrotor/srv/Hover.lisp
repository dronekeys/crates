; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude Hover-request.msg.html

(cl:defclass <Hover-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Hover-request (<Hover-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hover-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hover-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Hover-request> is deprecated: use hal_quadrotor-srv:Hover-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hover-request>) ostream)
  "Serializes a message object of type '<Hover-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hover-request>) istream)
  "Deserializes a message object of type '<Hover-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hover-request>)))
  "Returns string type for a service object of type '<Hover-request>"
  "hal_quadrotor/HoverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hover-request)))
  "Returns string type for a service object of type 'Hover-request"
  "hal_quadrotor/HoverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hover-request>)))
  "Returns md5sum for a message object of type '<Hover-request>"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hover-request)))
  "Returns md5sum for a message object of type 'Hover-request"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hover-request>)))
  "Returns full string definition for message of type '<Hover-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hover-request)))
  "Returns full string definition for message of type 'Hover-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hover-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hover-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Hover-request
))
;//! \htmlinclude Hover-response.msg.html

(cl:defclass <Hover-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Hover-response (<Hover-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hover-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hover-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<Hover-response> is deprecated: use hal_quadrotor-srv:Hover-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Hover-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Hover-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hover-response>) ostream)
  "Serializes a message object of type '<Hover-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hover-response>) istream)
  "Deserializes a message object of type '<Hover-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hover-response>)))
  "Returns string type for a service object of type '<Hover-response>"
  "hal_quadrotor/HoverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hover-response)))
  "Returns string type for a service object of type 'Hover-response"
  "hal_quadrotor/HoverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hover-response>)))
  "Returns md5sum for a message object of type '<Hover-response>"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hover-response)))
  "Returns md5sum for a message object of type 'Hover-response"
  "38b8954d32a849f31d78416b12bff5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hover-response>)))
  "Returns full string definition for message of type '<Hover-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hover-response)))
  "Returns full string definition for message of type 'Hover-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hover-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hover-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Hover-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Hover)))
  'Hover-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Hover)))
  'Hover-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hover)))
  "Returns string type for a service object of type '<Hover>"
  "hal_quadrotor/Hover")
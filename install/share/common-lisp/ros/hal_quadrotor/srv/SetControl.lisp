; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude SetControl-request.msg.html

(cl:defclass <SetControl-request> (roslisp-msg-protocol:ros-message)
  ((control
    :reader control
    :initarg :control
    :type hal_quadrotor-msg:Control
    :initform (cl:make-instance 'hal_quadrotor-msg:Control)))
)

(cl:defclass SetControl-request (<SetControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<SetControl-request> is deprecated: use hal_quadrotor-srv:SetControl-request instead.")))

(cl:ensure-generic-function 'control-val :lambda-list '(m))
(cl:defmethod control-val ((m <SetControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:control-val is deprecated.  Use hal_quadrotor-srv:control instead.")
  (control m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControl-request>) ostream)
  "Serializes a message object of type '<SetControl-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'control) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControl-request>) istream)
  "Deserializes a message object of type '<SetControl-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'control) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControl-request>)))
  "Returns string type for a service object of type '<SetControl-request>"
  "hal_quadrotor/SetControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControl-request)))
  "Returns string type for a service object of type 'SetControl-request"
  "hal_quadrotor/SetControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControl-request>)))
  "Returns md5sum for a message object of type '<SetControl-request>"
  "9768c483ed82dc040e3bed71672eb4ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControl-request)))
  "Returns md5sum for a message object of type 'SetControl-request"
  "9768c483ed82dc040e3bed71672eb4ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControl-request>)))
  "Returns full string definition for message of type '<SetControl-request>"
  (cl:format cl:nil "hal_quadrotor/Control control~%~%================================================================================~%MSG: hal_quadrotor/Control~%float64 t	    	# Time stamp~%float64 roll     	# Body-frame X ROLL~%float64 pitch     	# Body-frame Y PITCH~%float64 yaw     	# Body-frame Z YAW~%float64 throttle    # Body-frame THROTTLE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControl-request)))
  "Returns full string definition for message of type 'SetControl-request"
  (cl:format cl:nil "hal_quadrotor/Control control~%~%================================================================================~%MSG: hal_quadrotor/Control~%float64 t	    	# Time stamp~%float64 roll     	# Body-frame X ROLL~%float64 pitch     	# Body-frame Y PITCH~%float64 yaw     	# Body-frame Z YAW~%float64 throttle    # Body-frame THROTTLE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControl-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'control))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControl-request
    (cl:cons ':control (control msg))
))
;//! \htmlinclude SetControl-response.msg.html

(cl:defclass <SetControl-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetControl-response (<SetControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<SetControl-response> is deprecated: use hal_quadrotor-srv:SetControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControl-response>) ostream)
  "Serializes a message object of type '<SetControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControl-response>) istream)
  "Deserializes a message object of type '<SetControl-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControl-response>)))
  "Returns string type for a service object of type '<SetControl-response>"
  "hal_quadrotor/SetControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControl-response)))
  "Returns string type for a service object of type 'SetControl-response"
  "hal_quadrotor/SetControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControl-response>)))
  "Returns md5sum for a message object of type '<SetControl-response>"
  "9768c483ed82dc040e3bed71672eb4ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControl-response)))
  "Returns md5sum for a message object of type 'SetControl-response"
  "9768c483ed82dc040e3bed71672eb4ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControl-response>)))
  "Returns full string definition for message of type '<SetControl-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControl-response)))
  "Returns full string definition for message of type 'SetControl-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControl-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControl-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControl)))
  'SetControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControl)))
  'SetControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControl)))
  "Returns string type for a service object of type '<SetControl>"
  "hal_quadrotor/SetControl")
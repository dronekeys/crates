; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude GetControl-request.msg.html

(cl:defclass <GetControl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetControl-request (<GetControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetControl-request> is deprecated: use hal_quadrotor-srv:GetControl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControl-request>) ostream)
  "Serializes a message object of type '<GetControl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControl-request>) istream)
  "Deserializes a message object of type '<GetControl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControl-request>)))
  "Returns string type for a service object of type '<GetControl-request>"
  "hal_quadrotor/GetControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControl-request)))
  "Returns string type for a service object of type 'GetControl-request"
  "hal_quadrotor/GetControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControl-request>)))
  "Returns md5sum for a message object of type '<GetControl-request>"
  "a6439669a77b695ef3067ee69a1face7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControl-request)))
  "Returns md5sum for a message object of type 'GetControl-request"
  "a6439669a77b695ef3067ee69a1face7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControl-request>)))
  "Returns full string definition for message of type '<GetControl-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControl-request)))
  "Returns full string definition for message of type 'GetControl-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControl-request
))
;//! \htmlinclude GetControl-response.msg.html

(cl:defclass <GetControl-response> (roslisp-msg-protocol:ros-message)
  ((control
    :reader control
    :initarg :control
    :type hal_quadrotor-msg:Control
    :initform (cl:make-instance 'hal_quadrotor-msg:Control)))
)

(cl:defclass GetControl-response (<GetControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetControl-response> is deprecated: use hal_quadrotor-srv:GetControl-response instead.")))

(cl:ensure-generic-function 'control-val :lambda-list '(m))
(cl:defmethod control-val ((m <GetControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:control-val is deprecated.  Use hal_quadrotor-srv:control instead.")
  (control m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControl-response>) ostream)
  "Serializes a message object of type '<GetControl-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'control) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControl-response>) istream)
  "Deserializes a message object of type '<GetControl-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'control) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControl-response>)))
  "Returns string type for a service object of type '<GetControl-response>"
  "hal_quadrotor/GetControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControl-response)))
  "Returns string type for a service object of type 'GetControl-response"
  "hal_quadrotor/GetControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControl-response>)))
  "Returns md5sum for a message object of type '<GetControl-response>"
  "a6439669a77b695ef3067ee69a1face7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControl-response)))
  "Returns md5sum for a message object of type 'GetControl-response"
  "a6439669a77b695ef3067ee69a1face7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControl-response>)))
  "Returns full string definition for message of type '<GetControl-response>"
  (cl:format cl:nil "hal_quadrotor/Control control~%~%================================================================================~%MSG: hal_quadrotor/Control~%float64 t	    	# Time stamp~%float64 roll     	# Body-frame X ROLL~%float64 pitch     	# Body-frame Y PITCH~%float64 yaw     	# Body-frame Z YAW~%float64 throttle    # Body-frame THROTTLE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControl-response)))
  "Returns full string definition for message of type 'GetControl-response"
  (cl:format cl:nil "hal_quadrotor/Control control~%~%================================================================================~%MSG: hal_quadrotor/Control~%float64 t	    	# Time stamp~%float64 roll     	# Body-frame X ROLL~%float64 pitch     	# Body-frame Y PITCH~%float64 yaw     	# Body-frame Z YAW~%float64 throttle    # Body-frame THROTTLE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControl-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'control))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControl-response
    (cl:cons ':control (control msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControl)))
  'GetControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControl)))
  'GetControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControl)))
  "Returns string type for a service object of type '<GetControl>"
  "hal_quadrotor/GetControl")
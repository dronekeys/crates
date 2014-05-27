; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude GetEstimate-request.msg.html

(cl:defclass <GetEstimate-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetEstimate-request (<GetEstimate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEstimate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEstimate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetEstimate-request> is deprecated: use hal_quadrotor-srv:GetEstimate-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEstimate-request>) ostream)
  "Serializes a message object of type '<GetEstimate-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEstimate-request>) istream)
  "Deserializes a message object of type '<GetEstimate-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEstimate-request>)))
  "Returns string type for a service object of type '<GetEstimate-request>"
  "hal_quadrotor/GetEstimateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEstimate-request)))
  "Returns string type for a service object of type 'GetEstimate-request"
  "hal_quadrotor/GetEstimateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEstimate-request>)))
  "Returns md5sum for a message object of type '<GetEstimate-request>"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEstimate-request)))
  "Returns md5sum for a message object of type 'GetEstimate-request"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEstimate-request>)))
  "Returns full string definition for message of type '<GetEstimate-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEstimate-request)))
  "Returns full string definition for message of type 'GetEstimate-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEstimate-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEstimate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEstimate-request
))
;//! \htmlinclude GetEstimate-response.msg.html

(cl:defclass <GetEstimate-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type hal_quadrotor-msg:State
    :initform (cl:make-instance 'hal_quadrotor-msg:State)))
)

(cl:defclass GetEstimate-response (<GetEstimate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEstimate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEstimate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetEstimate-response> is deprecated: use hal_quadrotor-srv:GetEstimate-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetEstimate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:state-val is deprecated.  Use hal_quadrotor-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEstimate-response>) ostream)
  "Serializes a message object of type '<GetEstimate-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEstimate-response>) istream)
  "Deserializes a message object of type '<GetEstimate-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEstimate-response>)))
  "Returns string type for a service object of type '<GetEstimate-response>"
  "hal_quadrotor/GetEstimateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEstimate-response)))
  "Returns string type for a service object of type 'GetEstimate-response"
  "hal_quadrotor/GetEstimateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEstimate-response>)))
  "Returns md5sum for a message object of type '<GetEstimate-response>"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEstimate-response)))
  "Returns md5sum for a message object of type 'GetEstimate-response"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEstimate-response>)))
  "Returns full string definition for message of type '<GetEstimate-response>"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEstimate-response)))
  "Returns full string definition for message of type 'GetEstimate-response"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEstimate-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEstimate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEstimate-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetEstimate)))
  'GetEstimate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetEstimate)))
  'GetEstimate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEstimate)))
  "Returns string type for a service object of type '<GetEstimate>"
  "hal_quadrotor/GetEstimate")
; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude GetTruth-request.msg.html

(cl:defclass <GetTruth-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetTruth-request (<GetTruth-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTruth-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTruth-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetTruth-request> is deprecated: use hal_quadrotor-srv:GetTruth-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTruth-request>) ostream)
  "Serializes a message object of type '<GetTruth-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTruth-request>) istream)
  "Deserializes a message object of type '<GetTruth-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTruth-request>)))
  "Returns string type for a service object of type '<GetTruth-request>"
  "hal_quadrotor/GetTruthRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTruth-request)))
  "Returns string type for a service object of type 'GetTruth-request"
  "hal_quadrotor/GetTruthRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTruth-request>)))
  "Returns md5sum for a message object of type '<GetTruth-request>"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTruth-request)))
  "Returns md5sum for a message object of type 'GetTruth-request"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTruth-request>)))
  "Returns full string definition for message of type '<GetTruth-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTruth-request)))
  "Returns full string definition for message of type 'GetTruth-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTruth-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTruth-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTruth-request
))
;//! \htmlinclude GetTruth-response.msg.html

(cl:defclass <GetTruth-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type hal_quadrotor-msg:State
    :initform (cl:make-instance 'hal_quadrotor-msg:State)))
)

(cl:defclass GetTruth-response (<GetTruth-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTruth-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTruth-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<GetTruth-response> is deprecated: use hal_quadrotor-srv:GetTruth-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetTruth-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:state-val is deprecated.  Use hal_quadrotor-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTruth-response>) ostream)
  "Serializes a message object of type '<GetTruth-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTruth-response>) istream)
  "Deserializes a message object of type '<GetTruth-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTruth-response>)))
  "Returns string type for a service object of type '<GetTruth-response>"
  "hal_quadrotor/GetTruthResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTruth-response)))
  "Returns string type for a service object of type 'GetTruth-response"
  "hal_quadrotor/GetTruthResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTruth-response>)))
  "Returns md5sum for a message object of type '<GetTruth-response>"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTruth-response)))
  "Returns md5sum for a message object of type 'GetTruth-response"
  "f2033d5defdcc91d37ab097f3c9ec53b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTruth-response>)))
  "Returns full string definition for message of type '<GetTruth-response>"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTruth-response)))
  "Returns full string definition for message of type 'GetTruth-response"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTruth-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTruth-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTruth-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTruth)))
  'GetTruth-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTruth)))
  'GetTruth-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTruth)))
  "Returns string type for a service object of type '<GetTruth>"
  "hal_quadrotor/GetTruth")
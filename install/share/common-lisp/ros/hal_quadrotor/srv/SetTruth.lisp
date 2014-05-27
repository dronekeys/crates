; Auto-generated. Do not edit!


(cl:in-package hal_quadrotor-srv)


;//! \htmlinclude SetTruth-request.msg.html

(cl:defclass <SetTruth-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type hal_quadrotor-msg:State
    :initform (cl:make-instance 'hal_quadrotor-msg:State)))
)

(cl:defclass SetTruth-request (<SetTruth-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTruth-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTruth-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<SetTruth-request> is deprecated: use hal_quadrotor-srv:SetTruth-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetTruth-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:state-val is deprecated.  Use hal_quadrotor-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTruth-request>) ostream)
  "Serializes a message object of type '<SetTruth-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTruth-request>) istream)
  "Deserializes a message object of type '<SetTruth-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTruth-request>)))
  "Returns string type for a service object of type '<SetTruth-request>"
  "hal_quadrotor/SetTruthRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTruth-request)))
  "Returns string type for a service object of type 'SetTruth-request"
  "hal_quadrotor/SetTruthRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTruth-request>)))
  "Returns md5sum for a message object of type '<SetTruth-request>"
  "e5b0cff35b7ba6b3b27ca50b371e7ab2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTruth-request)))
  "Returns md5sum for a message object of type 'SetTruth-request"
  "e5b0cff35b7ba6b3b27ca50b371e7ab2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTruth-request>)))
  "Returns full string definition for message of type '<SetTruth-request>"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTruth-request)))
  "Returns full string definition for message of type 'SetTruth-request"
  (cl:format cl:nil "hal_quadrotor/State state~%~%================================================================================~%MSG: hal_quadrotor/State~%float64 t	    	# Time stamp~%float64 x	    	# n-frame X position (X == +East)~%float64 y	   		# n-frame Y position (Y == +North)~%float64 z	    	# n-frame Z position (Z == +Up)~%float64 roll	    # n-frame roll (anti-clockwise about X)~%float64 pitch	    # n-frame pitch (anti-clockwise about Y)~%float64 yaw	    	# n-frame yaw (anti-clockwise about Z)~%float64 u	    	# b-frame X velocity~%float64 v	    	# b-frame Y velocity~%float64 w	    	# b-frame Z velocity~%float64 p	    	# b-frame roll angular velocity~%float64 q	    	# b-frame pitch angular velocity~%float64 r	    	# b-frame yaw angular velocity~%float64 thrust	    # Current thrust force~%float64 remaining	# Flight time remaining~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTruth-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTruth-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTruth-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetTruth-response.msg.html

(cl:defclass <SetTruth-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetTruth-response (<SetTruth-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTruth-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTruth-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hal_quadrotor-srv:<SetTruth-response> is deprecated: use hal_quadrotor-srv:SetTruth-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetTruth-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:success-val is deprecated.  Use hal_quadrotor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetTruth-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hal_quadrotor-srv:status-val is deprecated.  Use hal_quadrotor-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTruth-response>) ostream)
  "Serializes a message object of type '<SetTruth-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTruth-response>) istream)
  "Deserializes a message object of type '<SetTruth-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTruth-response>)))
  "Returns string type for a service object of type '<SetTruth-response>"
  "hal_quadrotor/SetTruthResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTruth-response)))
  "Returns string type for a service object of type 'SetTruth-response"
  "hal_quadrotor/SetTruthResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTruth-response>)))
  "Returns md5sum for a message object of type '<SetTruth-response>"
  "e5b0cff35b7ba6b3b27ca50b371e7ab2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTruth-response)))
  "Returns md5sum for a message object of type 'SetTruth-response"
  "e5b0cff35b7ba6b3b27ca50b371e7ab2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTruth-response>)))
  "Returns full string definition for message of type '<SetTruth-response>"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTruth-response)))
  "Returns full string definition for message of type 'SetTruth-response"
  (cl:format cl:nil "~%bool    success~%string  status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTruth-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTruth-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTruth-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTruth)))
  'SetTruth-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTruth)))
  'SetTruth-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTruth)))
  "Returns string type for a service object of type '<SetTruth>"
  "hal_quadrotor/SetTruth")
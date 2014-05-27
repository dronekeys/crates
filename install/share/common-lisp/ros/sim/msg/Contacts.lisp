; Auto-generated. Do not edit!


(cl:in-package sim-msg)


;//! \htmlinclude Contacts.msg.html

(cl:defclass <Contacts> (roslisp-msg-protocol:ros-message)
  ((contacts
    :reader contacts
    :initarg :contacts
    :type (cl:vector sim-msg:Contact)
   :initform (cl:make-array 0 :element-type 'sim-msg:Contact :initial-element (cl:make-instance 'sim-msg:Contact))))
)

(cl:defclass Contacts (<Contacts>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contacts>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contacts)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-msg:<Contacts> is deprecated: use sim-msg:Contacts instead.")))

(cl:ensure-generic-function 'contacts-val :lambda-list '(m))
(cl:defmethod contacts-val ((m <Contacts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-msg:contacts-val is deprecated.  Use sim-msg:contacts instead.")
  (contacts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contacts>) ostream)
  "Serializes a message object of type '<Contacts>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'contacts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'contacts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contacts>) istream)
  "Deserializes a message object of type '<Contacts>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'contacts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'contacts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sim-msg:Contact))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contacts>)))
  "Returns string type for a message object of type '<Contacts>"
  "sim/Contacts")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contacts)))
  "Returns string type for a message object of type 'Contacts"
  "sim/Contacts")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contacts>)))
  "Returns md5sum for a message object of type '<Contacts>"
  "f43c7ed3affe8cafbc29979c5aa792b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contacts)))
  "Returns md5sum for a message object of type 'Contacts"
  "f43c7ed3affe8cafbc29979c5aa792b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contacts>)))
  "Returns full string definition for message of type '<Contacts>"
  (cl:format cl:nil "sim/Contact[] contacts~%~%================================================================================~%MSG: sim/Contact~%string name1~%string name2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contacts)))
  "Returns full string definition for message of type 'Contacts"
  (cl:format cl:nil "sim/Contact[] contacts~%~%================================================================================~%MSG: sim/Contact~%string name1~%string name2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contacts>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'contacts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contacts>))
  "Converts a ROS message object to a list"
  (cl:list 'Contacts
    (cl:cons ':contacts (contacts msg))
))

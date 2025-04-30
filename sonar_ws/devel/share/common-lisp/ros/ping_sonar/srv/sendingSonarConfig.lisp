; Auto-generated. Do not edit!


(cl:in-package ping_sonar-srv)


;//! \htmlinclude sendingSonarConfig-request.msg.html

(cl:defclass <sendingSonarConfig-request> (roslisp-msg-protocol:ros-message)
  ((stepSize
    :reader stepSize
    :initarg :stepSize
    :type cl:integer
    :initform 0)
   (range
    :reader range
    :initarg :range
    :type cl:integer
    :initform 0))
)

(cl:defclass sendingSonarConfig-request (<sendingSonarConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sendingSonarConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sendingSonarConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ping_sonar-srv:<sendingSonarConfig-request> is deprecated: use ping_sonar-srv:sendingSonarConfig-request instead.")))

(cl:ensure-generic-function 'stepSize-val :lambda-list '(m))
(cl:defmethod stepSize-val ((m <sendingSonarConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_sonar-srv:stepSize-val is deprecated.  Use ping_sonar-srv:stepSize instead.")
  (stepSize m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <sendingSonarConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_sonar-srv:range-val is deprecated.  Use ping_sonar-srv:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sendingSonarConfig-request>) ostream)
  "Serializes a message object of type '<sendingSonarConfig-request>"
  (cl:let* ((signed (cl:slot-value msg 'stepSize)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'range)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sendingSonarConfig-request>) istream)
  "Deserializes a message object of type '<sendingSonarConfig-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stepSize) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'range) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sendingSonarConfig-request>)))
  "Returns string type for a service object of type '<sendingSonarConfig-request>"
  "ping_sonar/sendingSonarConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendingSonarConfig-request)))
  "Returns string type for a service object of type 'sendingSonarConfig-request"
  "ping_sonar/sendingSonarConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sendingSonarConfig-request>)))
  "Returns md5sum for a message object of type '<sendingSonarConfig-request>"
  "6cb2431d101e8b181456b4863d041954")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sendingSonarConfig-request)))
  "Returns md5sum for a message object of type 'sendingSonarConfig-request"
  "6cb2431d101e8b181456b4863d041954")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sendingSonarConfig-request>)))
  "Returns full string definition for message of type '<sendingSonarConfig-request>"
  (cl:format cl:nil "int64 stepSize~%int64 range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sendingSonarConfig-request)))
  "Returns full string definition for message of type 'sendingSonarConfig-request"
  (cl:format cl:nil "int64 stepSize~%int64 range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sendingSonarConfig-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sendingSonarConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'sendingSonarConfig-request
    (cl:cons ':stepSize (stepSize msg))
    (cl:cons ':range (range msg))
))
;//! \htmlinclude sendingSonarConfig-response.msg.html

(cl:defclass <sendingSonarConfig-response> (roslisp-msg-protocol:ros-message)
  ((worked
    :reader worked
    :initarg :worked
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass sendingSonarConfig-response (<sendingSonarConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sendingSonarConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sendingSonarConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ping_sonar-srv:<sendingSonarConfig-response> is deprecated: use ping_sonar-srv:sendingSonarConfig-response instead.")))

(cl:ensure-generic-function 'worked-val :lambda-list '(m))
(cl:defmethod worked-val ((m <sendingSonarConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_sonar-srv:worked-val is deprecated.  Use ping_sonar-srv:worked instead.")
  (worked m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sendingSonarConfig-response>) ostream)
  "Serializes a message object of type '<sendingSonarConfig-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'worked) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sendingSonarConfig-response>) istream)
  "Deserializes a message object of type '<sendingSonarConfig-response>"
    (cl:setf (cl:slot-value msg 'worked) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sendingSonarConfig-response>)))
  "Returns string type for a service object of type '<sendingSonarConfig-response>"
  "ping_sonar/sendingSonarConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendingSonarConfig-response)))
  "Returns string type for a service object of type 'sendingSonarConfig-response"
  "ping_sonar/sendingSonarConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sendingSonarConfig-response>)))
  "Returns md5sum for a message object of type '<sendingSonarConfig-response>"
  "6cb2431d101e8b181456b4863d041954")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sendingSonarConfig-response)))
  "Returns md5sum for a message object of type 'sendingSonarConfig-response"
  "6cb2431d101e8b181456b4863d041954")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sendingSonarConfig-response>)))
  "Returns full string definition for message of type '<sendingSonarConfig-response>"
  (cl:format cl:nil "bool worked~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sendingSonarConfig-response)))
  "Returns full string definition for message of type 'sendingSonarConfig-response"
  (cl:format cl:nil "bool worked~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sendingSonarConfig-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sendingSonarConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'sendingSonarConfig-response
    (cl:cons ':worked (worked msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'sendingSonarConfig)))
  'sendingSonarConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'sendingSonarConfig)))
  'sendingSonarConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendingSonarConfig)))
  "Returns string type for a service object of type '<sendingSonarConfig>"
  "ping_sonar/sendingSonarConfig")
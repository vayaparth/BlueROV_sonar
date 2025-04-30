; Auto-generated. Do not edit!


(cl:in-package ping_sonar-msg)


;//! \htmlinclude SonarRange.msg.html

(cl:defclass <SonarRange> (roslisp-msg-protocol:ros-message)
  ((sonar_range
    :reader sonar_range
    :initarg :sonar_range
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SonarRange (<SonarRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SonarRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SonarRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ping_sonar-msg:<SonarRange> is deprecated: use ping_sonar-msg:SonarRange instead.")))

(cl:ensure-generic-function 'sonar_range-val :lambda-list '(m))
(cl:defmethod sonar_range-val ((m <SonarRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_sonar-msg:sonar_range-val is deprecated.  Use ping_sonar-msg:sonar_range instead.")
  (sonar_range m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <SonarRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_sonar-msg:angle-val is deprecated.  Use ping_sonar-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SonarRange>) ostream)
  "Serializes a message object of type '<SonarRange>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sonar_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'sonar_range))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'angle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SonarRange>) istream)
  "Deserializes a message object of type '<SonarRange>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sonar_range) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sonar_range)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angle)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SonarRange>)))
  "Returns string type for a message object of type '<SonarRange>"
  "ping_sonar/SonarRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SonarRange)))
  "Returns string type for a message object of type 'SonarRange"
  "ping_sonar/SonarRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SonarRange>)))
  "Returns md5sum for a message object of type '<SonarRange>"
  "635d4ca17ad4fed19b4f5bcdfb57f952")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SonarRange)))
  "Returns md5sum for a message object of type 'SonarRange"
  "635d4ca17ad4fed19b4f5bcdfb57f952")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SonarRange>)))
  "Returns full string definition for message of type '<SonarRange>"
  (cl:format cl:nil "float64[] sonar_range # len: 360/step_size, 0->front~%float64[] angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SonarRange)))
  "Returns full string definition for message of type 'SonarRange"
  (cl:format cl:nil "float64[] sonar_range # len: 360/step_size, 0->front~%float64[] angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SonarRange>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sonar_range) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SonarRange>))
  "Converts a ROS message object to a list"
  (cl:list 'SonarRange
    (cl:cons ':sonar_range (sonar_range msg))
    (cl:cons ':angle (angle msg))
))

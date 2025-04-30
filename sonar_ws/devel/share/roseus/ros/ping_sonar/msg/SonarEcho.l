;; Auto-generated. Do not edit!


(when (boundp 'ping_sonar::SonarEcho)
  (if (not (find-package "PING_SONAR"))
    (make-package "PING_SONAR"))
  (shadow 'SonarEcho (find-package "PING_SONAR")))
(unless (find-package "PING_SONAR::SONARECHO")
  (make-package "PING_SONAR::SONARECHO"))

(in-package "ROS")
;;//! \htmlinclude SonarEcho.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ping_sonar::SonarEcho
  :super ros::object
  :slots (_header _angle _gain _number_of_samples _transmit_frequency _speed_of_sound _range _step_size _intensities ))

(defmethod ping_sonar::SonarEcho
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    ((:gain __gain) 0)
    ((:number_of_samples __number_of_samples) 0)
    ((:transmit_frequency __transmit_frequency) 0)
    ((:speed_of_sound __speed_of_sound) 0)
    ((:range __range) 0)
    ((:step_size __step_size) 0)
    ((:intensities __intensities) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   (setq _gain (round __gain))
   (setq _number_of_samples (round __number_of_samples))
   (setq _transmit_frequency (round __transmit_frequency))
   (setq _speed_of_sound (round __speed_of_sound))
   (setq _range (round __range))
   (setq _step_size (round __step_size))
   (setq _intensities __intensities)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:number_of_samples
   (&optional __number_of_samples)
   (if __number_of_samples (setq _number_of_samples __number_of_samples)) _number_of_samples)
  (:transmit_frequency
   (&optional __transmit_frequency)
   (if __transmit_frequency (setq _transmit_frequency __transmit_frequency)) _transmit_frequency)
  (:speed_of_sound
   (&optional __speed_of_sound)
   (if __speed_of_sound (setq _speed_of_sound __speed_of_sound)) _speed_of_sound)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:step_size
   (&optional __step_size)
   (if __step_size (setq _step_size __step_size)) _step_size)
  (:intensities
   (&optional __intensities)
   (if __intensities (setq _intensities __intensities)) _intensities)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _angle
    4
    ;; uint8 _gain
    1
    ;; uint16 _number_of_samples
    2
    ;; uint16 _transmit_frequency
    2
    ;; uint16 _speed_of_sound
    2
    ;; uint8 _range
    1
    ;; uint8 _step_size
    1
    ;; uint8[] _intensities
    (* 1    (length _intensities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _gain
       (write-byte _gain s)
     ;; uint16 _number_of_samples
       (write-word _number_of_samples s)
     ;; uint16 _transmit_frequency
       (write-word _transmit_frequency s)
     ;; uint16 _speed_of_sound
       (write-word _speed_of_sound s)
     ;; uint8 _range
       (write-byte _range s)
     ;; uint8 _step_size
       (write-byte _step_size s)
     ;; uint8[] _intensities
     (write-long (length _intensities) s)
     (princ _intensities s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _gain
     (setq _gain (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _number_of_samples
     (setq _number_of_samples (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _transmit_frequency
     (setq _transmit_frequency (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _speed_of_sound
     (setq _speed_of_sound (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _range
     (setq _range (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _step_size
     (setq _step_size (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _intensities
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _intensities (make-array n :element-type :char))
     (replace _intensities buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ping_sonar::SonarEcho :md5sum-) "6a12e1e3318526964d851d44262d11cc")
(setf (get ping_sonar::SonarEcho :datatype-) "ping_sonar/SonarEcho")
(setf (get ping_sonar::SonarEcho :definition-)
      "Header header            #header info
float32 angle               # the measurement angle [rad]
uint8 gain  # Sonar Gain
uint16 number_of_samples 
uint16 transmit_frequency # [kHz]
uint16 speed_of_sound # [m/s]
uint8 range      #  range value [m]
uint8 step_size  #  steps for each measurement(from 1 to 10)
uint8[] intensities    # intensity data [0-255].  This is the actual data received from the sonar
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :ping_sonar/SonarEcho "6a12e1e3318526964d851d44262d11cc")



;; Auto-generated. Do not edit!


(when (boundp 'ping_sonar::SonarRange)
  (if (not (find-package "PING_SONAR"))
    (make-package "PING_SONAR"))
  (shadow 'SonarRange (find-package "PING_SONAR")))
(unless (find-package "PING_SONAR::SONARRANGE")
  (make-package "PING_SONAR::SONARRANGE"))

(in-package "ROS")
;;//! \htmlinclude SonarRange.msg.html


(defclass ping_sonar::SonarRange
  :super ros::object
  :slots (_sonar_range _angle ))

(defmethod ping_sonar::SonarRange
  (:init
   (&key
    ((:sonar_range __sonar_range) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _sonar_range __sonar_range)
   (setq _angle __angle)
   self)
  (:sonar_range
   (&optional __sonar_range)
   (if __sonar_range (setq _sonar_range __sonar_range)) _sonar_range)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64[] _sonar_range
    (* 8    (length _sonar_range)) 4
    ;; float64[] _angle
    (* 8    (length _angle)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _sonar_range
     (write-long (length _sonar_range) s)
     (dotimes (i (length _sonar_range))
       (sys::poke (elt _sonar_range i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _angle
     (write-long (length _angle) s)
     (dotimes (i (length _angle))
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _sonar_range
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sonar_range (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _sonar_range i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get ping_sonar::SonarRange :md5sum-) "635d4ca17ad4fed19b4f5bcdfb57f952")
(setf (get ping_sonar::SonarRange :datatype-) "ping_sonar/SonarRange")
(setf (get ping_sonar::SonarRange :definition-)
      "float64[] sonar_range # len: 360/step_size, 0->front
float64[] angle
")



(provide :ping_sonar/SonarRange "635d4ca17ad4fed19b4f5bcdfb57f952")



;; Auto-generated. Do not edit!


(when (boundp 'ping_sonar::sendingSonarConfig)
  (if (not (find-package "PING_SONAR"))
    (make-package "PING_SONAR"))
  (shadow 'sendingSonarConfig (find-package "PING_SONAR")))
(unless (find-package "PING_SONAR::SENDINGSONARCONFIG")
  (make-package "PING_SONAR::SENDINGSONARCONFIG"))
(unless (find-package "PING_SONAR::SENDINGSONARCONFIGREQUEST")
  (make-package "PING_SONAR::SENDINGSONARCONFIGREQUEST"))
(unless (find-package "PING_SONAR::SENDINGSONARCONFIGRESPONSE")
  (make-package "PING_SONAR::SENDINGSONARCONFIGRESPONSE"))

(in-package "ROS")





(defclass ping_sonar::sendingSonarConfigRequest
  :super ros::object
  :slots (_stepSize _range ))

(defmethod ping_sonar::sendingSonarConfigRequest
  (:init
   (&key
    ((:stepSize __stepSize) 0)
    ((:range __range) 0)
    )
   (send-super :init)
   (setq _stepSize (round __stepSize))
   (setq _range (round __range))
   self)
  (:stepSize
   (&optional __stepSize)
   (if __stepSize (setq _stepSize __stepSize)) _stepSize)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:serialization-length
   ()
   (+
    ;; int64 _stepSize
    8
    ;; int64 _range
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _stepSize
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _stepSize (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _stepSize) (= (length (_stepSize . bv)) 2)) ;; bignum
              (write-long (ash (elt (_stepSize . bv) 0) 0) s)
              (write-long (ash (elt (_stepSize . bv) 1) -1) s))
             ((and (class _stepSize) (= (length (_stepSize . bv)) 1)) ;; big1
              (write-long (elt (_stepSize . bv) 0) s)
              (write-long (if (>= _stepSize 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _stepSize s)(write-long (if (>= _stepSize 0) 0 #xffffffff) s)))
     ;; int64 _range
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _range (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _range) (= (length (_range . bv)) 2)) ;; bignum
              (write-long (ash (elt (_range . bv) 0) 0) s)
              (write-long (ash (elt (_range . bv) 1) -1) s))
             ((and (class _range) (= (length (_range . bv)) 1)) ;; big1
              (write-long (elt (_range . bv) 0) s)
              (write-long (if (>= _range 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _range s)(write-long (if (>= _range 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _stepSize
#+(or :alpha :irix6 :x86_64)
      (setf _stepSize (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _stepSize (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _range
#+(or :alpha :irix6 :x86_64)
      (setf _range (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _range (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass ping_sonar::sendingSonarConfigResponse
  :super ros::object
  :slots (_worked ))

(defmethod ping_sonar::sendingSonarConfigResponse
  (:init
   (&key
    ((:worked __worked) nil)
    )
   (send-super :init)
   (setq _worked __worked)
   self)
  (:worked
   (&optional (__worked :null))
   (if (not (eq __worked :null)) (setq _worked __worked)) _worked)
  (:serialization-length
   ()
   (+
    ;; bool _worked
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _worked
       (if _worked (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _worked
     (setq _worked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ping_sonar::sendingSonarConfig
  :super ros::object
  :slots ())

(setf (get ping_sonar::sendingSonarConfig :md5sum-) "6cb2431d101e8b181456b4863d041954")
(setf (get ping_sonar::sendingSonarConfig :datatype-) "ping_sonar/sendingSonarConfig")
(setf (get ping_sonar::sendingSonarConfig :request) ping_sonar::sendingSonarConfigRequest)
(setf (get ping_sonar::sendingSonarConfig :response) ping_sonar::sendingSonarConfigResponse)

(defmethod ping_sonar::sendingSonarConfigRequest
  (:response () (instance ping_sonar::sendingSonarConfigResponse :init)))

(setf (get ping_sonar::sendingSonarConfigRequest :md5sum-) "6cb2431d101e8b181456b4863d041954")
(setf (get ping_sonar::sendingSonarConfigRequest :datatype-) "ping_sonar/sendingSonarConfigRequest")
(setf (get ping_sonar::sendingSonarConfigRequest :definition-)
      "int64 stepSize
int64 range
---
bool worked
")

(setf (get ping_sonar::sendingSonarConfigResponse :md5sum-) "6cb2431d101e8b181456b4863d041954")
(setf (get ping_sonar::sendingSonarConfigResponse :datatype-) "ping_sonar/sendingSonarConfigResponse")
(setf (get ping_sonar::sendingSonarConfigResponse :definition-)
      "int64 stepSize
int64 range
---
bool worked
")



(provide :ping_sonar/sendingSonarConfig "6cb2431d101e8b181456b4863d041954")



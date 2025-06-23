;; Auto-generated. Do not edit!


(when (boundp 'mypackage88::Position)
  (if (not (find-package "MYPACKAGE88"))
    (make-package "MYPACKAGE88"))
  (shadow 'Position (find-package "MYPACKAGE88")))
(unless (find-package "MYPACKAGE88::POSITION")
  (make-package "MYPACKAGE88::POSITION"))

(in-package "ROS")
;;//! \htmlinclude Position.msg.html


(defclass mypackage88::Position
  :super ros::object
  :slots (_message _x _y ))

(defmethod mypackage88::Position
  (:init
   (&key
    ((:message __message) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mypackage88::Position :md5sum-) "7e1f88ae0dc8c54957ce388602e32426")
(setf (get mypackage88::Position :datatype-) "mypackage88/Position")
(setf (get mypackage88::Position :definition-)
      "string message
float32 x
float32 y
")



(provide :mypackage88/Position "7e1f88ae0dc8c54957ce388602e32426")



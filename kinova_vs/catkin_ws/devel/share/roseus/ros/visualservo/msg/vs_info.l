;; Auto-generated. Do not edit!


(when (boundp 'visualservo::vs_info)
  (if (not (find-package "VISUALSERVO"))
    (make-package "VISUALSERVO"))
  (shadow 'vs_info (find-package "VISUALSERVO")))
(unless (find-package "VISUALSERVO::VS_INFO")
  (make-package "VISUALSERVO::VS_INFO"))

(in-package "ROS")
;;//! \htmlinclude vs_info.msg.html


(defclass visualservo::vs_info
  :super ros::object
  :slots (_success _iterations _error_vector ))

(defmethod visualservo::vs_info
  (:init
   (&key
    ((:success __success) nil)
    ((:iterations __iterations) 0)
    ((:error_vector __error_vector) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _success __success)
   (setq _iterations (round __iterations))
   (setq _error_vector __error_vector)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:iterations
   (&optional __iterations)
   (if __iterations (setq _iterations __iterations)) _iterations)
  (:error_vector
   (&optional __error_vector)
   (if __error_vector (setq _error_vector __error_vector)) _error_vector)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _iterations
    4
    ;; float32[] _error_vector
    (* 4    (length _error_vector)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _iterations
       (write-long _iterations s)
     ;; float32[] _error_vector
     (write-long (length _error_vector) s)
     (dotimes (i (length _error_vector))
       (sys::poke (elt _error_vector i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _iterations
     (setq _iterations (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _error_vector
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error_vector (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _error_vector i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get visualservo::vs_info :md5sum-) "20b6e62368b4284b0dbbe944e7d01ad3")
(setf (get visualservo::vs_info :datatype-) "visualservo/vs_info")
(setf (get visualservo::vs_info :definition-)
      "bool success
int32 iterations
float32[] error_vector

")



(provide :visualservo/vs_info "20b6e62368b4284b0dbbe944e7d01ad3")



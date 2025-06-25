;; Auto-generated. Do not edit!


(when (boundp 'camera_node::image_point)
  (if (not (find-package "CAMERA_NODE"))
    (make-package "CAMERA_NODE"))
  (shadow 'image_point (find-package "CAMERA_NODE")))
(unless (find-package "CAMERA_NODE::IMAGE_POINT")
  (make-package "CAMERA_NODE::IMAGE_POINT"))

(in-package "ROS")
;;//! \htmlinclude image_point.msg.html


(defclass camera_node::image_point
  :super ros::object
  :slots (_x _y ))

(defmethod camera_node::image_point
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get camera_node::image_point :md5sum-) "209f516d3eb691f0663e25cb750d67c1")
(setf (get camera_node::image_point :datatype-) "camera_node/image_point")
(setf (get camera_node::image_point :definition-)
      "float64 x
float64 y
")



(provide :camera_node/image_point "209f516d3eb691f0663e25cb750d67c1")



; Auto-generated. Do not edit!


(cl:in-package visualservo-msg)


;//! \htmlinclude vs_info.msg.html

(cl:defclass <vs_info> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (iterations
    :reader iterations
    :initarg :iterations
    :type cl:integer
    :initform 0)
   (error_vector
    :reader error_vector
    :initarg :error_vector
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass vs_info (<vs_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vs_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vs_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualservo-msg:<vs_info> is deprecated: use visualservo-msg:vs_info instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <vs_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualservo-msg:success-val is deprecated.  Use visualservo-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'iterations-val :lambda-list '(m))
(cl:defmethod iterations-val ((m <vs_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualservo-msg:iterations-val is deprecated.  Use visualservo-msg:iterations instead.")
  (iterations m))

(cl:ensure-generic-function 'error_vector-val :lambda-list '(m))
(cl:defmethod error_vector-val ((m <vs_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualservo-msg:error_vector-val is deprecated.  Use visualservo-msg:error_vector instead.")
  (error_vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vs_info>) ostream)
  "Serializes a message object of type '<vs_info>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'iterations)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'error_vector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'error_vector))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vs_info>) istream)
  "Deserializes a message object of type '<vs_info>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'iterations) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'error_vector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'error_vector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vs_info>)))
  "Returns string type for a message object of type '<vs_info>"
  "visualservo/vs_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vs_info)))
  "Returns string type for a message object of type 'vs_info"
  "visualservo/vs_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vs_info>)))
  "Returns md5sum for a message object of type '<vs_info>"
  "20b6e62368b4284b0dbbe944e7d01ad3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vs_info)))
  "Returns md5sum for a message object of type 'vs_info"
  "20b6e62368b4284b0dbbe944e7d01ad3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vs_info>)))
  "Returns full string definition for message of type '<vs_info>"
  (cl:format cl:nil "bool success~%int32 iterations~%float32[] error_vector~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vs_info)))
  "Returns full string definition for message of type 'vs_info"
  (cl:format cl:nil "bool success~%int32 iterations~%float32[] error_vector~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vs_info>))
  (cl:+ 0
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'error_vector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vs_info>))
  "Converts a ROS message object to a list"
  (cl:list 'vs_info
    (cl:cons ':success (success msg))
    (cl:cons ':iterations (iterations msg))
    (cl:cons ':error_vector (error_vector msg))
))


(cl:in-package :asdf)

(defsystem "camera_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "image_point" :depends-on ("_package_image_point"))
    (:file "_package_image_point" :depends-on ("_package"))
  ))
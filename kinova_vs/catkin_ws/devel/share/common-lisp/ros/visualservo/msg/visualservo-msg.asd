
(cl:in-package :asdf)

(defsystem "visualservo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "image_point" :depends-on ("_package_image_point"))
    (:file "_package_image_point" :depends-on ("_package"))
    (:file "points_array" :depends-on ("_package_points_array"))
    (:file "_package_points_array" :depends-on ("_package"))
    (:file "vs_info" :depends-on ("_package_vs_info"))
    (:file "_package_vs_info" :depends-on ("_package"))
  ))
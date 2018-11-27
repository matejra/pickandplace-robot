
(cl:in-package :asdf)

(defsystem "camera_to_cv-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "points_array" :depends-on ("_package_points_array"))
    (:file "_package_points_array" :depends-on ("_package"))
  ))
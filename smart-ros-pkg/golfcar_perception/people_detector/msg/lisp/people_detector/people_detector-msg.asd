
(in-package :asdf)

(defsystem "people_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "labeled_obj" :depends-on ("_package"))
    (:file "_package_labeled_obj" :depends-on ("_package"))
    (:file "people_rect" :depends-on ("_package"))
    (:file "_package_people_rect" :depends-on ("_package"))
    (:file "people_rects" :depends-on ("_package"))
    (:file "_package_people_rects" :depends-on ("_package"))
    (:file "verified_obj" :depends-on ("_package"))
    (:file "_package_verified_obj" :depends-on ("_package"))
    (:file "labeled_objs" :depends-on ("_package"))
    (:file "_package_labeled_objs" :depends-on ("_package"))
    (:file "verified_objs" :depends-on ("_package"))
    (:file "_package_verified_objs" :depends-on ("_package"))
    ))


(in-package :asdf)

(defsystem "pedestrian_detector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "pedestrian" :depends-on ("_package"))
    (:file "_package_pedestrian" :depends-on ("_package"))
    ))

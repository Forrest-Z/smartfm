
(in-package :asdf)

(defsystem "golfcar_lowlevel-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "throttle" :depends-on ("_package"))
    (:file "_package_throttle" :depends-on ("_package"))
    (:file "vel" :depends-on ("_package"))
    (:file "_package_vel" :depends-on ("_package"))
    (:file "steering" :depends-on ("_package"))
    (:file "_package_steering" :depends-on ("_package"))
    (:file "brakepedal" :depends-on ("_package"))
    (:file "_package_brakepedal" :depends-on ("_package"))
    ))

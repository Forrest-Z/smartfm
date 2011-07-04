
(in-package :asdf)

(defsystem "golfcar_halsampler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "odo" :depends-on ("_package"))
    (:file "_package_odo" :depends-on ("_package"))
    ))

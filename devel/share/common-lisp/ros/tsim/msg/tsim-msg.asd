
(cl:in-package :asdf)

(defsystem "tsim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PoseError" :depends-on ("_package_PoseError"))
    (:file "_package_PoseError" :depends-on ("_package"))
  ))
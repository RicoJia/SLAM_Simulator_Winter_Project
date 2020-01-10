
(cl:in-package :asdf)

(defsystem "tsim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "traj_reset" :depends-on ("_package_traj_reset"))
    (:file "_package_traj_reset" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'tsim::traj_reset)
  (if (not (find-package "TSIM"))
    (make-package "TSIM"))
  (shadow 'traj_reset (find-package "TSIM")))
(unless (find-package "TSIM::TRAJ_RESET")
  (make-package "TSIM::TRAJ_RESET"))
(unless (find-package "TSIM::TRAJ_RESETREQUEST")
  (make-package "TSIM::TRAJ_RESETREQUEST"))
(unless (find-package "TSIM::TRAJ_RESETRESPONSE")
  (make-package "TSIM::TRAJ_RESETRESPONSE"))

(in-package "ROS")





(defclass tsim::traj_resetRequest
  :super ros::object
  :slots ())

(defmethod tsim::traj_resetRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass tsim::traj_resetResponse
  :super ros::object
  :slots ())

(defmethod tsim::traj_resetResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass tsim::traj_reset
  :super ros::object
  :slots ())

(setf (get tsim::traj_reset :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get tsim::traj_reset :datatype-) "tsim/traj_reset")
(setf (get tsim::traj_reset :request) tsim::traj_resetRequest)
(setf (get tsim::traj_reset :response) tsim::traj_resetResponse)

(defmethod tsim::traj_resetRequest
  (:response () (instance tsim::traj_resetResponse :init)))

(setf (get tsim::traj_resetRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get tsim::traj_resetRequest :datatype-) "tsim/traj_resetRequest")
(setf (get tsim::traj_resetRequest :definition-)
      "---

")

(setf (get tsim::traj_resetResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get tsim::traj_resetResponse :datatype-) "tsim/traj_resetResponse")
(setf (get tsim::traj_resetResponse :definition-)
      "---

")



(provide :tsim/traj_reset "d41d8cd98f00b204e9800998ecf8427e")



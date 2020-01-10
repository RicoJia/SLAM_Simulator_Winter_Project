;; Auto-generated. Do not edit!


(when (boundp 'tsim::PoseError)
  (if (not (find-package "TSIM"))
    (make-package "TSIM"))
  (shadow 'PoseError (find-package "TSIM")))
(unless (find-package "TSIM::POSEERROR")
  (make-package "TSIM::POSEERROR"))

(in-package "ROS")
;;//! \htmlinclude PoseError.msg.html


(defclass tsim::PoseError
  :super ros::object
  :slots (_x_error _y_error _theta_error ))

(defmethod tsim::PoseError
  (:init
   (&key
    ((:x_error __x_error) 0.0)
    ((:y_error __y_error) 0.0)
    ((:theta_error __theta_error) 0.0)
    )
   (send-super :init)
   (setq _x_error (float __x_error))
   (setq _y_error (float __y_error))
   (setq _theta_error (float __theta_error))
   self)
  (:x_error
   (&optional __x_error)
   (if __x_error (setq _x_error __x_error)) _x_error)
  (:y_error
   (&optional __y_error)
   (if __y_error (setq _y_error __y_error)) _y_error)
  (:theta_error
   (&optional __theta_error)
   (if __theta_error (setq _theta_error __theta_error)) _theta_error)
  (:serialization-length
   ()
   (+
    ;; float64 _x_error
    8
    ;; float64 _y_error
    8
    ;; float64 _theta_error
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_error
       (sys::poke _x_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_error
       (sys::poke _y_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta_error
       (sys::poke _theta_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_error
     (setq _x_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_error
     (setq _y_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta_error
     (setq _theta_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get tsim::PoseError :md5sum-) "206b7def6dfb0bd5e66edf944c1bb4dc")
(setf (get tsim::PoseError :datatype-) "tsim/PoseError")
(setf (get tsim::PoseError :definition-)
      "float64 x_error #absolute value of the x error
float64 y_error #absolute value of the x error
float64 theta_error #absolute value of the angular error
")



(provide :tsim/PoseError "206b7def6dfb0bd5e66edf944c1bb4dc")



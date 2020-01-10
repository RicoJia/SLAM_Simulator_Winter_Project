; Auto-generated. Do not edit!


(cl:in-package tsim-msg)


;//! \htmlinclude PoseError.msg.html

(cl:defclass <PoseError> (roslisp-msg-protocol:ros-message)
  ((x_error
    :reader x_error
    :initarg :x_error
    :type cl:float
    :initform 0.0)
   (y_error
    :reader y_error
    :initarg :y_error
    :type cl:float
    :initform 0.0)
   (theta_error
    :reader theta_error
    :initarg :theta_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass PoseError (<PoseError>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseError>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseError)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsim-msg:<PoseError> is deprecated: use tsim-msg:PoseError instead.")))

(cl:ensure-generic-function 'x_error-val :lambda-list '(m))
(cl:defmethod x_error-val ((m <PoseError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsim-msg:x_error-val is deprecated.  Use tsim-msg:x_error instead.")
  (x_error m))

(cl:ensure-generic-function 'y_error-val :lambda-list '(m))
(cl:defmethod y_error-val ((m <PoseError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsim-msg:y_error-val is deprecated.  Use tsim-msg:y_error instead.")
  (y_error m))

(cl:ensure-generic-function 'theta_error-val :lambda-list '(m))
(cl:defmethod theta_error-val ((m <PoseError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsim-msg:theta_error-val is deprecated.  Use tsim-msg:theta_error instead.")
  (theta_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseError>) ostream)
  "Serializes a message object of type '<PoseError>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseError>) istream)
  "Deserializes a message object of type '<PoseError>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseError>)))
  "Returns string type for a message object of type '<PoseError>"
  "tsim/PoseError")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseError)))
  "Returns string type for a message object of type 'PoseError"
  "tsim/PoseError")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseError>)))
  "Returns md5sum for a message object of type '<PoseError>"
  "206b7def6dfb0bd5e66edf944c1bb4dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseError)))
  "Returns md5sum for a message object of type 'PoseError"
  "206b7def6dfb0bd5e66edf944c1bb4dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseError>)))
  "Returns full string definition for message of type '<PoseError>"
  (cl:format cl:nil "float64 x_error #absolute value of the x error~%float64 y_error #absolute value of the x error~%float64 theta_error #absolute value of the angular error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseError)))
  "Returns full string definition for message of type 'PoseError"
  (cl:format cl:nil "float64 x_error #absolute value of the x error~%float64 y_error #absolute value of the x error~%float64 theta_error #absolute value of the angular error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseError>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseError>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseError
    (cl:cons ':x_error (x_error msg))
    (cl:cons ':y_error (y_error msg))
    (cl:cons ':theta_error (theta_error msg))
))

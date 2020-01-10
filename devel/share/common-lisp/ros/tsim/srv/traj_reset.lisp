; Auto-generated. Do not edit!


(cl:in-package tsim-srv)


;//! \htmlinclude traj_reset-request.msg.html

(cl:defclass <traj_reset-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass traj_reset-request (<traj_reset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traj_reset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traj_reset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsim-srv:<traj_reset-request> is deprecated: use tsim-srv:traj_reset-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traj_reset-request>) ostream)
  "Serializes a message object of type '<traj_reset-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traj_reset-request>) istream)
  "Deserializes a message object of type '<traj_reset-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traj_reset-request>)))
  "Returns string type for a service object of type '<traj_reset-request>"
  "tsim/traj_resetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traj_reset-request)))
  "Returns string type for a service object of type 'traj_reset-request"
  "tsim/traj_resetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traj_reset-request>)))
  "Returns md5sum for a message object of type '<traj_reset-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traj_reset-request)))
  "Returns md5sum for a message object of type 'traj_reset-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traj_reset-request>)))
  "Returns full string definition for message of type '<traj_reset-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traj_reset-request)))
  "Returns full string definition for message of type 'traj_reset-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traj_reset-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traj_reset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'traj_reset-request
))
;//! \htmlinclude traj_reset-response.msg.html

(cl:defclass <traj_reset-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass traj_reset-response (<traj_reset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traj_reset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traj_reset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsim-srv:<traj_reset-response> is deprecated: use tsim-srv:traj_reset-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traj_reset-response>) ostream)
  "Serializes a message object of type '<traj_reset-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traj_reset-response>) istream)
  "Deserializes a message object of type '<traj_reset-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traj_reset-response>)))
  "Returns string type for a service object of type '<traj_reset-response>"
  "tsim/traj_resetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traj_reset-response)))
  "Returns string type for a service object of type 'traj_reset-response"
  "tsim/traj_resetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traj_reset-response>)))
  "Returns md5sum for a message object of type '<traj_reset-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traj_reset-response)))
  "Returns md5sum for a message object of type 'traj_reset-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traj_reset-response>)))
  "Returns full string definition for message of type '<traj_reset-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traj_reset-response)))
  "Returns full string definition for message of type 'traj_reset-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traj_reset-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traj_reset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'traj_reset-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'traj_reset)))
  'traj_reset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'traj_reset)))
  'traj_reset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traj_reset)))
  "Returns string type for a service object of type '<traj_reset>"
  "tsim/traj_reset")
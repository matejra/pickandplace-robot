; Auto-generated. Do not edit!


(cl:in-package camera_to_cv-msg)


;//! \htmlinclude points_array.msg.html

(cl:defclass <points_array> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass points_array (<points_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <points_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'points_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_to_cv-msg:<points_array> is deprecated: use camera_to_cv-msg:points_array instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <points_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_to_cv-msg:points-val is deprecated.  Use camera_to_cv-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <points_array>) ostream)
  "Serializes a message object of type '<points_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <points_array>) istream)
  "Deserializes a message object of type '<points_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<points_array>)))
  "Returns string type for a message object of type '<points_array>"
  "camera_to_cv/points_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'points_array)))
  "Returns string type for a message object of type 'points_array"
  "camera_to_cv/points_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<points_array>)))
  "Returns md5sum for a message object of type '<points_array>"
  "3fb3f9dacc279b964c4c8341122c34df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'points_array)))
  "Returns md5sum for a message object of type 'points_array"
  "3fb3f9dacc279b964c4c8341122c34df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<points_array>)))
  "Returns full string definition for message of type '<points_array>"
  (cl:format cl:nil "geometry_msgs/Point[] points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'points_array)))
  "Returns full string definition for message of type 'points_array"
  (cl:format cl:nil "geometry_msgs/Point[] points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <points_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <points_array>))
  "Converts a ROS message object to a list"
  (cl:list 'points_array
    (cl:cons ':points (points msg))
))

; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude labeled_objs.msg.html

(defclass <labeled_objs> (ros-message)
  ((lb_objs_vector
    :reader lb_objs_vector-val
    :initarg :lb_objs_vector
    :type (vector people_detector-msg:<labeled_obj>)
   :initform (make-array 0 :element-type 'people_detector-msg:<labeled_obj> :initial-element (make-instance 'people_detector-msg:<labeled_obj>))))
)
(defmethod serialize ((msg <labeled_objs>) ostream)
  "Serializes a message object of type '<labeled_objs>"
  (let ((__ros_arr_len (length (slot-value msg 'lb_objs_vector))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'lb_objs_vector))
)
(defmethod deserialize ((msg <labeled_objs>) istream)
  "Deserializes a message object of type '<labeled_objs>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'lb_objs_vector) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'lb_objs_vector)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance 'people_detector-msg:<labeled_obj>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<labeled_objs>)))
  "Returns string type for a message object of type '<labeled_objs>"
  "people_detector/labeled_objs")
(defmethod md5sum ((type (eql '<labeled_objs>)))
  "Returns md5sum for a message object of type '<labeled_objs>"
  "3fbca97366a8e809e4e3a9af28655b50")
(defmethod message-definition ((type (eql '<labeled_objs>)))
  "Returns full string definition for message of type '<labeled_objs>"
  (format nil "people_detector/labeled_obj[] lb_objs_vector~%~%================================================================================~%MSG: people_detector/labeled_obj~%int32 object_label~%geometry_msgs/Point32 pedestrian_point~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(defmethod serialization-length ((msg <labeled_objs>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'lb_objs_vector) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <labeled_objs>))
  "Converts a ROS message object to a list"
  (list '<labeled_objs>
    (cons ':lb_objs_vector (lb_objs_vector-val msg))
))

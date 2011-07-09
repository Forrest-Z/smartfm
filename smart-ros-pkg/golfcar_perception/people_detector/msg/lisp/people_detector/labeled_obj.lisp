; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude labeled_obj.msg.html

(defclass <labeled_obj> (ros-message)
  ((object_label
    :reader object_label-val
    :initarg :object_label
    :type integer
    :initform 0)
   (pedestrian_point
    :reader pedestrian_point-val
    :initarg :pedestrian_point
    :type geometry_msgs-msg:<Point32>
    :initform (make-instance 'geometry_msgs-msg:<Point32>)))
)
(defmethod serialize ((msg <labeled_obj>) ostream)
  "Serializes a message object of type '<labeled_obj>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'object_label)) ostream)
  (serialize (slot-value msg 'pedestrian_point) ostream)
)
(defmethod deserialize ((msg <labeled_obj>) istream)
  "Deserializes a message object of type '<labeled_obj>"
  (setf (ldb (byte 8 0) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'object_label)) (read-byte istream))
  (deserialize (slot-value msg 'pedestrian_point) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<labeled_obj>)))
  "Returns string type for a message object of type '<labeled_obj>"
  "people_detector/labeled_obj")
(defmethod md5sum ((type (eql '<labeled_obj>)))
  "Returns md5sum for a message object of type '<labeled_obj>"
  "b67a48dfc781c81262238e4f6d8d6a1c")
(defmethod message-definition ((type (eql '<labeled_obj>)))
  "Returns full string definition for message of type '<labeled_obj>"
  (format nil "int32 object_label~%geometry_msgs/Point32 pedestrian_point~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(defmethod serialization-length ((msg <labeled_obj>))
  (+ 0
     4
     (serialization-length (slot-value msg 'pedestrian_point))
))
(defmethod ros-message-to-list ((msg <labeled_obj>))
  "Converts a ROS message object to a list"
  (list '<labeled_obj>
    (cons ':object_label (object_label-val msg))
    (cons ':pedestrian_point (pedestrian_point-val msg))
))

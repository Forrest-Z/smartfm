; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude verified_obj.msg.html

(defclass <verified_obj> (ros-message)
  ((object_label
    :reader object_label-val
    :initarg :object_label
    :type integer
    :initform 0)
   (complete_flag
    :reader complete_flag-val
    :initarg :complete_flag
    :type boolean
    :initform nil)
   (decision_flag
    :reader decision_flag-val
    :initarg :decision_flag
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <verified_obj>) ostream)
  "Serializes a message object of type '<verified_obj>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'object_label)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'complete_flag) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'decision_flag) 1 0)) ostream)
)
(defmethod deserialize ((msg <verified_obj>) istream)
  "Deserializes a message object of type '<verified_obj>"
  (setf (ldb (byte 8 0) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'object_label)) (read-byte istream))
  (setf (slot-value msg 'complete_flag) (not (zerop (read-byte istream))))
  (setf (slot-value msg 'decision_flag) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<verified_obj>)))
  "Returns string type for a message object of type '<verified_obj>"
  "people_detector/verified_obj")
(defmethod md5sum ((type (eql '<verified_obj>)))
  "Returns md5sum for a message object of type '<verified_obj>"
  "05d55c2630fe57ca54ba39e2030a2938")
(defmethod message-definition ((type (eql '<verified_obj>)))
  "Returns full string definition for message of type '<verified_obj>"
  (format nil "int32 object_label~%bool complete_flag~%bool decision_flag~%~%~%"))
(defmethod serialization-length ((msg <verified_obj>))
  (+ 0
     4
     1
     1
))
(defmethod ros-message-to-list ((msg <verified_obj>))
  "Converts a ROS message object to a list"
  (list '<verified_obj>
    (cons ':object_label (object_label-val msg))
    (cons ':complete_flag (complete_flag-val msg))
    (cons ':decision_flag (decision_flag-val msg))
))

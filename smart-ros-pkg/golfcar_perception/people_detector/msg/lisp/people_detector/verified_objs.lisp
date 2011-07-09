; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude verified_objs.msg.html

(defclass <verified_objs> (ros-message)
  ((veri_objs
    :reader veri_objs-val
    :initarg :veri_objs
    :type (vector people_detector-msg:<verified_obj>)
   :initform (make-array 0 :element-type 'people_detector-msg:<verified_obj> :initial-element (make-instance 'people_detector-msg:<verified_obj>))))
)
(defmethod serialize ((msg <verified_objs>) ostream)
  "Serializes a message object of type '<verified_objs>"
  (let ((__ros_arr_len (length (slot-value msg 'veri_objs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'veri_objs))
)
(defmethod deserialize ((msg <verified_objs>) istream)
  "Deserializes a message object of type '<verified_objs>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'veri_objs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'veri_objs)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance 'people_detector-msg:<verified_obj>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<verified_objs>)))
  "Returns string type for a message object of type '<verified_objs>"
  "people_detector/verified_objs")
(defmethod md5sum ((type (eql '<verified_objs>)))
  "Returns md5sum for a message object of type '<verified_objs>"
  "6402d5977c135ed13456e2f0904e8604")
(defmethod message-definition ((type (eql '<verified_objs>)))
  "Returns full string definition for message of type '<verified_objs>"
  (format nil "people_detector/verified_obj[] veri_objs~%~%================================================================================~%MSG: people_detector/verified_obj~%int32 object_label~%bool complete_flag~%bool decision_flag~%~%~%"))
(defmethod serialization-length ((msg <verified_objs>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'veri_objs) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <verified_objs>))
  "Converts a ROS message object to a list"
  (list '<verified_objs>
    (cons ':veri_objs (veri_objs-val msg))
))

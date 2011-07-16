; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude people_rects.msg.html

(defclass <people_rects> (ros-message)
  ((pr_vector
    :reader pr_vector-val
    :initarg :pr_vector
    :type (vector people_detector-msg:<people_rect>)
   :initform (make-array 0 :element-type 'people_detector-msg:<people_rect> :initial-element (make-instance 'people_detector-msg:<people_rect>))))
)
(defmethod serialize ((msg <people_rects>) ostream)
  "Serializes a message object of type '<people_rects>"
  (let ((__ros_arr_len (length (slot-value msg 'pr_vector))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'pr_vector))
)
(defmethod deserialize ((msg <people_rects>) istream)
  "Deserializes a message object of type '<people_rects>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'pr_vector) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'pr_vector)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance 'people_detector-msg:<people_rect>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<people_rects>)))
  "Returns string type for a message object of type '<people_rects>"
  "people_detector/people_rects")
(defmethod md5sum ((type (eql '<people_rects>)))
  "Returns md5sum for a message object of type '<people_rects>"
  "eabc228c7310deec8b5076821d1ceae4")
(defmethod message-definition ((type (eql '<people_rects>)))
  "Returns full string definition for message of type '<people_rects>"
  (format nil "people_detector/people_rect[] pr_vector~%~%================================================================================~%MSG: people_detector/people_rect~%int32 object_label~%bool complete_flag~%bool decision_flag~%int32 x~%int32 y~%int32 height~%int32 width~%int32 scaled_x~%int32 scaled_y~%int32 scaled_height~%int32 scaled_width~%float32 disz~%float32 positionx~%float32 positiony~%float32 positionz~%float32 cvRect_x1~%float32 cvRect_y1~%float32 cvRect_x2~%float32 cvRect_y2~%~%~%"))
(defmethod serialization-length ((msg <people_rects>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'pr_vector) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <people_rects>))
  "Converts a ROS message object to a list"
  (list '<people_rects>
    (cons ':pr_vector (pr_vector-val msg))
))

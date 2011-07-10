; Auto-generated. Do not edit!


(in-package golfcar_lowlevel-msg)


;//! \htmlinclude steering.msg.html

(defclass <steering> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <steering>) ostream)
  "Serializes a message object of type '<steering>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <steering>) istream)
  "Deserializes a message object of type '<steering>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<steering>)))
  "Returns string type for a message object of type '<steering>"
  "golfcar_lowlevel/steering")
(defmethod md5sum ((type (eql '<steering>)))
  "Returns md5sum for a message object of type '<steering>"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(defmethod message-definition ((type (eql '<steering>)))
  "Returns full string definition for message of type '<steering>"
  (format nil "float32 angle~%~%~%"))
(defmethod serialization-length ((msg <steering>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <steering>))
  "Converts a ROS message object to a list"
  (list '<steering>
    (cons ':angle (angle-val msg))
))

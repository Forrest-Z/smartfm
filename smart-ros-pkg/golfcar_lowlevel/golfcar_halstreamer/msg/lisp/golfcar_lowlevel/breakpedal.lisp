; Auto-generated. Do not edit!


(in-package golfcar_lowlevel-msg)


;//! \htmlinclude breakpedal.msg.html

(defclass <breakpedal> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <breakpedal>) ostream)
  "Serializes a message object of type '<breakpedal>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <breakpedal>) istream)
  "Deserializes a message object of type '<breakpedal>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<breakpedal>)))
  "Returns string type for a message object of type '<breakpedal>"
  "golfcar_lowlevel/breakpedal")
(defmethod md5sum ((type (eql '<breakpedal>)))
  "Returns md5sum for a message object of type '<breakpedal>"
  "2d11dcdbe5a6f73dd324353dc52315ab")
(defmethod message-definition ((type (eql '<breakpedal>)))
  "Returns full string definition for message of type '<breakpedal>"
  (format nil "float32 angle~%~%~%"))
(defmethod serialization-length ((msg <breakpedal>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <breakpedal>))
  "Converts a ROS message object to a list"
  (list '<breakpedal>
    (cons ':angle (angle-val msg))
))

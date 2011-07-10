; Auto-generated. Do not edit!


(in-package golfcar_lowlevel-msg)


;//! \htmlinclude throttle.msg.html

(defclass <throttle> (ros-message)
  ((volt
    :reader volt-val
    :initarg :volt
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <throttle>) ostream)
  "Serializes a message object of type '<throttle>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'volt))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <throttle>) istream)
  "Deserializes a message object of type '<throttle>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'volt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<throttle>)))
  "Returns string type for a message object of type '<throttle>"
  "golfcar_lowlevel/throttle")
(defmethod md5sum ((type (eql '<throttle>)))
  "Returns md5sum for a message object of type '<throttle>"
  "0d974440801c3c58757fe6ef22b11eb3")
(defmethod message-definition ((type (eql '<throttle>)))
  "Returns full string definition for message of type '<throttle>"
  (format nil "float32 volt~%~%~%"))
(defmethod serialization-length ((msg <throttle>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <throttle>))
  "Converts a ROS message object to a list"
  (list '<throttle>
    (cons ':volt (volt-val msg))
))

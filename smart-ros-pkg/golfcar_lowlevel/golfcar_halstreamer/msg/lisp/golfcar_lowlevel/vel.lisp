; Auto-generated. Do not edit!


(in-package golfcar_lowlevel-msg)


;//! \htmlinclude vel.msg.html

(defclass <vel> (ros-message)
  ((speed
    :reader speed-val
    :initarg :speed
    :type float
    :initform 0.0)
   (angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <vel>) ostream)
  "Serializes a message object of type '<vel>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'speed))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <vel>) istream)
  "Deserializes a message object of type '<vel>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<vel>)))
  "Returns string type for a message object of type '<vel>"
  "golfcar_lowlevel/vel")
(defmethod md5sum ((type (eql '<vel>)))
  "Returns md5sum for a message object of type '<vel>"
  "e18a4dfdb52fee48fc2e3bc9e7b74071")
(defmethod message-definition ((type (eql '<vel>)))
  "Returns full string definition for message of type '<vel>"
  (format nil "float32 speed~%float32 angle~%~%~%"))
(defmethod serialization-length ((msg <vel>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <vel>))
  "Converts a ROS message object to a list"
  (list '<vel>
    (cons ':speed (speed-val msg))
    (cons ':angle (angle-val msg))
))

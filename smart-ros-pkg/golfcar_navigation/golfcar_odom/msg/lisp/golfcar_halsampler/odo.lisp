; Auto-generated. Do not edit!


(in-package golfcar_halsampler-msg)


;//! \htmlinclude odo.msg.html

(defclass <odo> (ros-message)
  ((pose
    :reader pose-val
    :initarg :pose
    :type float
    :initform 0.0)
   (vel
    :reader vel-val
    :initarg :vel
    :type float
    :initform 0.0)
   (steering_angle
    :reader steering_angle-val
    :initarg :steering_angle
    :type float
    :initform 0.0)
   (emergency
    :reader emergency-val
    :initarg :emergency
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <odo>) ostream)
  "Serializes a message object of type '<odo>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'pose))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'vel))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'steering_angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'emergency) 1 0)) ostream)
)
(defmethod deserialize ((msg <odo>) istream)
  "Deserializes a message object of type '<odo>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'pose) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'vel) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'steering_angle) (roslisp-utils:decode-double-float-bits bits)))
  (setf (slot-value msg 'emergency) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<odo>)))
  "Returns string type for a message object of type '<odo>"
  "golfcar_halsampler/odo")
(defmethod md5sum ((type (eql '<odo>)))
  "Returns md5sum for a message object of type '<odo>"
  "e68537df914d3d6ed7f2787b0001f4ed")
(defmethod message-definition ((type (eql '<odo>)))
  "Returns full string definition for message of type '<odo>"
  (format nil "float64 pose~%float64 vel~%float64 steering_angle~%bool emergency~%~%~%"))
(defmethod serialization-length ((msg <odo>))
  (+ 0
     8
     8
     8
     1
))
(defmethod ros-message-to-list ((msg <odo>))
  "Converts a ROS message object to a list"
  (list '<odo>
    (cons ':pose (pose-val msg))
    (cons ':vel (vel-val msg))
    (cons ':steering_angle (steering_angle-val msg))
    (cons ':emergency (emergency-val msg))
))

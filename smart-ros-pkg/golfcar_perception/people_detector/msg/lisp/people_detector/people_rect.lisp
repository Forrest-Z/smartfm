; Auto-generated. Do not edit!


(in-package people_detector-msg)


;//! \htmlinclude people_rect.msg.html

(defclass <people_rect> (ros-message)
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
    :initform nil)
   (x
    :reader x-val
    :initarg :x
    :type integer
    :initform 0)
   (y
    :reader y-val
    :initarg :y
    :type integer
    :initform 0)
   (height
    :reader height-val
    :initarg :height
    :type integer
    :initform 0)
   (width
    :reader width-val
    :initarg :width
    :type integer
    :initform 0)
   (scaled_x
    :reader scaled_x-val
    :initarg :scaled_x
    :type integer
    :initform 0)
   (scaled_y
    :reader scaled_y-val
    :initarg :scaled_y
    :type integer
    :initform 0)
   (scaled_height
    :reader scaled_height-val
    :initarg :scaled_height
    :type integer
    :initform 0)
   (scaled_width
    :reader scaled_width-val
    :initarg :scaled_width
    :type integer
    :initform 0)
   (disz
    :reader disz-val
    :initarg :disz
    :type float
    :initform 0.0)
   (positionx
    :reader positionx-val
    :initarg :positionx
    :type float
    :initform 0.0)
   (positiony
    :reader positiony-val
    :initarg :positiony
    :type float
    :initform 0.0)
   (positionz
    :reader positionz-val
    :initarg :positionz
    :type float
    :initform 0.0)
   (cvRect_x1
    :reader cvRect_x1-val
    :initarg :cvRect_x1
    :type float
    :initform 0.0)
   (cvRect_y1
    :reader cvRect_y1-val
    :initarg :cvRect_y1
    :type float
    :initform 0.0)
   (cvRect_x2
    :reader cvRect_x2-val
    :initarg :cvRect_x2
    :type float
    :initform 0.0)
   (cvRect_y2
    :reader cvRect_y2-val
    :initarg :cvRect_y2
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <people_rect>) ostream)
  "Serializes a message object of type '<people_rect>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'object_label)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'object_label)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'complete_flag) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'decision_flag) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'height)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'width)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'scaled_x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'scaled_x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'scaled_x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'scaled_x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'scaled_y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'scaled_y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'scaled_y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'scaled_y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'scaled_height)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'scaled_height)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'scaled_height)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'scaled_height)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'scaled_width)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'scaled_width)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'scaled_width)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'scaled_width)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'disz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'positionx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'positiony))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'positionz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'cvRect_x1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'cvRect_y1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'cvRect_x2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'cvRect_y2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <people_rect>) istream)
  "Deserializes a message object of type '<people_rect>"
  (setf (ldb (byte 8 0) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'object_label)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'object_label)) (read-byte istream))
  (setf (slot-value msg 'complete_flag) (not (zerop (read-byte istream))))
  (setf (slot-value msg 'decision_flag) (not (zerop (read-byte istream))))
  (setf (ldb (byte 8 0) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'scaled_x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'scaled_x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'scaled_x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'scaled_x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'scaled_y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'scaled_y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'scaled_y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'scaled_y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'scaled_height)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'scaled_height)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'scaled_height)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'scaled_height)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'scaled_width)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'scaled_width)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'scaled_width)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'scaled_width)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'disz) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'positionx) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'positiony) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'positionz) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'cvRect_x1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'cvRect_y1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'cvRect_x2) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'cvRect_y2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<people_rect>)))
  "Returns string type for a message object of type '<people_rect>"
  "people_detector/people_rect")
(defmethod md5sum ((type (eql '<people_rect>)))
  "Returns md5sum for a message object of type '<people_rect>"
  "079a28ad41b4cee9351921a21258bc8f")
(defmethod message-definition ((type (eql '<people_rect>)))
  "Returns full string definition for message of type '<people_rect>"
  (format nil "int32 object_label~%bool complete_flag~%bool decision_flag~%int32 x~%int32 y~%int32 height~%int32 width~%int32 scaled_x~%int32 scaled_y~%int32 scaled_height~%int32 scaled_width~%float32 disz~%float32 positionx~%float32 positiony~%float32 positionz~%float32 cvRect_x1~%float32 cvRect_y1~%float32 cvRect_x2~%float32 cvRect_y2~%~%~%"))
(defmethod serialization-length ((msg <people_rect>))
  (+ 0
     4
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <people_rect>))
  "Converts a ROS message object to a list"
  (list '<people_rect>
    (cons ':object_label (object_label-val msg))
    (cons ':complete_flag (complete_flag-val msg))
    (cons ':decision_flag (decision_flag-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':height (height-val msg))
    (cons ':width (width-val msg))
    (cons ':scaled_x (scaled_x-val msg))
    (cons ':scaled_y (scaled_y-val msg))
    (cons ':scaled_height (scaled_height-val msg))
    (cons ':scaled_width (scaled_width-val msg))
    (cons ':disz (disz-val msg))
    (cons ':positionx (positionx-val msg))
    (cons ':positiony (positiony-val msg))
    (cons ':positionz (positionz-val msg))
    (cons ':cvRect_x1 (cvRect_x1-val msg))
    (cons ':cvRect_y1 (cvRect_y1-val msg))
    (cons ':cvRect_x2 (cvRect_x2-val msg))
    (cons ':cvRect_y2 (cvRect_y2-val msg))
))

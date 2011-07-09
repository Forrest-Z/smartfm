; Auto-generated. Do not edit!


(in-package pedestrian_detector-srv)


;//! \htmlinclude pedestrian-request.msg.html

(defclass <pedestrian-request> (ros-message)
  ((request
    :reader request-val
    :initarg :request
    :type string
    :initform ""))
)
(defmethod serialize ((msg <pedestrian-request>) ostream)
  "Serializes a message object of type '<pedestrian-request>"
  (let ((__ros_str_len (length (slot-value msg 'request))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'request))
)
(defmethod deserialize ((msg <pedestrian-request>) istream)
  "Deserializes a message object of type '<pedestrian-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'request) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'request) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<pedestrian-request>)))
  "Returns string type for a service object of type '<pedestrian-request>"
  "pedestrian_detector/pedestrianRequest")
(defmethod md5sum ((type (eql '<pedestrian-request>)))
  "Returns md5sum for a message object of type '<pedestrian-request>"
  "8457315da567e77df49ff275cde45da0")
(defmethod message-definition ((type (eql '<pedestrian-request>)))
  "Returns full string definition for message of type '<pedestrian-request>"
  (format nil "string request~%~%"))
(defmethod serialization-length ((msg <pedestrian-request>))
  (+ 0
     4 (length (slot-value msg 'request))
))
(defmethod ros-message-to-list ((msg <pedestrian-request>))
  "Converts a ROS message object to a list"
  (list '<pedestrian-request>
    (cons ':request (request-val msg))
))
;//! \htmlinclude pedestrian-response.msg.html

(defclass <pedestrian-response> (ros-message)
  ((people
    :reader people-val
    :initarg :people
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <pedestrian-response>) ostream)
  "Serializes a message object of type '<pedestrian-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'people)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'people)) ostream)
)
(defmethod deserialize ((msg <pedestrian-response>) istream)
  "Deserializes a message object of type '<pedestrian-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'people)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'people)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<pedestrian-response>)))
  "Returns string type for a service object of type '<pedestrian-response>"
  "pedestrian_detector/pedestrianResponse")
(defmethod md5sum ((type (eql '<pedestrian-response>)))
  "Returns md5sum for a message object of type '<pedestrian-response>"
  "8457315da567e77df49ff275cde45da0")
(defmethod message-definition ((type (eql '<pedestrian-response>)))
  "Returns full string definition for message of type '<pedestrian-response>"
  (format nil "int64 people~%~%~%"))
(defmethod serialization-length ((msg <pedestrian-response>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <pedestrian-response>))
  "Converts a ROS message object to a list"
  (list '<pedestrian-response>
    (cons ':people (people-val msg))
))
(defmethod service-request-type ((msg (eql 'pedestrian)))
  '<pedestrian-request>)
(defmethod service-response-type ((msg (eql 'pedestrian)))
  '<pedestrian-response>)
(defmethod ros-datatype ((msg (eql 'pedestrian)))
  "Returns string type for a service object of type '<pedestrian>"
  "pedestrian_detector/pedestrian")

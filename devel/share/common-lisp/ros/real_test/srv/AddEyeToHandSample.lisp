; Auto-generated. Do not edit!


(cl:in-package real_test-srv)


;//! \htmlinclude AddEyeToHandSample-request.msg.html

(cl:defclass <AddEyeToHandSample-request> (roslisp-msg-protocol:ros-message)
  ((base_x
    :reader base_x
    :initarg :base_x
    :type cl:float
    :initform 0.0)
   (base_y
    :reader base_y
    :initarg :base_y
    :type cl:float
    :initform 0.0)
   (base_z
    :reader base_z
    :initarg :base_z
    :type cl:float
    :initform 0.0)
   (optical_x
    :reader optical_x
    :initarg :optical_x
    :type cl:float
    :initform 0.0)
   (optical_y
    :reader optical_y
    :initarg :optical_y
    :type cl:float
    :initform 0.0)
   (optical_z
    :reader optical_z
    :initarg :optical_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass AddEyeToHandSample-request (<AddEyeToHandSample-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddEyeToHandSample-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddEyeToHandSample-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_test-srv:<AddEyeToHandSample-request> is deprecated: use real_test-srv:AddEyeToHandSample-request instead.")))

(cl:ensure-generic-function 'base_x-val :lambda-list '(m))
(cl:defmethod base_x-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:base_x-val is deprecated.  Use real_test-srv:base_x instead.")
  (base_x m))

(cl:ensure-generic-function 'base_y-val :lambda-list '(m))
(cl:defmethod base_y-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:base_y-val is deprecated.  Use real_test-srv:base_y instead.")
  (base_y m))

(cl:ensure-generic-function 'base_z-val :lambda-list '(m))
(cl:defmethod base_z-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:base_z-val is deprecated.  Use real_test-srv:base_z instead.")
  (base_z m))

(cl:ensure-generic-function 'optical_x-val :lambda-list '(m))
(cl:defmethod optical_x-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:optical_x-val is deprecated.  Use real_test-srv:optical_x instead.")
  (optical_x m))

(cl:ensure-generic-function 'optical_y-val :lambda-list '(m))
(cl:defmethod optical_y-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:optical_y-val is deprecated.  Use real_test-srv:optical_y instead.")
  (optical_y m))

(cl:ensure-generic-function 'optical_z-val :lambda-list '(m))
(cl:defmethod optical_z-val ((m <AddEyeToHandSample-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:optical_z-val is deprecated.  Use real_test-srv:optical_z instead.")
  (optical_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddEyeToHandSample-request>) ostream)
  "Serializes a message object of type '<AddEyeToHandSample-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'base_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'base_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'base_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'optical_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'optical_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'optical_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddEyeToHandSample-request>) istream)
  "Deserializes a message object of type '<AddEyeToHandSample-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'optical_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'optical_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'optical_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddEyeToHandSample-request>)))
  "Returns string type for a service object of type '<AddEyeToHandSample-request>"
  "real_test/AddEyeToHandSampleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddEyeToHandSample-request)))
  "Returns string type for a service object of type 'AddEyeToHandSample-request"
  "real_test/AddEyeToHandSampleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddEyeToHandSample-request>)))
  "Returns md5sum for a message object of type '<AddEyeToHandSample-request>"
  "b1d912318536383476f3a61d6d9c6b77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddEyeToHandSample-request)))
  "Returns md5sum for a message object of type 'AddEyeToHandSample-request"
  "b1d912318536383476f3a61d6d9c6b77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddEyeToHandSample-request>)))
  "Returns full string definition for message of type '<AddEyeToHandSample-request>"
  (cl:format cl:nil "# 手动增加一对 3D 对应点：同一点在基座系与相机光学系下的坐标（米）~%float32 base_x~%float32 base_y~%float32 base_z~%float32 optical_x~%float32 optical_y~%float32 optical_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddEyeToHandSample-request)))
  "Returns full string definition for message of type 'AddEyeToHandSample-request"
  (cl:format cl:nil "# 手动增加一对 3D 对应点：同一点在基座系与相机光学系下的坐标（米）~%float32 base_x~%float32 base_y~%float32 base_z~%float32 optical_x~%float32 optical_y~%float32 optical_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddEyeToHandSample-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddEyeToHandSample-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddEyeToHandSample-request
    (cl:cons ':base_x (base_x msg))
    (cl:cons ':base_y (base_y msg))
    (cl:cons ':base_z (base_z msg))
    (cl:cons ':optical_x (optical_x msg))
    (cl:cons ':optical_y (optical_y msg))
    (cl:cons ':optical_z (optical_z msg))
))
;//! \htmlinclude AddEyeToHandSample-response.msg.html

(cl:defclass <AddEyeToHandSample-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass AddEyeToHandSample-response (<AddEyeToHandSample-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddEyeToHandSample-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddEyeToHandSample-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_test-srv:<AddEyeToHandSample-response> is deprecated: use real_test-srv:AddEyeToHandSample-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddEyeToHandSample-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:success-val is deprecated.  Use real_test-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <AddEyeToHandSample-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:message-val is deprecated.  Use real_test-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <AddEyeToHandSample-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_test-srv:count-val is deprecated.  Use real_test-srv:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddEyeToHandSample-response>) ostream)
  "Serializes a message object of type '<AddEyeToHandSample-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddEyeToHandSample-response>) istream)
  "Deserializes a message object of type '<AddEyeToHandSample-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddEyeToHandSample-response>)))
  "Returns string type for a service object of type '<AddEyeToHandSample-response>"
  "real_test/AddEyeToHandSampleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddEyeToHandSample-response)))
  "Returns string type for a service object of type 'AddEyeToHandSample-response"
  "real_test/AddEyeToHandSampleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddEyeToHandSample-response>)))
  "Returns md5sum for a message object of type '<AddEyeToHandSample-response>"
  "b1d912318536383476f3a61d6d9c6b77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddEyeToHandSample-response)))
  "Returns md5sum for a message object of type 'AddEyeToHandSample-response"
  "b1d912318536383476f3a61d6d9c6b77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddEyeToHandSample-response>)))
  "Returns full string definition for message of type '<AddEyeToHandSample-response>"
  (cl:format cl:nil "bool success~%string message~%int32 count~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddEyeToHandSample-response)))
  "Returns full string definition for message of type 'AddEyeToHandSample-response"
  (cl:format cl:nil "bool success~%string message~%int32 count~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddEyeToHandSample-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddEyeToHandSample-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddEyeToHandSample-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':count (count msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddEyeToHandSample)))
  'AddEyeToHandSample-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddEyeToHandSample)))
  'AddEyeToHandSample-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddEyeToHandSample)))
  "Returns string type for a service object of type '<AddEyeToHandSample>"
  "real_test/AddEyeToHandSample")
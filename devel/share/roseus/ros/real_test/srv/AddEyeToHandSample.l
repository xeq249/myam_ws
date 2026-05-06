;; Auto-generated. Do not edit!


(when (boundp 'real_test::AddEyeToHandSample)
  (if (not (find-package "REAL_TEST"))
    (make-package "REAL_TEST"))
  (shadow 'AddEyeToHandSample (find-package "REAL_TEST")))
(unless (find-package "REAL_TEST::ADDEYETOHANDSAMPLE")
  (make-package "REAL_TEST::ADDEYETOHANDSAMPLE"))
(unless (find-package "REAL_TEST::ADDEYETOHANDSAMPLEREQUEST")
  (make-package "REAL_TEST::ADDEYETOHANDSAMPLEREQUEST"))
(unless (find-package "REAL_TEST::ADDEYETOHANDSAMPLERESPONSE")
  (make-package "REAL_TEST::ADDEYETOHANDSAMPLERESPONSE"))

(in-package "ROS")





(defclass real_test::AddEyeToHandSampleRequest
  :super ros::object
  :slots (_base_x _base_y _base_z _optical_x _optical_y _optical_z ))

(defmethod real_test::AddEyeToHandSampleRequest
  (:init
   (&key
    ((:base_x __base_x) 0.0)
    ((:base_y __base_y) 0.0)
    ((:base_z __base_z) 0.0)
    ((:optical_x __optical_x) 0.0)
    ((:optical_y __optical_y) 0.0)
    ((:optical_z __optical_z) 0.0)
    )
   (send-super :init)
   (setq _base_x (float __base_x))
   (setq _base_y (float __base_y))
   (setq _base_z (float __base_z))
   (setq _optical_x (float __optical_x))
   (setq _optical_y (float __optical_y))
   (setq _optical_z (float __optical_z))
   self)
  (:base_x
   (&optional __base_x)
   (if __base_x (setq _base_x __base_x)) _base_x)
  (:base_y
   (&optional __base_y)
   (if __base_y (setq _base_y __base_y)) _base_y)
  (:base_z
   (&optional __base_z)
   (if __base_z (setq _base_z __base_z)) _base_z)
  (:optical_x
   (&optional __optical_x)
   (if __optical_x (setq _optical_x __optical_x)) _optical_x)
  (:optical_y
   (&optional __optical_y)
   (if __optical_y (setq _optical_y __optical_y)) _optical_y)
  (:optical_z
   (&optional __optical_z)
   (if __optical_z (setq _optical_z __optical_z)) _optical_z)
  (:serialization-length
   ()
   (+
    ;; float32 _base_x
    4
    ;; float32 _base_y
    4
    ;; float32 _base_z
    4
    ;; float32 _optical_x
    4
    ;; float32 _optical_y
    4
    ;; float32 _optical_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _base_x
       (sys::poke _base_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _base_y
       (sys::poke _base_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _base_z
       (sys::poke _base_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _optical_x
       (sys::poke _optical_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _optical_y
       (sys::poke _optical_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _optical_z
       (sys::poke _optical_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _base_x
     (setq _base_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _base_y
     (setq _base_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _base_z
     (setq _base_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _optical_x
     (setq _optical_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _optical_y
     (setq _optical_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _optical_z
     (setq _optical_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass real_test::AddEyeToHandSampleResponse
  :super ros::object
  :slots (_success _message _count ))

(defmethod real_test::AddEyeToHandSampleResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _count (round __count))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; int32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; int32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass real_test::AddEyeToHandSample
  :super ros::object
  :slots ())

(setf (get real_test::AddEyeToHandSample :md5sum-) "b1d912318536383476f3a61d6d9c6b77")
(setf (get real_test::AddEyeToHandSample :datatype-) "real_test/AddEyeToHandSample")
(setf (get real_test::AddEyeToHandSample :request) real_test::AddEyeToHandSampleRequest)
(setf (get real_test::AddEyeToHandSample :response) real_test::AddEyeToHandSampleResponse)

(defmethod real_test::AddEyeToHandSampleRequest
  (:response () (instance real_test::AddEyeToHandSampleResponse :init)))

(setf (get real_test::AddEyeToHandSampleRequest :md5sum-) "b1d912318536383476f3a61d6d9c6b77")
(setf (get real_test::AddEyeToHandSampleRequest :datatype-) "real_test/AddEyeToHandSampleRequest")
(setf (get real_test::AddEyeToHandSampleRequest :definition-)
      "# 手动增加一对 3D 对应点：同一点在基座系与相机光学系下的坐标（米）
float32 base_x
float32 base_y
float32 base_z
float32 optical_x
float32 optical_y
float32 optical_z
---
bool success
string message
int32 count

")

(setf (get real_test::AddEyeToHandSampleResponse :md5sum-) "b1d912318536383476f3a61d6d9c6b77")
(setf (get real_test::AddEyeToHandSampleResponse :datatype-) "real_test/AddEyeToHandSampleResponse")
(setf (get real_test::AddEyeToHandSampleResponse :definition-)
      "# 手动增加一对 3D 对应点：同一点在基座系与相机光学系下的坐标（米）
float32 base_x
float32 base_y
float32 base_z
float32 optical_x
float32 optical_y
float32 optical_z
---
bool success
string message
int32 count

")



(provide :real_test/AddEyeToHandSample "b1d912318536383476f3a61d6d9c6b77")



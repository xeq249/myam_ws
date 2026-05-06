
(cl:in-package :asdf)

(defsystem "real_test-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddEyeToHandSample" :depends-on ("_package_AddEyeToHandSample"))
    (:file "_package_AddEyeToHandSample" :depends-on ("_package"))
  ))

(cl:in-package :asdf)

(defsystem "joint_states_listener_real_jaco-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReturnJointStatesRealJaco" :depends-on ("_package_ReturnJointStatesRealJaco"))
    (:file "_package_ReturnJointStatesRealJaco" :depends-on ("_package"))
  ))
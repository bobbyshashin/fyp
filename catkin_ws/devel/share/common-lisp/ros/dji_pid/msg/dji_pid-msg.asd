
(cl:in-package :asdf)

(defsystem "dji_pid-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ekf_data" :depends-on ("_package_ekf_data"))
    (:file "_package_ekf_data" :depends-on ("_package"))
    (:file "pid_ctrl_data" :depends-on ("_package_pid_ctrl_data"))
    (:file "_package_pid_ctrl_data" :depends-on ("_package"))
    (:file "stick_data" :depends-on ("_package_stick_data"))
    (:file "_package_stick_data" :depends-on ("_package"))
    (:file "move_target" :depends-on ("_package_move_target"))
    (:file "_package_move_target" :depends-on ("_package"))
  ))
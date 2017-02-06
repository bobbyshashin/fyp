
(cl:in-package :asdf)

(defsystem "dji_api-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dji_sdk-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "api_velocity" :depends-on ("_package_api_velocity"))
    (:file "_package_api_velocity" :depends-on ("_package"))
    (:file "api_quaternion" :depends-on ("_package_api_quaternion"))
    (:file "_package_api_quaternion" :depends-on ("_package"))
    (:file "api_gimbal" :depends-on ("_package_api_gimbal"))
    (:file "_package_api_gimbal" :depends-on ("_package"))
    (:file "api_gps" :depends-on ("_package_api_gps"))
    (:file "_package_api_gps" :depends-on ("_package"))
    (:file "api_accelerate" :depends-on ("_package_api_accelerate"))
    (:file "_package_api_accelerate" :depends-on ("_package"))
    (:file "api_wrench" :depends-on ("_package_api_wrench"))
    (:file "_package_api_wrench" :depends-on ("_package"))
    (:file "api_magnetic" :depends-on ("_package_api_magnetic"))
    (:file "_package_api_magnetic" :depends-on ("_package"))
    (:file "api_ctrl_data" :depends-on ("_package_api_ctrl_data"))
    (:file "_package_api_ctrl_data" :depends-on ("_package"))
    (:file "api_rc" :depends-on ("_package_api_rc"))
    (:file "_package_api_rc" :depends-on ("_package"))
    (:file "api_fc_data" :depends-on ("_package_api_fc_data"))
    (:file "_package_api_fc_data" :depends-on ("_package"))
  ))
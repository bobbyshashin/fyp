
(cl:in-package :asdf)

(defsystem "bluefox2-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetExposeSrv" :depends-on ("_package_SetExposeSrv"))
    (:file "_package_SetExposeSrv" :depends-on ("_package"))
  ))
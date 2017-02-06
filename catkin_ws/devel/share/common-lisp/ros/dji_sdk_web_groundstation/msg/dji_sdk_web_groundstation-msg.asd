
(cl:in-package :asdf)

(defsystem "dji_sdk_web_groundstation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :dji_sdk-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MapNavSrvCmd" :depends-on ("_package_MapNavSrvCmd"))
    (:file "_package_MapNavSrvCmd" :depends-on ("_package"))
    (:file "WebWaypointReceiveActionFeedback" :depends-on ("_package_WebWaypointReceiveActionFeedback"))
    (:file "_package_WebWaypointReceiveActionFeedback" :depends-on ("_package"))
    (:file "WebWaypointReceiveActionGoal" :depends-on ("_package_WebWaypointReceiveActionGoal"))
    (:file "_package_WebWaypointReceiveActionGoal" :depends-on ("_package"))
    (:file "WebWaypointReceiveActionResult" :depends-on ("_package_WebWaypointReceiveActionResult"))
    (:file "_package_WebWaypointReceiveActionResult" :depends-on ("_package"))
    (:file "WebWaypointReceiveFeedback" :depends-on ("_package_WebWaypointReceiveFeedback"))
    (:file "_package_WebWaypointReceiveFeedback" :depends-on ("_package"))
    (:file "WebWaypointReceiveResult" :depends-on ("_package_WebWaypointReceiveResult"))
    (:file "_package_WebWaypointReceiveResult" :depends-on ("_package"))
    (:file "WebWaypointReceiveGoal" :depends-on ("_package_WebWaypointReceiveGoal"))
    (:file "_package_WebWaypointReceiveGoal" :depends-on ("_package"))
    (:file "WebWaypointReceiveAction" :depends-on ("_package_WebWaypointReceiveAction"))
    (:file "_package_WebWaypointReceiveAction" :depends-on ("_package"))
  ))
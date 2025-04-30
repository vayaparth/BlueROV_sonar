
(cl:in-package :asdf)

(defsystem "ping_sonar-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sendingSonarConfig" :depends-on ("_package_sendingSonarConfig"))
    (:file "_package_sendingSonarConfig" :depends-on ("_package"))
  ))
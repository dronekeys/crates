
(cl:in-package :asdf)

(defsystem "hal_sensor_imu-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Configure" :depends-on ("_package_Configure"))
    (:file "_package_Configure" :depends-on ("_package"))
  ))
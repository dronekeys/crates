
(cl:in-package :asdf)

(defsystem "sim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Reset" :depends-on ("_package_Reset"))
    (:file "_package_Reset" :depends-on ("_package"))
    (:file "Resume" :depends-on ("_package_Resume"))
    (:file "_package_Resume" :depends-on ("_package"))
    (:file "Noise" :depends-on ("_package_Noise"))
    (:file "_package_Noise" :depends-on ("_package"))
    (:file "Delete" :depends-on ("_package_Delete"))
    (:file "_package_Delete" :depends-on ("_package"))
    (:file "Step" :depends-on ("_package_Step"))
    (:file "_package_Step" :depends-on ("_package"))
    (:file "Seed" :depends-on ("_package_Seed"))
    (:file "_package_Seed" :depends-on ("_package"))
    (:file "Pause" :depends-on ("_package_Pause"))
    (:file "_package_Pause" :depends-on ("_package"))
    (:file "Insert" :depends-on ("_package_Insert"))
    (:file "_package_Insert" :depends-on ("_package"))
  ))
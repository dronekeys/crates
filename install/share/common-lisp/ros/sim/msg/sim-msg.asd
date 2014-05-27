
(cl:in-package :asdf)

(defsystem "sim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Contact" :depends-on ("_package_Contact"))
    (:file "_package_Contact" :depends-on ("_package"))
    (:file "Contacts" :depends-on ("_package_Contacts"))
    (:file "_package_Contacts" :depends-on ("_package"))
  ))
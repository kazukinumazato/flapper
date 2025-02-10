
(cl:in-package :asdf)

(defsystem "flapper-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Rpm" :depends-on ("_package_Rpm"))
    (:file "_package_Rpm" :depends-on ("_package"))
    (:file "Stabilizer" :depends-on ("_package_Stabilizer"))
    (:file "_package_Stabilizer" :depends-on ("_package"))
  ))
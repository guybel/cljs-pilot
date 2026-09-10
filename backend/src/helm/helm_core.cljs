(ns helm.core
  (:require [helm.values :as v]))

;; A appeler au demarrage, avant server/start!

(defn register-values! []
  ;; Active/desactive le pilote automatique
  (v/boolean-property! "ap.enabled" false)

  ;; Cap cible (0-360 degres), modifiable directement ou via l'increment
  ;; ap.heading_command.adjust (voir server.clj)
  (v/range-property! "ap.heading_command" 0 0 360)

  ;; Cap actuel mesure (lecture seule, mis a jour par ta boucle de nav/IMU)
  (v/sensor-value! "ap.heading" 0 :fmt "%.1f"))

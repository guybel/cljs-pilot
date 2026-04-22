(ns main
  (:require [helm.config :as config]
            [helm.server :as server]
            [helm.autopilot :as ap]
            [helm.servo :as servo]
            [helm.boatimu :as boatimu]
            [helm.signalk :as signalk]))

;; ---------------------------------------------------------------------------
;; Démarrage
;; ---------------------------------------------------------------------------

(defn -main []
  (js/console.log "helmpilot backend démarrage (ClojureScript / nbb)")

  ;; 1. Lecture de config.edn
  (config/load!)

  ;; 2. Autopilot + pilotes (gains depuis config)
  (ap/init! {:gains (config/get-cfg :gains)})

  ;; 3. Serveur TCP + WebSocket
  (server/start! (config/get-cfg :server))

  ;; 4. Servo Arduino
  (servo/start! (config/get-cfg :servo))

  ;; 5. Source(s) IMU selon :imu :source dans config.edn
  ;;    :boatimu → MPU-9250 I2C (simulation si pas de matériel)
  ;;    :signalk → serveur Signal K réseau
  ;;    :both    → les deux ; Signal K prend le dessus si connecté
  (let [src (config/get-cfg :imu :source)]
    (when (#{:boatimu :both} src)
      (boatimu/start! ap/on-imu-data!))
    (when (#{:signalk :both} src)
      (signalk/start! ap/on-imu-data! (config/get-cfg :signalk))))

  (js/console.log (str "Prêt. Connexion : nc localhost " (config/get-cfg :server :port))))

(-main)

(ns helm.servo
  (:require [helm.values :as v]
            ["mqtt" :as mqtt]))

;; ============================================================================
;; Servo Motor Control — MQTT (ESP32 S3) or Serial (Arduino)
;; ============================================================================
;;
;; Backend Node.js (ClojureScript compile pour Node, pas pour le navigateur) :
;; on parle MQTT en TCP brut directement sur le port 1883, pas besoin de
;; WebSocket ni du paquet npm mqtt expose comme global `js/mqtt` - ici on
;; requiert directement le module npm `mqtt` (a installer : `npm i mqtt`).
;;
;; Le firmware ESP32-S3 du verin ecoute en MQTT (voir verin_controller_mqtt.py) :
;;   - verin/cmd/motor    <- raw value [0, 1023], 511 = centre/stop
;;   - verin/cmd/enabled  <- "true" / "false" (doit etre "true" pour que le
;;                           verin reagisse aux commandes moteur - sinon le
;;                           firmware applique hard_stop() en continu)
;;   - verin/status       -> "ON" / "OFF" / "FAULT" (publie par l'ESP32)

(defonce state
  (atom {:connected               false
         :last-speed              0
         :neutral-zone-threshold  20
         :mode                    nil
         :client                  nil
         :broker-url              nil
         :motor-topic             "verin/cmd/motor"
         :enabled-topic           "verin/cmd/enabled"
         :status-topic            "verin/status"}))

;; ---------------------------------------------------------------------------
;; Registre des valeurs
;; ---------------------------------------------------------------------------

(defn register-values! []
  (v/range-property! "servo.command" 511 0 1023 :persistent? true)
  (v/boolean-value!  "servo.connected" false)
  (v/string-value!   "servo.mode"      "unknown")
  (v/string-value!   "servo.esp32_status" "unknown"))

(defn speed->normalized [speed]
  (let [raw (max 0 (min 1023 (double speed)))]
    (- (/ raw 511.5) 1.0)))

;; ---------------------------------------------------------------------------
;; Mode MQTT/WebSocket (ESP32)
;; ---------------------------------------------------------------------------

(defn- publish! [topic payload]
  (when-let [client (:client @state)]
    (.publish client topic (str payload))))

(defn mqtt:enable! []
  (publish! (:enabled-topic @state) "true"))

(defn mqtt:disable! []
  (publish! (:enabled-topic @state) "false"))

(defn mqtt:send-command!
  "Envoie une commande normalisée cmd ∈ [-1, 1] au vérin via MQTT.
   Convertit en speed [0, 1023].
   Comportement aligné sur la logique de référence : on ignore la zone neutre
   autour de 511 ± 25 et on ne renvoie pas les valeurs dupliquées / micro-corrections."
  [cmd]
  (when (:connected @state)
    (let [clamped           (max -1.0 (min 1.0 (double cmd)))
          speed             (int (* (+ clamped 1.0) 511.5))
          centered          (js/Math.abs (- speed 511))
          last-speed        (:last-speed @state)
          neutral-threshold (:neutral-zone-threshold @state)]
      (cond
        (= last-speed speed)
        (js/console.debug "[servo:mqtt] Ignoring duplicate motor speed:" speed)

        ;; Le stop exact au centre (speed=511, ex: OFF ou reset-ap-output!) doit
        ;; TOUJOURS partir, meme si l'ecart au centre est sous le seuil de zone
        ;; neutre - sinon un arret volontaire peut etre silencieusement ignore
        ;; et le moteur continue de tourner sur la derniere commande active
        ;; jusqu'au watchdog reseau de l'ESP32 (delai de plusieurs secondes).
        (and (< centered neutral-threshold) (not= speed 511))
        (js/console.debug "[servo:mqtt] Ignoring neutral micro-movement: raw=" speed "(centered=" centered ")")

        :else
        (do
          (swap! state assoc :last-speed speed)
          (v/update-value! "servo.command" speed)
          (publish! (:motor-topic @state) speed)
          (js/console.log "[servo:mqtt] Motor speed:" speed))))))

(defn mqtt:stop! []
  (when (:connected @state)
    (mqtt:send-command! 0)
    (mqtt:disable!)))

(defn- handle-message [topic payload]
  (let [topic-str (if (string? topic) topic (.toString topic))
        msg       (.toString payload)]
    (when (= topic-str (:status-topic @state))
      (v/update-value! "servo.esp32_status" msg)
      (when (= msg "FAULT")
        (js/console.warn "[servo:mqtt] ESP32 reporte FAULT (surintensite/butee)")))))

(defn mqtt:start! [ws-url & [{:keys [motor-topic enabled-topic status-topic]}]]
  (register-values!)
  (swap! state assoc
         :ws-url ws-url
         :motor-topic    (or motor-topic    (:motor-topic @state))
         :enabled-topic  (or enabled-topic  (:enabled-topic @state))
         :status-topic   (or status-topic   (:status-topic @state)))

  (if-not (exists? js/mqtt)
    (do
      (js/console.error "[servo:mqtt] La lib mqtt.js n'est pas chargee (global `mqtt` introuvable)")
      (swap! state assoc :connected false :mode nil)
      (v/update-value! "servo.connected" false)
      (v/update-value! "servo.mode" "offline"))
    (let [client (.connect js/mqtt ws-url)]
      (swap! state assoc :client client)

      (.on client "connect"
           (fn []
             (js/console.log (str "[servo:mqtt] Connecté → " ws-url))
             (swap! state assoc :connected true :mode :mqtt)
             (v/update-value! "servo.connected" true)
             (v/update-value! "servo.mode" "mqtt")
             (.subscribe client (:status-topic @state))
             (mqtt:enable!)
             (mqtt:send-command! 0)))

      (.on client "reconnect"
           (fn []
             (js/console.log "[servo:mqtt] Reconnexion en cours...")))

      (.on client "close"
           (fn []
             (js/console.warn "[servo:mqtt] Connexion fermée")
             (swap! state assoc :connected false)
             (v/update-value! "servo.connected" false)
             (v/update-value! "servo.mode" "offline")))

      (.on client "error"
           (fn [e]
             (js/console.error "[servo:mqtt] Erreur:" (.-message e))
             (swap! state assoc :connected false)
             (v/update-value! "servo.connected" false)
             (v/update-value! "servo.mode" "offline")))

      (.on client "message"
           (fn [topic payload]
             (handle-message topic payload))))))

;; ---------------------------------------------------------------------------
;; Mode Serial (Arduino) — Legacy support
;; ---------------------------------------------------------------------------

(defn serial:send-command! [cmd]
  (when (:connected @state)
    (js/console.warn "[servo:serial] Not implemented yet")))

(defn serial:start! [port baud]
  (js/console.log (str "[servo:serial] Port " port " @" baud " baud (not yet implemented)")))

;; ---------------------------------------------------------------------------
;; Interface unifiée
;; ---------------------------------------------------------------------------

(defn send-command! [cmd]
  (case (:mode @state)
    :mqtt   (mqtt:send-command! cmd)
    :serial (serial:send-command! cmd)
    (js/console.warn "[servo] Not connected")))

(defn set-command! [speed]
  (let [raw (int (js/Math.round (max 0 (min 1023 (double speed)))))]
    (v/set-value! "servo.command" raw)
    (send-command! (speed->normalized raw))))

(defn stop-servo! []
  (case (:mode @state)
    :mqtt   (mqtt:stop!)
    :serial nil
    (js/console.warn "[servo] Not connected")))

;; ---------------------------------------------------------------------------
;; Startup
;; ---------------------------------------------------------------------------

(defn start! [servo-cfg]
  (register-values!)

  (let [servo-type (or (:type servo-cfg) :serial)]

    (case servo-type
      :mqtt
      (do
        (js/console.log "[servo] Mode: MQTT/WebSocket (ESP32 S3)")
        (mqtt:start! (:ws-url servo-cfg) servo-cfg))

      (js/console.error "[servo] Unknown servo type:" servo-type))))

(defn stop! []
  (stop-servo!)
  (when-let [client (:client @state)]
    (.end client))
  (swap! state assoc :connected false :mode nil :client nil)
  (v/update-value! "servo.connected" false))

;; ---------------------------------------------------------------------------
;; Debug / REPL
;; ---------------------------------------------------------------------------

(defn status []
  @state)

(defn test-speed! [speed]
  (if (and (= :mqtt (:mode @state)) (:connected @state))
    (do
      (publish! (:motor-topic @state) speed)
      (js/console.log "[servo:test] Published speed" speed))
    (js/console.warn "[servo] Not in MQTT mode or not connected")))
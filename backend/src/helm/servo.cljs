(ns helm.servo
  (:require [helm.values :as v]
            [helm.config :as config]))

;; ============================================================================
;; Servo Motor Control — WiFi (ESP32 S3) or Serial (Arduino)
;; ============================================================================
;; Support dual-mode :
;; - :wifi  → HTTP GET http://192.168.42.10/motor/<speed>
;; - :serial → [deprecated] SerialPort Arduino (legacy)
;; ============================================================================

(defonce state
  (atom {:connected false
         :last-speed 0
         :mode nil}))

;; ---------------------------------------------------------------------------
;; Registre des valeurs
;; ---------------------------------------------------------------------------

(defn register-values! []
  (v/sensor-value!  "servo.command"   0)
  (v/boolean-value! "servo.connected" false)
  (v/string-value!  "servo.mode"      "unknown"))

;; ---------------------------------------------------------------------------
;; Mode WiFi (ESP32)
;; ---------------------------------------------------------------------------

(defn wifi:send-command!
  "Envoie une commande normalisée cmd ∈ [-1, 1] à l'ESP32.
   Convertit en speed [0, 1023]."
  [url cmd]
  (when (:connected @state)
    (let [clamped (max -1.0 (min 1.0 (double cmd)))
          speed   (int (* (+ clamped 1.0) 511.5))]  ; [-1, 1] → [0, 1023]
      (swap! state assoc :last-speed speed)
      (v/update-value! "servo.command" speed)

      ;; Appel HTTP asynchrone à l'ESP32
      (-> (js/fetch (str url "/motor/" speed))
          (.then #(.text %))
          (.then (fn [response]
                   (js/console.log "[servo:wifi] Motor speed:" speed "→" response)))
          (.catch (fn [e]
                    (js/console.error "[servo:wifi] HTTP error:" (.-message e))))))))

(defn wifi:stop! []
  (when (:connected @state)
    (wifi:send-command! (-> @config/data :servo :url) 0)))

(defn wifi:start! [url]
  "Démarre la connexion à l'ESP32 WiFi.
   url : http://192.168.42.10"
  (register-values!)

  ;; Test de connexion
  (-> (js/fetch (str url "/motor/0"))
      (.then #(.text %))
      (.then (fn [response]
               (js/console.log (str "[servo:wifi] Connecté → " url))
               (swap! state assoc :connected true :mode :wifi)
               (v/update-value! "servo.connected" true)
               (v/update-value! "servo.mode" "wifi")))
      (.catch (fn [e]
                (js/console.error (str "[servo:wifi] Impossible de joindre " url ": " (.-message e)))
                (swap! state assoc :connected false :mode nil)
                (v/update-value! "servo.connected" false)
                (v/update-value! "servo.mode" "offline")))))

;; ---------------------------------------------------------------------------
;; Mode Serial (Arduino) — Legacy support
;; ---------------------------------------------------------------------------

(defn serial:send-command! [cmd]
  "Envoie cmd ∈ [-1, 1] au servo Arduino (legacy)."
  (when (:connected @state)
    (js/console.warn "[servo:serial] Not implemented yet")))

(defn serial:start! [port baud]
  "Démarre la connexion série Arduino (legacy - non utilisé)."
  (js/console.log (str "[servo:serial] Port " port " @" baud " baud (not yet implemented)")))

;; ---------------------------------------------------------------------------
;; Interface unifiée
;; ---------------------------------------------------------------------------

(defn send-command!
  "Envoie une commande normalisée cmd ∈ [-1, 1] au moteur."
  [cmd]
  (case (:mode @state)
    :wifi   (wifi:send-command! (-> @config/data :servo :url) cmd)
    :serial (serial:send-command! cmd)
    (js/console.warn "[servo] Not connected")))

(defn stop-servo! []
  (case (:mode @state)
    :wifi   (wifi:stop!)
    :serial nil  ; serial:stop!
    (js/console.warn "[servo] Not connected")))

;; ---------------------------------------------------------------------------
;; Startup — détecte le mode depuis la config
;; ---------------------------------------------------------------------------

(defn start!
  "Démarre le servo selon la config : :wifi ou :serial"
  []
  (register-values!)

  (let [servo-cfg (:servo @config/data)
        servo-type (or (:type servo-cfg) :serial)]  ; défaut :serial (legacy)

    (case servo-type
      :wifi
      (do
        (js/console.log "[servo] Mode: WiFi (ESP32 S3)")
        (wifi:start! (:url servo-cfg)))

      :serial
      (do
        (js/console.log "[servo] Mode: Serial (Arduino) - deprecated")
        (serial:start! (:port servo-cfg) (:baud servo-cfg)))

      (js/console.error "[servo] Unknown servo type:" servo-type))))

(defn stop! []
  (stop-servo!)
  (swap! state assoc :connected false :mode nil)
  (v/update-value! "servo.connected" false))

;; ---------------------------------------------------------------------------
;; Debug / REPL
;; ---------------------------------------------------------------------------

(defn status []
  "Retourne l'état du servo"
  @state)

(defn test-speed! [speed]
  "Test : envoie une vitesse brute [0, 1023]"
  (let [url (-> @config/data :servo :url)]
    (if (= :wifi (:mode @state))
      (-> (js/fetch (str url "/motor/" speed))
          (.then #(.text %))
          (.then #(js/console.log "[servo:test]" %)))
      (js/console.warn "[servo] Not in WiFi mode"))))
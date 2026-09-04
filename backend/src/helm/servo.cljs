(ns helm.servo
  (:require [helm.values :as v]))

;; ============================================================================
;; Servo Motor Control — WiFi (ESP32 S3) or Serial (Arduino)
;; ============================================================================

(defonce state
  (atom {:connected false
         :last-speed 0
         :mode nil
         :url nil}))

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
          speed   (int (* (+ clamped 1.0) 511.5))]
      (swap! state assoc :last-speed speed)
      (v/update-value! "servo.command" speed)

      (-> (js/fetch (str url "/motor/" speed))
          (.then #(.text %))
          (.then (fn [response]
                   (js/console.log "[servo:wifi] Motor speed:" speed "→" response)))
          (.catch (fn [e]
                    (js/console.error "[servo:wifi] HTTP error:" (.-message e))))))))

(defn wifi:stop! []
  (when (:connected @state)
    (wifi:send-command! (:url @state) 0)))

(defn wifi:start! [url]
  (register-values!)
  (swap! state assoc :url url)

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
  (when (:connected @state)
    (js/console.warn "[servo:serial] Not implemented yet")))

(defn serial:start! [port baud]
  (js/console.log (str "[servo:serial] Port " port " @" baud " baud (not yet implemented)")))

;; ---------------------------------------------------------------------------
;; Interface unifiée
;; ---------------------------------------------------------------------------

(defn send-command! [cmd]
  (case (:mode @state)
    :wifi   (wifi:send-command! (:url @state) cmd)
    :serial (serial:send-command! cmd)
    (js/console.warn "[servo] Not connected")))

(defn stop-servo! []
  (case (:mode @state)
    :wifi   (wifi:stop!)
    :serial nil
    (js/console.warn "[servo] Not connected")))

;; ---------------------------------------------------------------------------
;; Startup
;; ---------------------------------------------------------------------------

(defn start! [servo-cfg]
  (register-values!)

  (let [servo-type (or (:type servo-cfg) :serial)]

    (case servo-type
      :wifi
      (do
        (js/console.log "[servo] Mode: WiFi (ESP32 S3)")
        (wifi:start! (:url servo-cfg)))
      
      (js/console.error "[servo] Unknown servo type:" servo-type))))

(defn stop! []
  (stop-servo!)
  (swap! state assoc :connected false :mode nil)
  (v/update-value! "servo.connected" false))

;; ---------------------------------------------------------------------------
;; Debug / REPL
;; ---------------------------------------------------------------------------

(defn status []
  @state)

(defn test-speed! [speed]
  (let [url (:url @state)]
    (if (and (= :wifi (:mode @state)) url)
      (-> (js/fetch (str url "/motor/" speed))
          (.then #(.text %))
          (.then #(js/console.log "[servo:test]" %)))
      (js/console.warn "[servo] Not in WiFi mode or URL not set"))))
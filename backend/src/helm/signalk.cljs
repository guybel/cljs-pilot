(ns helm.signalk
  (:require [helm.values :as v]))

;; Client Signal K WebSocket — source de données de test.
;;
;; Paths Signal K consommés :
;;   navigation.attitude            → pitch, roll, yaw (rad)
;;   navigation.headingMagnetic     → cap magnétique (rad)
;;   navigation.headingTrue         → cap vrai (rad)
;;   navigation.rateOfTurn          → taux de virage (rad/s)
;;   navigation.position            → lat/lon
;;   navigation.speedOverGround     → vitesse sol (m/s)
;;   navigation.courseOverGroundTrue→ route fond (rad)
;;
;; Appelle on-data-fn avec {:heading :headingrate :headingraterate :pitch :roll}
;; — même interface que boatimu/start! — pour que l'autopilot fonctionne tel quel.

(def RAD2DEG (/ 180.0 Math/PI))

;; ---------------------------------------------------------------------------
;; Enregistrement des valeurs
;; ---------------------------------------------------------------------------

(defn register-values!
  [{:keys [host port enabled] :or {host "localhost" port 3000 enabled false}}]
  (v/property!         "signalk.host"    host    :persistent? true)
  (v/range-property!   "signalk.port"    port 1 65535 :persistent? true)
  (v/boolean-property! "signalk.enabled" enabled :persistent? true)
  ;; Lecture seule
  (v/boolean-value!  "signalk.connected"  false)
  (v/sensor-value!   "signalk.heading"    false :directional true)
  (v/sensor-value!   "signalk.pitch"      false)
  (v/sensor-value!   "signalk.roll"       false)
  (v/sensor-value!   "signalk.heel"       false)
  (v/sensor-value!   "signalk.headingrate" false)
  ;; GPS
  (v/sensor-value!   "gps.lat"   false :fmt "%.6f")
  (v/sensor-value!   "gps.lon"   false :fmt "%.6f")
  (v/sensor-value!   "gps.speed" false)
  (v/sensor-value!   "gps.track" false :directional true))

;; ---------------------------------------------------------------------------
;; État interne
;; ---------------------------------------------------------------------------

(defonce state
  (atom {:ws               nil
         :reconnect-timer  nil
         :on-data-fn       nil
         ;; Dernières valeurs Signal K reçues
         :attitude         nil    ; {:roll :pitch :yaw} en radians
         :heading-mag      nil    ; degrés
         :heading-true     nil    ; degrés
         :rate-of-turn     nil    ; °/s
         ;; Pour headingraterate
         :last-headingrate  0
         :last-rate-t       0
         ;; Gîte filtrée (IIR lent du roll)
         :heel              0}))

;; ---------------------------------------------------------------------------
;; Traitement des deltas
;; ---------------------------------------------------------------------------

(defn- normalize-heading [h]
  (let [h (mod h 360)]
    (if (< h 0) (+ h 360) h)))

(defn- set-attitude-component! [component value]
  (swap! state update :attitude #(assoc (or % {}) component value)))

(defn- dispatch-path! [path value]
  (js/console.log (str "[SignalK] Received path: " path " = " (pr-str value)))
  (case path
    "navigation.attitude"
    (swap! state assoc :attitude
           {:roll  (.-roll  value)
            :pitch (.-pitch value)
            :yaw   (.-yaw   value)})

    "navigation.attitude.roll"
    (set-attitude-component! :roll value)

    "navigation.attitude.pitch"
    (set-attitude-component! :pitch value)

    "navigation.attitude.yaw"
    (set-attitude-component! :yaw value)

    "navigation.headingMagnetic"
    (swap! state assoc :heading-mag (normalize-heading (* value RAD2DEG)))

    "navigation.headingTrue"
    (swap! state assoc :heading-true (normalize-heading (* value RAD2DEG)))

    "navigation.rateOfTurn"
    (swap! state assoc :rate-of-turn (* value RAD2DEG))

    "navigation.position"
    (do (v/update-value! "gps.lat" (.-latitude  value))
        (v/update-value! "gps.lon" (.-longitude value)))

    "navigation.speedOverGround"
    (v/update-value! "gps.speed" (* value 1.94384))   ; m/s → nœuds

    "navigation.courseOverGroundTrue"
    (v/update-value! "gps.track" (normalize-heading (* value RAD2DEG)))

    nil))  ; path inconnu, ignoré

(defn- build-imu-data []
  ;; Heading : préférer headingMagnetic, sinon headingTrue, sinon attitude.yaw
  (let [{:keys [attitude heading-mag heading-true rate-of-turn
                last-headingrate last-rate-t heel]} @state
        now       (js/Date.now)
        heading   (or heading-mag
                      heading-true
                      (when attitude
                        (normalize-heading (* (:yaw attitude) RAD2DEG))))
        pitch     (if attitude (* (:pitch attitude) RAD2DEG) 0)
        roll      (if attitude (* (:roll  attitude) RAD2DEG) 0)
        headingrate (or rate-of-turn 0)

        ;; headingraterate — dérivée du taux de virage
        dt               (/ (- now last-rate-t) 1000.0)
        headingraterate  (if (and (pos? dt) (< dt 0.5))
                           (/ (- headingrate last-headingrate) dt)
                           0)
        ;; Gîte : IIR très lent du roll (identique à boatimu)
        new-heel         (+ (* 0.03 roll) (* 0.97 heel))]

    (swap! state assoc
           :last-headingrate headingrate
           :last-rate-t      now
           :heel             new-heel)

    (when heading
      {:heading         (normalize-heading heading)
       :headingrate     headingrate
       :headingraterate headingraterate
       :pitch           pitch
       :roll            roll
       :heel            new-heel})))

(defn- process-message! [raw-msg]
  (try
    (let [delta (js/JSON.parse raw-msg)]
      (when (.-updates delta)
        (doseq [upd (array-seq (.-updates delta))]
          (when (.-values upd)
            (doseq [entry (array-seq (.-values upd))]
              (dispatch-path! (.-path entry) (.-value entry)))))

        ;; Émettre les données synthétisées si on a un heading
        (when-let [on-data (:on-data-fn @state)]
          (when-let [imu (build-imu-data)]
            (v/update-value! "signalk.heading"     (:heading     imu))
            (v/update-value! "signalk.headingrate" (:headingrate imu))
            (v/update-value! "signalk.pitch"       (:pitch       imu))
            (v/update-value! "signalk.roll"        (:roll        imu))
            (v/update-value! "signalk.heel"        (:heel        imu))
            (on-data imu)))))
    (catch :default e
      (js/console.error "[SignalK] Erreur parse:" (.-message e)))))

;; ---------------------------------------------------------------------------
;; WebSocket
;; ---------------------------------------------------------------------------

(declare connect!)

(defn- schedule-reconnect! [on-data-fn cfg]
  (when-not (:reconnect-timer @state)
    (let [t (js/setTimeout
             (fn []
               (swap! state assoc :reconnect-timer nil)
               (connect! on-data-fn cfg))
             5000)]
      (swap! state assoc :reconnect-timer t))))

(def ^:private subscribe-msg
  (js/JSON.stringify
   (clj->js
    {:context "vessels.self"
     :subscribe [{:path "*"}]})))

(defn connect! [on-data-fn cfg]
  (let [{:keys [host port url scheme]} cfg
        host   (or host (v/get-value "signalk.host") "localhost")
        port   (or port (v/get-value "signalk.port") 3000)
        scheme (or scheme "ws")
        url    (or url (str scheme "://" host ":" port "/signalk/v1/stream?subscribe=none"))]
    (try
      (let [WS (js/require "ws")
            ws (new WS url #js {:handshakeTimeout 10000
                                :rejectUnauthorized false})]
        (swap! state assoc :ws ws :on-data-fn on-data-fn)

        (.on ws "open"
             (fn []
               (js/console.log (str "[SignalK] Connecté → " url))
               (v/update-value! "signalk.connected" true)
               (.send ws subscribe-msg)))

        (.on ws "message"
             (fn [data]
               (process-message! (str data))))

        (.on ws "close"
             (fn [code]
               (js/console.log (str "[SignalK] Déconnecté (" code ") — reconnexion dans 5 s"))
               (v/update-value! "signalk.connected" false)
               (swap! state assoc :ws nil)
               (schedule-reconnect! on-data-fn cfg)))

        (.on ws "error"
             (fn [e]
               ;; L'évènement "close" suit toujours "error" — pas besoin de schedule ici
               (js/console.error "[SignalK] Erreur:" (.-message e)))))

      (catch :default e
        (js/console.error "[SignalK] Impossible de charger 'ws':" (.-message e))
        (js/console.error "[SignalK] Essayez : npm install ws")))))

;; ---------------------------------------------------------------------------
;; API publique
;; ---------------------------------------------------------------------------

(defn start!
  "Enregistre les valeurs signalk.* et gps.*, puis se connecte au serveur Signal K.
   on-data-fn : même interface que boatimu/start! — {:heading :headingrate :headingraterate :pitch :roll}
   cfg : map optionnelle {:host ... :port ... :enabled ...} depuis config/get-cfg :signalk"
  ([on-data-fn]      (start! on-data-fn {}))
  ([on-data-fn cfg]
   (register-values! cfg)
   (if (v/get-value "signalk.enabled")
     (do
       (js/console.log
        (str "[SignalK] Démarrage → "
             (v/get-value "signalk.host") ":" (v/get-value "signalk.port")))
       (connect! on-data-fn cfg))
     (js/console.log "[SignalK] Désactivé (signalk.enabled=false)"))))

(defn stop! []
  (when-let [t (:reconnect-timer @state)]
    (js/clearTimeout t))
  (when-let [ws (:ws @state)]
    (.close ws))
  (swap! state assoc :ws nil :reconnect-timer nil :on-data-fn nil)
  (v/update-value! "signalk.connected" false))

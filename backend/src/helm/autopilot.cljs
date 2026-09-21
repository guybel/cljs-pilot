(ns helm.autopilot
  (:require [helm.values :as v]
            [helm.pilots.pilot :as pilot]
            [helm.pilots.basic :as basic]
            [helm.servo :as servo]
            [utils.resolv :refer [resolv]]))

;; Port de pypilot/autopilot.py
;; Boucle principale déclenchée par les données IMU (20Hz)

;; ---------------------------------------------------------------------------
;; Helpers
;; ---------------------------------------------------------------------------

(defn- minmax [x r] (min (max x (- r)) r))

;; Delai minimum entre deux inversions de sens du verin, pour eviter de
;; stresser mecaniquement l'actionneur sur des commandes qui oscillent vite.
(def ^:private min-reversal-ms 150)

;; ---------------------------------------------------------------------------
;; État interne (mutable via atom)
;; ---------------------------------------------------------------------------

(defonce state
  (atom {:heading-error-int      0
         :heading-error-int-time 0
         :heading-command-rate   0
         :heading-command-prev   nil
         :heading-command-time   0
         :last-heading           nil
         :last-enabled            false
         :pilot-armed             false
         :pilots                  {}
         :initialized?            false
         ;; ---- CHAMPS SERVO (windup / deadband) ----
         ;; Verin sans retour de position (2 fils, pas de potentiometre/encodeur).
         ;; En dessous de :servo-min-speed, une commande continue est trop
         ;; faible pour vaincre la stiction et ne fait rien bouger : on la
         ;; convertit en impulsions a vitesse minimale via l'integrateur
         ;; :servo-windup plutot que d'envoyer un signal inefficace.
         :servo-windup             0.0
         :servo-windup-change      0
         :servo-last-speed         0.0
         :servo-last-time          0.0
         :servo-period             0.4   ; fenetre d'integration de l'effort (s)
         :servo-min-speed          0.05  ; A CALIBRER: vitesse mini qui bouge reellement le verin (voir test ESP32)
         :servo-max-speed          1.0}))

;; ---------------------------------------------------------------------------
;; Initialisation des valeurs serveur
;; ---------------------------------------------------------------------------

(defn init!
  "Enregistre toutes les valeurs ap.* dans le registre values et crée les pilotes.
   opts : {:gains {:P 0.003 :D 0.09 ...}} — défauts si omis."
  ([] (init! {}))
  ([{:keys [gains]}]
   (when-not (:initialized? @state)
     ;; Valeurs de mode et cap
     (v/enum-property!    "ap.mode"            "compass"
                          ["compass" "gps" "nav" "wind" "true wind"]
                          :persistent? true)
     (v/range-property!   "ap.heading_command" 0 -180 360 :persistent? true)
     (v/boolean-property! "ap.enabled"         false)
     (v/sensor-value!     "ap.heading"         0 :directional true)
     (v/sensor-value!     "ap.heading_error"   0)
     (v/sensor-value!     "ap.heading_error_int" 0)
     (v/sensor-value!     "ap.heading_command_rate" 0)

     ;; Offsets GPS/vent
     (v/sensor-value! "ap.gps_compass_offset"        0)
     (v/sensor-value! "ap.wind_compass_offset"        0)
     (v/sensor-value! "ap.true_wind_compass_offset"   0)

     ;; Modes disponibles (liste JSON)
     (v/json-value!  "ap.modes"      ["compass"])
     (v/enum-property! "ap.pilot"    "basic" ["basic"] :persistent? true)

     ;; Valeur interne de commande du pilote sélectionné
     (v/sensor-value! "ap.pilot.basic.command" 0)

     ;; Parametres du servo (verin sans retour de position) — reglables en live
     (v/range-property! "ap.servo_min_speed" (:servo-min-speed @state) 0 1    :persistent? true)
     (v/range-property! "ap.servo_max_speed" (:servo-max-speed @state) 0 1    :persistent? true)
     (v/range-property! "ap.servo_period"    (:servo-period @state)    0.05 2 :persistent? true)

     ;; Initialiser les pilotes
     (let [basic-pilot (basic/init! gains)]
       (swap! state assoc
              :pilots {:basic basic-pilot}
              :initialized? true
              :heading-error-int-time (js/Date.now))))))

;; ---------------------------------------------------------------------------
;; Calcul de heading_command_rate
;; ---------------------------------------------------------------------------

(defn- update-heading-command-rate! [now]
  (let [{:keys [heading-command-prev heading-command-time]} @state
        hc (v/get-value "ap.heading_command")]
    (if (nil? heading-command-prev)
      (swap! state assoc
             :heading-command-prev hc
             :heading-command-time now)
      (let [dt (/ (- now heading-command-time) 1000.0)]
        (when (and (pos? dt) (not= hc heading-command-prev))
          (let [rate (/ (resolv (- hc heading-command-prev)) dt)]
            (v/update-value! "ap.heading_command_rate" rate)
            (swap! state assoc
                   :heading-command-prev hc
                   :heading-command-time now
                   :heading-command-rate rate)))))))

;; ---------------------------------------------------------------------------
;; Windup servo : convertit une commande continue en commande reellement
;; efficace pour un verin sans retour de position
;; ---------------------------------------------------------------------------

(defn- apply-servo-windup!
  "cmd : commande normalisee du pilote, entre -1 et 1.
   now : timestamp ms (js/Date.now).
   Retourne la commande a envoyer a servo/send-command!.

   - |cmd| >= servo-min-speed  -> envoi direct (mode continu), borne a servo-max-speed
   - |cmd| <  servo-min-speed  -> accumulation dans servo-windup ; des que
     l'effort accumule depasse (servo-min-speed * servo-period), une
     impulsion courte a servo-min-speed est envoyee, sinon 0.
   - Une garde anti-inversion rapide empeche de changer de sens plus
     souvent que min-reversal-ms."
  [cmd now]
  (let [{:keys [servo-windup servo-last-time servo-last-speed
                servo-windup-change]} @state
        min-speed (v/get-value "ap.servo_min_speed")
        max-speed (v/get-value "ap.servo_max_speed")
        period    (v/get-value "ap.servo_period")
        dt        (if (pos? servo-last-time)
                    (min (/ (- now servo-last-time) 1000.0) period)
                    0)
        cmd       (minmax cmd max-speed)
        windup    (+ servo-windup (* cmd dt))
        threshold (* min-speed period)
        raw-output (cond
                     (>= (js/Math.abs cmd) min-speed)
                     cmd

                     (>= (js/Math.abs windup) threshold)
                     (* min-speed (if (pos? windup) 1 -1))

                     :else
                     0)
        reversing? (and (not= 0 raw-output)
                        (not= 0 servo-last-speed)
                        (not= (pos? raw-output) (pos? servo-last-speed)))
        output     (if (and reversing?
                            (< (- now servo-windup-change) min-reversal-ms))
                     0
                     raw-output)
        windup'    (cond
                     (>= (js/Math.abs cmd) min-speed)
                     0

                     (and (= output raw-output) (>= (js/Math.abs windup) threshold))
                     (- windup (* threshold (if (pos? windup) 1 -1)))

                     :else
                     windup)
        change-ms' (if reversing? now servo-windup-change)]
    (swap! state assoc
           :servo-windup        windup'
           :servo-last-speed    output
           :servo-last-time     now
           :servo-windup-change change-ms')
    output))

;; ---------------------------------------------------------------------------
;; Reset d’un cycle AP
;; ---------------------------------------------------------------------------

(defn- reset-ap-output! []
  (swap! state assoc
         :heading-error-int      0
         :heading-error-int-time (js/Date.now)
         :heading-command-rate   0
         :heading-command-prev   nil
         :heading-command-time   0
         :servo-windup           0.0
         :servo-windup-change    0
         :servo-last-speed       0.0
         :servo-last-time        0.0)
  (v/update-value! "ap.heading_error" 0)
  (v/update-value! "ap.heading_error_int" 0)
  (v/update-value! "ap.heading_command_rate" 0)
  (v/update-value! "servo.command" 511)
  (servo/send-command! 0))

(defn- sync-heading-command-to-current! []
  (let [heading (v/get-value "ap.heading")
        current (if (number? heading) heading 0)]
    (v/set-value! "ap.heading_command" current)
    (swap! state assoc
           :heading-command-prev current
           :heading-command-time (js/Date.now)
           :heading-command-rate 0
           :pilot-armed false)))

;; ---------------------------------------------------------------------------
;; Calcul de heading_error et heading_error_int
;; ---------------------------------------------------------------------------

(defn- update-heading-error! [now]
  (let [heading  (v/get-value "ap.heading")
        hc       (v/get-value "ap.heading_command")
        mode     (v/get-value "ap.mode")
        windmode (and mode (.includes mode "wind"))]
    (when (and (number? heading) (number? hc))
      (let [raw-err (-> (resolv (- heading hc))
                        (minmax 60))
            err     (let [e (if windmode (- raw-err) raw-err)]
                      (if (< (js/Math.abs e) 2) 0 e))]
        (v/update-value! "ap.heading_error" err)

        ;; Intégrale : dt limité à 1s, sans accumulation dans la zone morte ±2°.
        (let [{:keys [heading-error-int heading-error-int-time]} @state
              dt  (min (/ (- now heading-error-int-time) 1000.0) 1.0)
              new-int (minmax (+ heading-error-int (* (/ err 1500) dt)) 5)]
          (v/update-value! "ap.heading_error_int" new-int)
          (swap! state assoc
                 :heading-error-int      new-int
                 :heading-error-int-time now))))))

;; ---------------------------------------------------------------------------
;; Boucle principale : déclenchée par les données IMU
;; ---------------------------------------------------------------------------

(defn on-imu-data!
  "Appelé à chaque nouvelle donnée IMU (~20Hz), après traitement par boatimu.
   Les valeurs imu.* sont déjà à jour dans le registre.
   imu-data : {:heading :headingrate :headingraterate :pitch :roll}"
  [{:keys [heading headingrate headingraterate pitch roll]}]
  (let [now (js/Date.now)
        enabled (v/get-value "ap.enabled")
        was-enabled (:last-enabled @state)]
    (when-not enabled
      (when (or was-enabled
                (not= (v/get-value "ap.heading_error") 0)
                (not= (v/get-value "ap.heading_error_int") 0))
        (reset-ap-output!))
      (swap! state assoc :last-enabled false :pilot-armed false)
      (v/update-value! "ap.heading_error" 0)
      (v/update-value! "ap.heading_error_int" 0)
      (v/update-value! "ap.heading_command_rate" 0)
      nil)

    (when enabled
      (when (and enabled (not was-enabled))
        (reset-ap-output!)
        (sync-heading-command-to-current!))
      (swap! state assoc :last-enabled enabled)

      ;; Calcul du cap courant (ap.heading) selon le mode
      (pilot/compute-heading! nil)

      ;; Première frame après activation : on repart proprement du cap courant,
      ;; sans héritage d’ancienne commande ni d’erreur résiduelle.
      (when (and enabled (not (:pilot-armed @state)))
        (sync-heading-command-to-current!)
        (swap! state assoc :pilot-armed true))

      ;; Taux de changement du cap commandé
      (update-heading-command-rate! now)

      ;; Erreur de cap et intégrale
      (update-heading-error! now)

      ;; Appel du pilote sélectionné
      (let [pilot-name (v/get-value "ap.pilot")
            pilots     (:pilots @state)
            pilot      (get pilots (keyword pilot-name))
            ap-state   {:heading-error        (v/get-value "ap.heading_error")
                        :heading-error-int    (:heading-error-int @state)
                        :headingrate          headingrate
                        :headingraterate      headingraterate
                        :heading-command-rate (:heading-command-rate @state)}]
        (when pilot
          (let [cmd (basic/process! pilot ap-state)
                out (apply-servo-windup! cmd now)]
            (servo/send-command! out)))))))
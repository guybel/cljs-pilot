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
         :last-enabled           false
         :pilots                 {}
         :initialized?           false}))

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
    (v/sensor-value!     "ap.heading"         false :directional true)
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
;; Calcul de heading_error et heading_error_int
;; ---------------------------------------------------------------------------

(defn- update-heading-error! [now]
  (let [heading  (v/get-value "ap.heading")
        hc       (v/get-value "ap.heading_command")
        mode     (v/get-value "ap.mode")
        windmode (and mode (.includes mode "wind"))]
    (when (and heading hc)
      (let [err (-> (resolv (- heading hc))
                    (minmax 60)
                    (#(if windmode (- %) %)))]
        (v/update-value! "ap.heading_error" err)

        ;; Intégrale : dt limité à 1s
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
  (let [now (js/Date.now)]
    ;; Calcul du cap courant (ap.heading) selon le mode
    (pilot/compute-heading! nil)

    ;; Taux de changement du cap commandé
    (update-heading-command-rate! now)

    ;; Erreur de cap et intégrale
    (update-heading-error! now)

    ;; Appel du pilote sélectionné
    (let [enabled    (v/get-value "ap.enabled")
          pilot-name (v/get-value "ap.pilot")
          pilots     (:pilots @state)
          pilot      (get pilots (keyword pilot-name))
          ap-state   {:heading-error        (v/get-value "ap.heading_error")
                      :heading-error-int    (:heading-error-int @state)
                      :headingrate          headingrate
                      :headingraterate      headingraterate
                      :heading-command-rate (:heading-command-rate @state)}]
      (when pilot
        (let [cmd (basic/process! pilot ap-state)]
          (when enabled
            (v/update-value! "servo.command" cmd)
            (servo/send-command! cmd)))))))

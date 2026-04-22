(ns helm.pilots.pilot
  (:require [helm.values :as v]
            [utils.resolv :refer [resolv]]))

;; Port de pypilot/pilots/pilot.py (AutopilotPilot)
;;
;; Les gains sont enregistrés dans values sous "ap.pilot.<name>.<gain>"
;; et mis à jour via set-value!.
;;
;; gains est une map : {gain-name {:apgain-name str :sensor-name str}}

(defn make-pilot
  "Crée l'état d'un pilote. name = 'basic', 'rate', etc."
  [name]
  {:name  name
   :gains {}})

(defn- gain-prefix [pilot-name gain-name]
  (str "ap.pilot." pilot-name "." gain-name))

(defn- gain-sensor-name [pilot-name gain-name]
  (str "ap.pilot." pilot-name "." gain-name "gain"))

(defn add-gain!
  "Enregistre un gain PID dans le registre values.
   Retourne le pilot-state mis à jour avec le nouveau gain."
  [pilot-state gain-name default-val min-val max-val]
  (let [pname     (:name pilot-state)
        ap-name   (gain-prefix pname gain-name)
        sens-name (gain-sensor-name pname gain-name)]
    (v/range-property! ap-name default-val min-val max-val :persistent? true)
    (v/sensor-value!   sens-name 0)
    (update pilot-state :gains assoc gain-name
            {:apgain-name ap-name
             :sensor-name sens-name})))

(defn add-pos-gain!
  "Gain de position (min = 0)."
  [pilot-state gain-name default-val max-val]
  (add-gain! pilot-state gain-name default-val 0 max-val))

(defn compute
  "Calcule la commande servo à partir des valeurs de gain courantes.
   gain-inputs : {gain-name input-value}
   Retourne la commande (nombre), met à jour les SensorValues de gain."
  [pilot-state gain-inputs]
  (reduce
   (fn [cmd [gain-name {:keys [apgain-name sensor-name]}]]
     (let [input    (get gain-inputs gain-name 0)
           apgain   (or (v/get-value apgain-name) 0)
           contrib  (* input apgain)]
       (v/update-value! sensor-name contrib)
       (+ cmd contrib)))
   0
   (:gains pilot-state)))

(defn compute-heading!
  "Met à jour ap.heading selon le mode actif."
  [ap-state]
  (let [mode    (v/get-value "ap.mode")
        compass (v/get-value "imu.heading_lowpass")]
    (when compass
      (v/set-value! "ap.heading"
                    (case mode
                      "true wind" (resolv (- (v/get-value "ap.true_wind_compass_offset") compass))
                      "wind"      (resolv (- (v/get-value "ap.wind_compass_offset") compass))
                      ("gps" "nav") (resolv (+ compass (v/get-value "ap.gps_compass_offset")) 180)
                      ;; "compass" ou par défaut
                      compass)))))

(def mode-fallbacks
  {"nav" "gps" "gps" "compass" "wind" "compass" "true wind" "wind"})

(defn best-mode
  "Retourne le meilleur mode disponible, avec repli si le mode demandé n'est pas dispo."
  [mode available-modes]
  (loop [m mode]
    (if (contains? (set available-modes) m)
      m
      (if-let [fallback (mode-fallbacks m)]
        (recur fallback)
        m))))

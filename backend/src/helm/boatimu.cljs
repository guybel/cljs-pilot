(ns helm.boatimu
  (:require [helm.values :as v]
            [utils.quaternion :as q]
            [utils.vector :as vec]))

;; Port de pypilot/boatimu.py (BoatIMU + IMU)
;;
;; Ce namespace :
;;   1. Enregistre toutes les valeurs imu.* dans le registre
;;   2. Démarre un worker Node.js (imu_worker.js) qui lit le MPU-9250 via I2C
;;      ou simule si I2C n'est pas disponible
;;   3. Reçoit les données fusionnées (quaternion Mahony)
;;   4. Calcule heading/pitch/roll/headingrate/headingraterate + filtres lowpass
;;   5. Appelle le callback on-data (autopilot/on-imu-data!)

;; ---------------------------------------------------------------------------
;; Constantes
;; ---------------------------------------------------------------------------
(def RAD2DEG (/ 180.0 Math/PI))

;; ---------------------------------------------------------------------------
;; Enregistrement des valeurs imu.*
;; ---------------------------------------------------------------------------

(defn register-values! []
  ;; Configuration
  (v/enum-property!    "imu.rate"                          20   [10 20]   :persistent? true)
  (v/range-property!   "imu.heading_offset"                0    -180 180  :persistent? true)
  (v/resettable-value! "imu.alignmentQ"                    [1 0 0 0]      :persistent? true)
  (v/range-property!   "imu.heading_lowpass_constant"      0.2  0.05 0.3)
  (v/range-property!   "imu.headingrate_lowpass_constant"  0.2  0.05 0.3)
  (v/range-property!   "imu.headingraterate_lowpass_constant" 0.1 0.05 0.3)

  ;; Valeurs capteurs
  (v/sensor-value! "imu.accel"                false)
  (v/sensor-value! "imu.gyro"                 false)
  (v/sensor-value! "imu.compass"              false)
  (v/sensor-value! "imu.pitch"                false)
  (v/sensor-value! "imu.roll"                 false)
  (v/sensor-value! "imu.pitchrate"            false)
  (v/sensor-value! "imu.rollrate"             false)
  (v/sensor-value! "imu.headingrate"          false)
  (v/sensor-value! "imu.headingraterate"      false)
  (v/sensor-value! "imu.heel"                 false)
  (v/sensor-value! "imu.headingrate_lowpass"       false)
  (v/sensor-value! "imu.headingraterate_lowpass"   false)
  (v/sensor-value! "imu.heading"              false :directional true)
  (v/sensor-value! "imu.heading_lowpass"      false :directional true)
  (v/sensor-value! "imu.fusionQPose"          false :fmt "%.10f")
  (v/sensor-value! "imu.frequency"            false)
  (v/string-value! "imu.error"                ""))

;; ---------------------------------------------------------------------------
;; État interne
;; ---------------------------------------------------------------------------
(defonce internal-state
  (atom {:last-headingrate    0
         :last-timestamp      0
         :heel                0
         :freq-count          0
         :freq-t0             0
         :worker              nil}))

;; ---------------------------------------------------------------------------
;; heading_filter — gère le wraparound 0/360
;; ---------------------------------------------------------------------------
(defn- heading-filter [lp a b]
  (cond
    (or (false? a) (nil? a)) b
    (or (false? b) (nil? b)) a
    :else
    ;; Fidèle à pypilot/boatimu.py heading_filter :
    ;;   si a-b > 180 → a -= 360
    ;;   si b-a > 180 → b -= 360  (ajuste b, pas a)
    ;; puis clip [0, 360)
    (let [[a b] (cond (> (- a b) 180) [(- a 360) b]
                      (> (- b a) 180) [a (- b 360)]
                      :else           [a b])
          r     (+ (* lp a) (* (- 1 lp) b))]
      (cond (< r 0)    (+ r 360)
            (>= r 360) (- r 360)
            :else r))))

;; ---------------------------------------------------------------------------
;; Traitement d'une frame IMU
;; ---------------------------------------------------------------------------
(defn- process-frame!
  "Calcule toutes les valeurs dérivées depuis le quaternion fusionné.
   Appelle on-data-fn avec {:heading :headingrate :headingraterate :pitch :roll}."
  [msg on-data-fn]
  (let [{:keys [fusionQPose accel gyro compass timestamp]} msg

        ;; Appliquer alignmentQ
        alignment-q  (or (v/get-value "imu.alignmentQ") [1 0 0 0])
        aligned      (q/normalize (q/multiply fusionQPose alignment-q))

        ;; Euler angles (radians → degrés)
        [roll-r pitch-r heading-r] (q/toeuler aligned)
        roll    (* roll-r    RAD2DEG)
        pitch   (* pitch-r   RAD2DEG)
        heading (let [h (* heading-r RAD2DEG)]
                  (if (< h 0) (+ h 360) h))

        ;; headingrate depuis gyro en repère monde
        ;; gyro est en rad/s dans le repère capteur
        gyro-world (q/rotvecquat gyro fusionQPose)
        [ur vr headingrate-r] gyro-world
        headingrate     (* headingrate-r RAD2DEG)
        rh              (* heading-r 1)   ; déjà en radians
        pitchrate       (- (* vr (Math/cos heading-r))
                           (* ur (Math/sin heading-r)))
        rollrate        (+ (* ur (Math/cos heading-r))
                           (* vr (Math/sin heading-r)))

        ;; headingraterate
        {:keys [last-headingrate last-timestamp heel]} @internal-state
        dt           (let [d (/ (- timestamp last-timestamp) 1000.0)]
                       (if (and (> d 0.01) (< d 0.2)) d 0))
        headingraterate (if (pos? dt)
                          (/ (- headingrate last-headingrate) dt)
                          0)

        ;; heel (roll IIR très lent)
        new-heel     (+ (* 0.03 roll) (* 0.97 heel))

        ;; Lowpass heading
        lp-h  (v/get-value "imu.heading_lowpass_constant")
        lp-r  (v/get-value "imu.headingrate_lowpass_constant")
        lp-rr (v/get-value "imu.headingraterate_lowpass_constant")

        heading-lp      (heading-filter lp-h heading
                                        (v/get-value "imu.heading_lowpass"))
        headingrate-lp  (+ (* lp-r headingrate)
                           (* (- 1 lp-r)
                              (or (v/get-value "imu.headingrate_lowpass") 0)))
        headingraterate-lp (+ (* lp-rr headingraterate)
                               (* (- 1 lp-rr)
                                  (or (v/get-value "imu.headingraterate_lowpass") 0)))]

    ;; Mise à jour de l'état interne
    (swap! internal-state assoc
           :last-headingrate headingrate
           :last-timestamp   timestamp
           :heel             new-heel)

    ;; Fréquence (mise à jour tous les 4 cycles)
    (let [{:keys [freq-count freq-t0]} @internal-state
          new-count (inc freq-count)]
      (if (= new-count 4)
        (let [freq (/ 4.0 (/ (- timestamp freq-t0) 1000.0))]
          (v/update-value! "imu.frequency" freq)
          (swap! internal-state assoc :freq-count 0 :freq-t0 timestamp))
        (swap! internal-state assoc :freq-count new-count)))

    ;; Mettre à jour toutes les valeurs dans le registre
    (v/update-value! "imu.fusionQPose"              (vec fusionQPose))
    (v/update-value! "imu.accel"                    (vec accel))
    (v/update-value! "imu.gyro"                     (mapv #(* % RAD2DEG) gyro))
    (v/update-value! "imu.compass"                  (vec compass))
    (v/update-value! "imu.heading"                  heading)
    (v/update-value! "imu.roll"                     roll)
    (v/update-value! "imu.pitch"                    pitch)
    (v/update-value! "imu.headingrate"              headingrate)
    (v/update-value! "imu.headingraterate"          headingraterate)
    (v/update-value! "imu.pitchrate"                (* pitchrate RAD2DEG))
    (v/update-value! "imu.rollrate"                 (* rollrate  RAD2DEG))
    (v/update-value! "imu.heel"                     new-heel)
    (v/update-value! "imu.heading_lowpass"          heading-lp)
    (v/update-value! "imu.headingrate_lowpass"      headingrate-lp)
    (v/update-value! "imu.headingraterate_lowpass"  headingraterate-lp)

    ;; Appel du callback autopilot
    (on-data-fn {:heading           heading-lp
                 :headingrate        headingrate-lp
                 :headingraterate    headingraterate-lp
                 :pitch              pitch
                 :roll               roll})))

;; ---------------------------------------------------------------------------
;; Démarrage du worker
;; ---------------------------------------------------------------------------

(defn start!
  "Enregistre les valeurs imu.*, démarre le worker IMU et l'autopilot.
   on-data-fn : fonction appelée à chaque frame IMU avec les données filtrées."
  [on-data-fn]
  (register-values!)
  (let [wt     (js/require "worker_threads")
        path   (js/require "path")
        worker-path (.resolve path (.cwd js/process) "imu_worker.js")
        worker (new (.-Worker wt) worker-path)]
    (.on worker "message"
         (fn [raw-msg]
           (let [msg (js->clj raw-msg :keywordize-keys true)]
             (case (:type msg)
               "data"   (process-frame! msg on-data-fn)
               "status" (js/console.log (str "[IMU] " (:msg msg)))
               nil))))
    (.on worker "error"
         (fn [e] (js/console.error "[IMU] worker error:" e)))
    (swap! internal-state assoc
           :worker       worker
           :freq-t0      (js/Date.now))
    worker))

(defn stop! []
  (when-let [w (:worker @internal-state)]
    (.terminate w)
    (swap! internal-state assoc :worker nil)))

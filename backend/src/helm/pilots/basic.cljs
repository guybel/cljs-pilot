(ns helm.pilots.basic
  (:require [helm.values :as v]
            [helm.pilots.pilot :as pilot]))

;; Port de pypilot/pilots/basic.py (BasicPilot)
;;
;; Gains PID complets : P, I, D, DD, PR, FF

(def ^:private gain-defaults
  {:P 0.003 :I 0.0 :D 0.09 :DD 0.075 :PR 0.005 :FF 0.6})

(defn init!
  "Enregistre les gains du pilote basic et retourne son état.
   gains : map optionnelle depuis config/get-cfg :gains — ex {:P 0.003 :D 0.09 ...}"
  ([] (init! nil))
  ([gains]
   (let [g (merge gain-defaults gains)]
     (-> (pilot/make-pilot "basic")
         (pilot/add-pos-gain! "P"  (:P  g) 0.03)
         (pilot/add-pos-gain! "I"  (:I  g) 0.05)
         (pilot/add-pos-gain! "D"  (:D  g) 0.24)
         (pilot/add-pos-gain! "DD" (:DD g) 0.24)
         (pilot/add-pos-gain! "PR" (:PR g) 0.02)
         (pilot/add-pos-gain! "FF" (:FF g) 2.4)))))

(defn process!
  "Calcule et envoie la commande servo.
   ap-state : {:heading-error :heading-error-int :headingrate :headingraterate :heading-command-rate}
   Retourne la commande calculée."
  [pilot-state ap-state]
  (let [{:keys [heading-error heading-error-int
                headingrate headingraterate
                heading-command-rate]} ap-state
        P  heading-error
        PR (* (Math/sign P) (Math/sqrt (Math/abs P)))
        gain-inputs {:P  P
                     :I  heading-error-int
                     :D  headingrate
                     :DD headingraterate
                     :PR PR
                     :FF heading-command-rate}
        cmd (pilot/compute pilot-state gain-inputs)]
    (v/update-value! "ap.pilot.basic.command" cmd)
    cmd))

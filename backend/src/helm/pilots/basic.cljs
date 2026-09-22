(ns helm.pilots.basic
  (:require [helm.values :as v]
            [helm.pilots.pilot :as pilot]))

;; Port de pypilot/pilots/basic.py (BasicPilot)
;;
;; Gains PID complets : P, I, D, DD, PR, FF

(def ^:private gain-defaults
  {:P 0.03 :I 0.0 :D 0.09 :DD 0.075 :PR 0.005 :FF 0.6})

;; Etat de persistance pour l'hysteresis anti-bruit (voir process! plus bas).
;; Suit depuis combien de temps une erreur soutenue dans une direction donnee
;; est observee, avant d'autoriser le plancher de commande a se declencher.
(defonce ^:private sustain-state (atom {:direction 0 :since-ms 0}))

(def ^:private sustain-duration-ms
  "Duree pendant laquelle une commande doit rester au-dela du deadband,
   dans la MEME direction, avant de declencher le plancher. Filtre le bruit
   IMU transitoire (qui change de signe rapidement) sans retarder les
   vraies corrections de cap (qui persistent par nature)."
  200)

(defn init!
  "Enregistre les gains du pilote basic et retourne son état.
   gains : map optionnelle depuis config/get-cfg :gains — ex {:P 0.003 :D 0.09 ...}"
  ([] (init! nil))
  ([gains]
   (let [g (merge gain-defaults gains)]
     (-> (pilot/make-pilot "basic")
         (pilot/add-pos-gain! "P"  (:P  g) 0.3)
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
        ;; calcul identique a pypilot/pilots/basic.py
        P  heading-error
        PR (* (Math/sign heading-error)
              (Math/sqrt (Math/abs heading-error)))
        gain-inputs {"P"  P
                     "I"  heading-error-int
                     "D"  headingrate
                     "DD" headingraterate
                     "PR" PR
                     "FF" heading-command-rate}
        ;; Compute = somme ponderee input * gain (identique à self.Compute)
        cmd (pilot/compute pilot-state gain-inputs)]
    ;; On met a jour la valeur de commande pour le monitoring
        (v/update-value! "ap.pilot.basic.command" cmd)
        cmd))


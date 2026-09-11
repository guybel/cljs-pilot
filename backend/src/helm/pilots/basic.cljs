(ns helm.pilots.basic
  (:require [helm.values :as v]
            [helm.pilots.pilot :as pilot]))

;; Port de pypilot/pilots/basic.py (BasicPilot)
;;
;; Gains PID complets : P, I, D, DD, PR, FF

(def ^:private gain-defaults
  {:P 0.3 :I 0.0 :D 0.09 :DD 0.075 :PR 0.005 :FF 0.6})

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
        P  heading-error
        PR (* (Math/sign P) (Math/sqrt (Math/abs P)))
        gain-inputs {"P"  P
                     "I"  heading-error-int
                     "D"  headingrate
                     "DD" headingraterate
                     "PR" PR
                     "FF" heading-command-rate}
        cmd (pilot/compute pilot-state gain-inputs)
        ;; Le moteur/servo accepte une commande normalisée complete dans [-1, 1].
        ;;
        ;; Trois protections distinctes gerees ici :
        ;; 1) command-deadband : ignore les micro-oscillations autour du neutre.
        ;; 2) sustain-duration-ms : une commande qui depasse le deadband doit
        ;;    persister dans la MEME direction pendant sustain-duration-ms avant
        ;;    de declencher quoi que ce soit. Sans ca, le moindre pic de bruit
        ;;    IMU (meme bateau parfaitement immobile) est amplifie par le
        ;;    plancher ci-dessous et produit des corrections visibles pour rien.
        ;; 3) command-floor : le verin a un seuil de frottement statique (stiction) -
        ;;    en dessous d'une certaine puissance, le courant passe mais le verin
        ;;    ne bouge pas ou presque pas. Une fois qu'une correction est jugee
        ;;    reelle (point 2), elle demarre directement a command-floor plutot
        ;;    que de monter trop doucement pour produire un mouvement reel.
        command-deadband 0.02
        command-floor    0.15   ; A CALIBRER selon le seuil de mouvement reel du verin
        now (js/Date.now)
        mag (js/Math.abs cmd)
        raw-direction (cond (> cmd command-deadband) 1
                            (< cmd (- command-deadband)) -1
                            :else 0)
        _ (when (not= raw-direction (:direction @sustain-state))
            ;; La direction a change (ou on repasse a zero) : on redemarre le chrono.
            (reset! sustain-state {:direction raw-direction :since-ms now}))
        sustained? (and (not= raw-direction 0)
                        (>= (- now (:since-ms @sustain-state)) sustain-duration-ms))
        safe-cmd (if-not sustained?
                   0.0
                   (let [sign (if (neg? cmd) -1.0 1.0)
                         scaled (+ command-floor
                                   (* (- 1.0 command-floor)
                                      (/ (- mag command-deadband)
                                         (- 1.0 command-deadband))))]
                     (* sign (min (max scaled command-floor) 1.0))))]
    (v/update-value! "ap.pilot.basic.command" safe-cmd)
    safe-cmd))
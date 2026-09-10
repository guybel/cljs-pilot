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
        gain-inputs {"P"  P
                     "I"  heading-error-int
                     "D"  headingrate
                     "DD" headingraterate
                     "PR" PR
                     "FF" heading-command-rate}
        cmd (pilot/compute pilot-state gain-inputs)
        ;; Le moteur/servo accepte une commande normalisée complete dans [-1, 1].
        ;;
        ;; Deux problemes distincts geres ici :
        ;; 1) command-deadband : ignore les micro-oscillations autour du neutre
        ;;    (sinon le pilot envoie des corrections en boucle pour du bruit).
        ;; 2) command-floor : le verin a un seuil de frottement statique (stiction) -
        ;;    en dessous d'une certaine puissance, le courant passe mais le verin
        ;;    ne bouge pas ou presque pas. Sans plancher, une petite erreur de cap
        ;;    produit une commande trop faible pour produire un mouvement reel.
        ;;
        ;; Le remappage garantit qu'au-dela du deadband, la commande demarre
        ;; directement a command-floor (mouvement immediatement significatif)
        ;; puis continue a augmenter proportionnellement jusqu'a 1.0 pour les
        ;; grosses erreurs - on garde la proportionnalite, juste decalee.
        command-deadband 0.02
        command-floor    0.35   ; A CALIBRER: teste au REPL/slider le seuil reel
                                  ; de mouvement visible de ton verin, mets ce chiffre
                                  ; legerement au-dessus.
        safe-cmd (let [c      (max -1.0 (min 1.0 cmd))
                       mag    (js/Math.abs c)
                       sign   (if (neg? c) -1.0 1.0)]
                   (cond
                     (<= mag command-deadband)
                     0.0

                     :else
                     (let [scaled (+ command-floor
                                     (* (- 1.0 command-floor)
                                        (/ (- mag command-deadband)
                                           (- 1.0 command-deadband))))]
                       (* sign (min scaled 1.0)))))]
    (v/update-value! "ap.pilot.basic.command" safe-cmd)
    safe-cmd))
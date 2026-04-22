(ns helm.config
  (:require [cljs.reader :as edn]))

;; Lecteur de config.edn.
;;
;; Usage :
;;   (config/load!)              ; à appeler en premier dans main
;;   (config/get-cfg :signalk)   ; → {:enabled true :host "..." :port 3000}
;;   (config/get-cfg :gains :P)  ; → 0.003
;;   (config/all)                ; → map complète

;; Valeurs par défaut — utilisées si config.edn est absent ou incomplet.
(def ^:private defaults
  {:server  {:port 23322 :ws-port 23323}
   :imu     {:source :both :rate 20}
   :signalk {:enabled false :host "localhost" :port 3000}
   :servo   {:port "/dev/ttyUSB0" :baud 38400}
   :gains   {:P 0.003 :I 0.0 :D 0.09 :DD 0.075 :PR 0.005 :FF 0.6}})

(defonce ^:private loaded (atom defaults))

(defn load!
  "Lit config.edn depuis le répertoire courant.
   Merge section par section avec les défauts.
   Retourne la config complète."
  []
  (try
    (let [fs  (js/require "fs")
          p   (js/require "path")
          fp  (.resolve p (.cwd js/process) "config.edn")
          raw (.readFileSync fs fp "utf8")]
      (reset! loaded (merge-with merge defaults (edn/read-string raw)))
      (js/console.log "[config] Chargé:" fp)
      @loaded)
    (catch :default e
      (js/console.warn "[config] config.edn introuvable —" (.-message e) "— défauts utilisés")
      @loaded)))

(defn all
  "Retourne la config complète sous forme de map."
  []
  @loaded)

(defn get-cfg
  "Retourne la valeur au chemin de clés.
   (get-cfg :signalk)        → {:enabled true :host ... :port ...}
   (get-cfg :signalk :host)  → \"openplotter.local\"
   (get-cfg :gains :P)       → 0.003"
  ([k]      (get @loaded k))
  ([k & ks] (get-in @loaded (into [k] ks))))

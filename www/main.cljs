(ns main
  (:require
   [reagent.core :as r]
   [reagent.dom  :as rdom]
   [clojure.string :as str]))

;; ===========================================================================
;; [A] Wire / WebSocket
;; ===========================================================================

(def ws-url
  (let [host (or (let [h (.-hostname js/location)]
                   (when (and h (pos? (.-length h))) h))
                 "localhost")
        scheme (if (or (= (.-protocol js/location) "https:")
                       (= (.-protocol js/location) "wss:"))
                 "wss:"
                 "ws:")]
    (str scheme "//" host ":23323")))

(defn- parse-wire-value
  "Décode la partie droite d'une ligne wire (name=VALUE).
   Retourne number/bool/string/vector selon format pypilot."
  [s]
  (try (js/JSON.parse s)
       (catch :default _ s)))

;; ===========================================================================
;; [B] État global
;; ===========================================================================
;;
;; state :
;;   :values     — {"ap.heading" 42.3, "ap.enabled" false, …}
;;   :meta       — {"ap.pilot.basic.P" {"type" "RangeProperty" "min" 0 "max" 0.03 "writable" true} …}
;;   :ws         — WebSocket instance
;;   :connected  — bool
;;   :tab        — :control | :gain | :calibration | :config | :stats
;;   :theme      — "dark" | "light"
;;   :host-draft — édition en cours du champ signalk.host (non commité)

(def tab-ids #{:control :gain :calibration :config :stats})

(defn- tab-from-hash []
  (let [h (some-> (.-hash js/location) (.substring 1))
        kw (when (seq h) (keyword h))]
    (if (tab-ids kw) kw :control)))

(defonce state
  (r/atom {:values    {}
           :meta      {}
           :ws        nil
           :connected false
           :tab       (tab-from-hash)
           :theme     (or (.getItem js/localStorage "helm-theme") "dark")
           :host-draft nil}))

(defn v
  "Lit la valeur courante d'une clé du registre."
  [k]
  (get-in @state [:values k]))

(defn meta-of [k]
  (get-in @state [:meta k]))

(defn writable? [k]
  (boolean (get (meta-of k) "writable")))

;; ===========================================================================
;; [C] Envoi & watches
;; ===========================================================================

(defn- ws-send! [s]
  (let [ws (:ws @state)]
    (when (and ws (= (.-readyState ws) 1))
      (.send ws s))))

(defn set!*
  "Envoie une commande set au backend. v peut être bool/number/string/vector.
   Sérialise en JSON pour respecter le format attendu côté serveur."
  [k val]
  (let [wire (cond
               (boolean? val) (if val "true" "false")
               (number? val)  (str val)
               (string? val)  (js/JSON.stringify val)
               (nil? val)     "false"
               :else          (js/JSON.stringify (clj->js val)))]
    (ws-send! (str k "=" wire "\n"))))

(defn watch!
  "Souscrit à un ensemble de clés. m = {\"key\" period | false}"
  [m]
  (ws-send! (str "watch=" (js/JSON.stringify (clj->js m)) "\n")))

(def base-watches
  "Watches actifs en permanence (tous onglets)."
  {"ap.enabled"            0
   "ap.mode"               0
   "ap.heading_command"    0
   "ap.heading"            0.5
   "ap.heading_error"      0.5
   "ap.heading_command_rate" 0.5
   "signalk.heading"       0.5
   "signalk.pitch"         0.5
   "signalk.roll"          0.5
   "signalk.heel"          0.5
   "signalk.connected"     0
   "servo.voltage"         1
   "servo.current"         1
   "servo.flags"           0
   "servo.connected"       0
   "imu.frequency"         1
   "imu.heading"           0.5
   "imu.pitch"             0.5
   "imu.roll"              0.5
   "imu.heel"              0.5
   "imu.heading_offset"    0
   "imu.rate"              0
   "imu.heading_lowpass_constant"      0
   "imu.headingrate_lowpass_constant"  0
   "imu.headingraterate_lowpass_constant" 0
   "imu.calibration"       1
   "signalk.host"          0
   "signalk.port"          0
   "signalk.enabled"       0
   "gps.lat"               2
   "gps.lon"               2
   "gps.speed"             1
   "gps.track"             1
   "ap.pilot"              0})

(defn- subscribe-all-pilot-gains! []
  ;; Souscrit dynamiquement à toutes les clés ap.pilot.*.{P,I,D,DD,PR,FF}
  (let [pattern #"^ap\.pilot\.[^.]+\.[A-Z]+$"
        ks (filter #(re-matches pattern %) (keys (:meta @state)))]
    (when (seq ks)
      (watch! (zipmap ks (repeat 0))))))

;; ===========================================================================
;; [D] Parsing entrant
;; ===========================================================================

(defn- parse-line! [line]
  (when-not (str/blank? line)
    (let [idx (.indexOf line "=")]
      (when (pos? idx)
        (let [k (.substring line 0 idx)
              rest (.substring line (inc idx))]
          (cond
            (= k "values")
            (try
              (let [parsed (js->clj (js/JSON.parse rest))]
                (swap! state update :meta merge parsed)
                ;; Une fois le catalogue reçu on peut souscrire aux gains dynamiques
                (subscribe-all-pilot-gains!))
              (catch :default e
                (js/console.warn "[UI] values parse error:" e)))

            (= k "error")
            (js/console.warn "[UI] server error:" rest)

            :else
            (swap! state assoc-in [:values k] (parse-wire-value rest))))))))

(defn- connect! []
  (let [ws (js/WebSocket. ws-url)
        buf (atom "")]
    (set! (.-onopen ws)
          (fn [_]
            (js/console.log "[UI] WS connected")
            (swap! state assoc :connected true :ws ws)
            (watch! base-watches)))
    (set! (.-onmessage ws)
          (fn [evt]
            (swap! buf str (.-data evt))
            (loop []
              (let [s @buf
                    i (.indexOf s "\n")]
                (when (>= i 0)
                  (let [line (.substring s 0 i)]
                    (reset! buf (.substring s (inc i)))
                    (parse-line! line)
                    (recur)))))))
    (set! (.-onclose ws)
          (fn [_]
            (swap! state assoc :connected false :ws nil)
            (js/setTimeout connect! 3000)))
    (set! (.-onerror ws)
          (fn [_]
            (try (.close ws) (catch :default _ nil))))))

;; ===========================================================================
;; [E] Helpers formatage
;; ===========================================================================

(defn- num?
  "Vrai ssi v est un number fini (filtre nil/false/NaN/strings)."
  [v]
  (and (number? v) (not (js/isNaN v))))

(defn- fmt-deg
  ([val] (fmt-deg val false))
  ([val signed?]
   (if-not (num? val) "---"
       (let [n (js/Math.round val)]
         (if signed?
           (str (if (>= n 0) "+" "") n "°")
           (str (-> (str "00" (js/Math.abs n)) (.slice -3)) "°"))))))

(defn- fmt-1 [val]
  (if (num? val) (.toFixed val 1) "--.-"))

(defn- fmt-n [val n]
  (if (num? val) (.toFixed val n) "---"))

(defn- resolv [a]
  (loop [a a]
    (cond (> a  180) (recur (- a 360))
          (< a -180) (recur (+ a 360))
          :else a)))

(defn- num-str? [s]
  (and (string? s) (not (js/isNaN (js/parseFloat s)))))

;; ===========================================================================
;; [F] Composants réutilisables
;; ===========================================================================

(defn range-slider
  "Slider générique lié à une RangeProperty.
   opts : {:key 'ap.pilot.basic.P' :label 'P' :step 0.001 :decimals 4}
   Si la clé n'est pas présente dans les métadonnées, affiche un placeholder désactivé."
  [{:keys [key label step decimals]}]
  (let [info   (meta-of key)]
    (if-not info
      [:div.slider-row.missing
       [:div.slider-head
        [:span.slider-label label]
        [:span.slider-val "n/a"]]
       [:div.slider-empty "valeur non exposée par le backend"]]
      (let [val    (v key)
            mn     (get info "min" 0)
            mx     (get info "max" 1)
            rng    (- mx mn)
            step   (or step (cond (<= rng 0.1) 0.0001
                                  (<= rng 1)   0.001
                                  (<= rng 10)  0.01
                                  (<= rng 100) 0.1
                                  :else        1))
            decimals (or decimals (cond (<= rng 0.1) 4
                                        (<= rng 1)   3
                                        (<= rng 10)  2
                                        (<= rng 100) 1
                                        :else        0))
            disabled? (or (not (writable? key))
                          (not (:connected @state))
                          (nil? val)
                          (false? val))]
        [:div.slider-row
         [:div.slider-head
          [:span.slider-label label]
          [:span.slider-val (if (number? val) (.toFixed val decimals) "--")]]
         [:input.slider
          {:type      "range"
           :min       mn
           :max       mx
           :step      step
           :value     (if (number? val) val mn)
           :disabled  disabled?
           :on-change (fn [e]
                        (let [n (js/parseFloat (.. e -target -value))]
                          (when-not (js/isNaN n)
                            (swap! state assoc-in [:values key] n)
                            (set!* key n))))}]
         [:div.slider-minmax
          [:span (fmt-n mn decimals)]
          [:span (fmt-n mx decimals)]]]))))

(defn bool-toggle
  [{:keys [key label]}]
  (if-not (meta-of key)
    [:div.toggle-row.missing
     [:span.toggle-label label]
     [:span.slider-val "n/a"]]
    (let [val       (v key)
          connected (:connected @state)
          disabled? (or (not (writable? key)) (not connected))]
      [:label.toggle-row {:class (when disabled? "disabled")}
       [:span.toggle-label label]
       [:span.toggle-switch {:class (when val "on")}
        [:input {:type      "checkbox"
                 :checked   (boolean val)
                 :disabled  disabled?
                 :on-change #(set!* key (not val))}]
        [:span.toggle-slider]]])))

(defn enum-select
  [{:keys [key label choices-override]}]
  (let [info (meta-of key)]
    (if (and (nil? info) (nil? choices-override))
      [:div.select-row.missing
       (when label [:span.select-label label])
       [:span.slider-val "n/a"]]
      (let [choices   (or choices-override (get info "choices") [])
            val       (v key)
            connected (:connected @state)
            disabled? (or (not (writable? key)) (not connected))]
        [:label.select-row {:class (when disabled? "disabled")}
         (when label [:span.select-label label])
         [:select.select
          {:value     (if (nil? val) "" (str val))
           :disabled  disabled?
           :on-change (fn [e]
                        (let [new-v (.. e -target -value)
                              coerced (cond
                                        (num-str? new-v) (js/parseFloat new-v)
                                        :else            new-v)]
                          (set!* key coerced)))}
          (for [c choices]
            ^{:key (str c)} [:option {:value (str c)} (str c)])]]))))

(defn text-input
  "Champ texte commité à la perte de focus ou Entrée.
   Pas de mise à jour live pour éviter le flux de set! à chaque frappe."
  [{:keys [key label]}]
  (if-not (meta-of key)
    [:div.text-row.missing
     [:span.text-label label]
     [:span.slider-val "n/a"]]
    (let [val         (v key)
          draft-key   (keyword (str "draft-" key))
          draft       (get @state draft-key)
          shown       (if (some? draft)
                        draft
                        (if (string? val) val ""))
          connected   (:connected @state)
          disabled?   (or (not (writable? key)) (not connected))
          commit!     (fn []
                        (when (and (some? draft) (not= draft val))
                          (set!* key draft))
                        (swap! state dissoc draft-key))]
      [:label.text-row {:class (when disabled? "disabled")}
       [:span.text-label label]
       [:input.text-input
        {:type      "text"
         :value     shown
         :disabled  disabled?
         :on-change #(swap! state assoc draft-key (.. % -target -value))
         :on-blur   commit!
         :on-key-down (fn [e]
                        (when (= (.-key e) "Enter") (commit!)))}]])))

(defn number-input
  "Champ numérique commité à la perte de focus ou Entrée.
   Respecte min/max/step issus des métadonnées si présents."
  [{:keys [key label step]}]
  (let [info (meta-of key)]
    (if-not info
      [:div.text-row.missing
       [:span.text-label label]
       [:span.slider-val "n/a"]]
      (let [val         (v key)
            draft-key   (keyword (str "draft-" key))
            draft       (get @state draft-key)
            shown       (cond
                          (some? draft) draft
                          (number? val) (str val)
                          :else "")
            mn          (get info "min")
            mx          (get info "max")
            connected   (:connected @state)
            disabled?   (or (not (writable? key)) (not connected))
            commit!     (fn []
                          (when (some? draft)
                            (let [n (js/parseFloat draft)]
                              (when (and (not (js/isNaN n)) (not= n val))
                                (let [clamped (cond-> n
                                                (number? mn) (max mn)
                                                (number? mx) (min mx))]
                                  (set!* key clamped)))))
                          (swap! state dissoc draft-key))]
        [:label.text-row {:class (when disabled? "disabled")}
         [:span.text-label label]
         [:input.text-input
          (cond-> {:type        "number"
                   :step        (or step 1)
                   :value       shown
                   :disabled    disabled?
                   :on-change   #(swap! state assoc draft-key (.. % -target -value))
                   :on-blur     commit!
                   :on-key-down (fn [e]
                                  (when (= (.-key e) "Enter") (commit!)))}
            (number? mn) (assoc :min mn)
            (number? mx) (assoc :max mx))]]))))

(defn- first-present
  "Renvoie la première valeur du registre utilisable (nombre fini)."
  [ks]
  (some (fn [k] (let [vv (v k)] (when (num? vv) vv))) ks))

(defn value-display
  [{:keys [key fallback label unit fmt badge-fn]}]
  (let [val (if fallback
              (first-present (cons key (if (vector? fallback) fallback [fallback])))
              (v key))
        formatted
        (case fmt
          :deg       (fmt-deg val)
          :deg-signed (fmt-deg val true)
          :d1        (fmt-1 val)
          :d2        (fmt-n val 2)
          :d4        (fmt-n val 4)
          :bool      (cond (true? val) "oui"
                           (false? val) "non"
                           :else "--")
          :hex       (if (number? val) (str "0x" (.toString (int val) 16)) "--")
          (cond (nil? val) "--"
                (false? val) "--"
                (number? val) (.toString val)
                :else (str val)))
        cls (when badge-fn (badge-fn val))]
    [:div.value-cell
     [:div.value-label label]
     [:div.value-num {:class cls}
      formatted
      (when unit [:span.value-unit (str " " unit)])]]))

;; ===========================================================================
;; [F.bis] Rose des vents (préservée)
;; ===========================================================================

(def ^:private cardinal ["N" "NE" "E" "SE" "S" "SO" "O" "NO"])

(defn compass-rose [heading heading-cmd]
  (let [r 120
        cx 130
        cy 130
        rot   (if (number? heading) heading 0)
        cmd-a (if (number? heading-cmd) heading-cmd 0)]
    [:svg {:viewBox "0 0 260 260" :xmlns "http://www.w3.org/2000/svg"}
     [:circle {:cx cx :cy cy :r r :fill "#0d1921" :stroke "#2a3f58" :stroke-width 1.5}]
     [:circle {:cx cx :cy cy :r (- r 18) :fill "none" :stroke "#1a2f42" :stroke-width 1}]

     [:g {:transform (str "rotate(" (- rot) " " cx " " cy ")")}
      (for [deg (range 0 360 5)]
        (let [rad (-> deg (* js/Math.PI) (/ 180))
              len (if (zero? (mod deg 10)) 10 6)
              x1  (+ cx (* (- r 2)   (js/Math.sin rad)))
              y1  (- cy (* (- r 2)   (js/Math.cos rad)))
              x2  (+ cx (* (- r len) (js/Math.sin rad)))
              y2  (- cy (* (- r len) (js/Math.cos rad)))]
          ^{:key deg}
          [:line {:x1 x1 :y1 y1 :x2 x2 :y2 y2
                  :stroke "#2a3f58" :stroke-width (if (zero? (mod deg 10)) 1.5 1)}]))

      (for [[i label] (map-indexed vector cardinal)]
        (let [deg (* i 45)
              rad (-> deg (* js/Math.PI) (/ 180))
              rd  (- r 26)
              x   (+ cx (* rd (js/Math.sin rad)))
              y   (+ (- cy (* rd (js/Math.cos rad))) 5)]
          ^{:key label}
          [:text {:x x :y y
                  :text-anchor "middle"
                  :fill (if (= label "N") "#ff4455" "#607080")
                  :font-size (if (= label "N") 14 11)
                  :font-weight "bold"
                  :font-family "Courier New, monospace"}
           label]))]

     (when (number? heading-cmd)
       (let [rel   (resolv (- cmd-a rot))
             rad   (-> rel (* js/Math.PI) (/ 180))
             rd    (- r 8)
             tx    (+ cx (* rd (js/Math.sin rad)))
             ty    (- cy (* rd (js/Math.cos rad)))
             perp  (+ rad (/ js/Math.PI 2))
             hw    5
             px    (* hw (js/Math.cos perp))
             py    (* hw (js/Math.sin perp))]
         [:polygon {:points (str (+ tx px) "," (+ ty py) " "
                                 (- tx px) "," (- ty py) " "
                                 (+ cx (* (- rd 16) (js/Math.sin rad))) ","
                                 (- cy (* (- rd 16) (js/Math.cos rad))))
                    :fill "#00aaff"
                    :opacity 0.85}]))

     [:line {:x1 cx :y1 (- cy r -2) :x2 cx :y2 (- cy r 18)
             :stroke "#ff4455" :stroke-width 3 :stroke-linecap "round"}]
     [:circle {:cx cx :cy cy :r 5 :fill "#1d2a3a" :stroke "#2a3f58" :stroke-width 1.5}]]))

;; ===========================================================================
;; [G] Commandes
;; ===========================================================================

(defn- set-ap! [enabled?] (set!* "ap.enabled" (boolean enabled?)))

(defn- adjust-heading! [delta]
  (let [cur (or (v "ap.heading_command") (v "ap.heading") (v "signalk.heading") 0)
        nxt (mod (+ cur delta) 360)]
    (set!* "ap.heading_command" nxt)))

;; ===========================================================================
;; [H] Status bar
;; ===========================================================================

(defn status-bar []
  (let [{:keys [connected]} @state
        sk-conn    (v "signalk.connected")
        servo-conn (v "servo.connected")
        voltage    (v "servo.voltage")
        freq       (v "imu.frequency")]
    [:div.status-bar
     [:div.dot {:class (cond connected "ok" :else "err")}]
     [:span (if connected "connecté" "hors ligne")]
     (when sk-conn    [:span.badge.sk "Signal K"])
     (when servo-conn [:span.badge.srv "Servo"])
     [:span.spacer]
     (when (number? voltage) [:span.voltage (str (.toFixed voltage 1) "V")])
     (when (number? freq)    [:span.hz (str (.toFixed freq 0) "Hz")])]))

;; ===========================================================================
;; [I] Onglets
;; ===========================================================================

(defn tab-control []
  (let [heading     (or (v "signalk.heading") (v "imu.heading") (v "ap.heading"))
        heading-cmd (v "ap.heading_command")
        heading-err (v "ap.heading_error")
        ap-enabled  (v "ap.enabled")
        connected   (:connected @state)
        err-cls     (cond (or (nil? heading-err) (false? heading-err)) ""
                          (> (js/Math.abs heading-err) 20) "err"
                          (> (js/Math.abs heading-err) 10) "warn"
                          :else "")]
    [:div.tab.tab-control
     [:div.compass-section
      [:div.compass-wrap
       [compass-rose heading heading-cmd]]
      [:div.heading-display (fmt-deg heading)]]

     [:div.sensor-grid
      [value-display {:key "signalk.pitch" :fallback "imu.pitch" :label "PITCH" :fmt :d1 :unit "°"}]
      [value-display {:key "signalk.roll"  :fallback "imu.roll"  :label "ROLL"  :fmt :d1 :unit "°"}]
      [value-display {:key "signalk.heel"  :fallback "imu.heel"  :label "GÎTE"  :fmt :d1 :unit "°"}]
      [:div.value-cell
       [:div.value-label "ERREUR"]
       [:div.value-num {:class err-cls} (fmt-deg heading-err true)]]]

     [:div.target-row
      [:span "COMMANDÉ"]
      [:span.tval (fmt-deg heading-cmd)]
      (when (number? heading-err)
        [:span {:class (str "err-val" (when (> (js/Math.abs heading-err) 20) " warn"))}
         (str "Δ " (fmt-deg heading-err true))])]

     [:div.mode-row
      [enum-select {:key "ap.mode" :label "MODE"}]]

     [:div.controls
      [:button.btn {:on-click #(adjust-heading! -10) :disabled (not connected)} "−10°"]
      [:button.btn {:on-click #(adjust-heading! -1)  :disabled (not connected)} "−1°"]
      [:button {:class (str "btn btn-ap" (when ap-enabled " on"))
                :on-click #(set-ap! (not ap-enabled))
                :disabled (not connected)}
       (if ap-enabled "AP ON" "AP OFF")]
      [:button.btn {:on-click #(adjust-heading! 1)   :disabled (not connected)} "+1°"]
      [:button.btn {:on-click #(adjust-heading! 10)  :disabled (not connected)} "+10°"]]]))

(defn tab-gain []
  (let [pilot       (or (v "ap.pilot") "basic")
        gain-pattern (re-pattern (str "^ap\\.pilot\\." pilot "\\.[A-Z]+$"))
        gain-keys   (->> (:meta @state)
                         keys
                         (filter #(re-matches gain-pattern %))
                         sort)]
    [:div.tab.tab-gain
     [:div.section-head "PILOTE"]
     [enum-select {:key "ap.pilot" :label "Pilote actif"}]

     [:div.section-head "GAINS PID"]
     (if (empty? gain-keys)
       [:div.empty "Aucun gain exposé (backend pas initialisé ?)"]
       [:div.slider-grid
        (for [k gain-keys]
          (let [label (last (str/split k #"\."))]
            ^{:key k} [range-slider {:key k :label label}]))])

     [:div.section-head "COMMANDE COURANTE"]
     [:div.sensor-grid
      [value-display {:key (str "ap.pilot." pilot ".command")
                      :label "CMD" :fmt :d4}]
      [value-display {:key "ap.heading_command_rate"
                      :label "TAUX CMD" :fmt :d2 :unit "°/s"}]]]))

(defn- calib-cell
  "Affiche un niveau de calib BNO055 : pastilles 0-3 + label."
  [label level]
  (let [n      (or level 0)
        klass  (cond (= n 3) "calib-cell calib-ok"
                     (= n 0) "calib-cell calib-bad"
                     :else   "calib-cell calib-mid")]
    [:div {:class klass}
     [:div.calib-dots
      (for [i (range 3)]
        ^{:key i}
        [:span.calib-dot {:class (when (> n i) "on")}])]
     [:div.value-label label]
     [:div.calib-score (str n "/3")]]))

(defn- calib-status []
  (let [c     (v "imu.calibration")
        getf  (fn [k] (when (and c (not (false? c))) (aget c k)))]
    [:div.calib-grid
     [calib-cell "SYS" (getf "sys")]
     [calib-cell "GYR" (getf "gyr")]
     [calib-cell "ACC" (getf "acc")]
     [calib-cell "MAG" (getf "mag")]]))

(defn tab-calibration []
  [:div.tab.tab-calibration
   [:div.section-head "LECTURE LIVE"]
   [:div.sensor-grid
    [value-display {:key "imu.heading" :label "CAP IMU" :fmt :deg}]
    [value-display {:key "imu.pitch"   :label "PITCH"   :fmt :d1 :unit "°"}]
    [value-display {:key "imu.roll"    :label "ROLL"    :fmt :d1 :unit "°"}]
    [value-display {:key "imu.frequency" :label "FREQ IMU" :fmt :d1 :unit "Hz"}]]

   [:div.section-head "CALIBRATION BNO055"]
   [calib-status]
   [:div.calib-hint
    "Pour finaliser : gyr → laisser immobile 5 s · acc → poser sur 6 faces · mag → faire un 8 dans l'air"]

   [:div.section-head "ALIGNEMENT COMPAS"]
   [range-slider {:key "imu.heading_offset" :label "Offset magnétique"}]
   [:div.btn-row
    [:button.btn.btn-full
     {:on-click #(set!* "imu.alignmentQ" false)
      :disabled (not (:connected @state))}
     "Remettre à zéro l'alignement IMU"]]

   [:div.section-head "FRÉQUENCE IMU"]
   [enum-select {:key "imu.rate" :label "Rate (Hz)"}]

   [:details.advanced
    [:summary "Paramètres filtres avancés"]
    [range-slider {:key "imu.heading_lowpass_constant"      :label "Lowpass cap"}]
    [range-slider {:key "imu.headingrate_lowpass_constant"  :label "Lowpass taux"}]
    [range-slider {:key "imu.headingraterate_lowpass_constant" :label "Lowpass accél"}]]

   [:div.placeholder
    "Calibrage capteur de barre & alignement compas GPS : nécessite backend Phase 3."]])

(defn tab-config []
  (let [theme (:theme @state)]
    [:div.tab.tab-config
     [:div.section-head "SIGNAL K"]
     [bool-toggle {:key "signalk.enabled" :label "Activé"}]
     [text-input   {:key "signalk.host"    :label "Hôte"}]
     [number-input {:key "signalk.port"    :label "Port" :step 1}]
     [:div.info-row
      [:span "Connecté :"]
      [:span {:class (if (v "signalk.connected") "badge sk" "badge off")}
       (if (v "signalk.connected") "oui" "non")]]

     [:div.section-head "THÈME"]
     [:div.theme-row
      (for [t ["dark" "light"]]
        ^{:key t}
        [:button.btn.theme-btn
         {:class (when (= theme t) "active")
          :on-click #(do (swap! state assoc :theme t)
                         (.setItem js/localStorage "helm-theme" t)
                         (.setAttribute (.-documentElement js/document) "data-theme" t))}
         (str/upper-case t)])]

     [:div.placeholder
      "NMEA / profils / langue : Phase 3."]]))

(defn tab-stats []
  [:div.tab.tab-stats
   [:div.section-head "SERVO"]
   [:div.sensor-grid
    [value-display {:key "servo.voltage" :label "TENSION" :fmt :d2 :unit "V"}]
    [value-display {:key "servo.current" :label "COURANT" :fmt :d2 :unit "A"}]
    [value-display {:key "servo.flags"   :label "FLAGS"   :fmt :hex}]
    [value-display {:key "servo.connected" :label "LIEN"  :fmt :bool}]]

   [:div.section-head "IMU"]
   [:div.sensor-grid
    [value-display {:key "imu.frequency" :label "FRÉQ" :fmt :d1 :unit "Hz"}]
    [value-display {:key "imu.heading"   :label "CAP"  :fmt :deg}]
    [value-display {:key "imu.pitch"     :label "PITCH" :fmt :d1 :unit "°"}]
    [value-display {:key "imu.roll"      :label "ROLL"  :fmt :d1 :unit "°"}]]

   [:div.section-head "GPS"]
   [:div.sensor-grid
    [value-display {:key "gps.lat"   :label "LAT"   :fmt :d4 :unit "°"}]
    [value-display {:key "gps.lon"   :label "LON"   :fmt :d4 :unit "°"}]
    [value-display {:key "gps.speed" :label "VITESSE" :fmt :d1 :unit "kn"}]
    [value-display {:key "gps.track" :label "ROUTE" :fmt :deg}]]

   [:div.placeholder
    "Runtime, amp-heures, températures, version : Phase 3."]])

;; ===========================================================================
;; [J] Navigation onglets & app root
;; ===========================================================================

(def tabs
  [[:control      "BARRE"]
   [:gain         "GAINS"]
   [:calibration  "CALIB"]
   [:config       "CONFIG"]
   [:stats        "STATS"]])

(defn tab-nav []
  (let [cur (:tab @state)]
    [:nav.tab-nav
     (for [[id label] tabs]
       ^{:key id}
       [:button.tab-btn {:class (when (= id cur) "active")
                         :on-click #(do (swap! state assoc :tab id)
                                        (set! (.-hash js/location) (name id)))}
        label])]))

(defn app []
  (let [t (:tab @state)]
    [:div#app {:data-theme (:theme @state)}
     [status-bar]
     [:div.tab-content
      (case t
        :control     [tab-control]
        :gain        [tab-gain]
        :calibration [tab-calibration]
        :config      [tab-config]
        :stats       [tab-stats]
        [tab-control])]
     [tab-nav]]))

;; ===========================================================================
;; [K] Init
;; ===========================================================================

(defn ^:dev/after-load mount! []
  (.setAttribute (.-documentElement js/document) "data-theme" (:theme @state))
  (rdom/render [app] (.getElementById js/document "app")))

(connect!)
(mount!)

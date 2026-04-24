(ns main
  (:require
   [reagent.core :as r]
   [reagent.dom  :as rdom]))

;; ---------------------------------------------------------------------------
;; Config
;; ---------------------------------------------------------------------------

(def ws-url
  (let [host (or (let [h (.-hostname js/location)]
                      (when (and h (pos? (.-length h))) h))
                 "localhost")
        scheme (if (or (= (.-protocol js/location) "https:")
                      (= (.-protocol js/location) "wss:"))
                 "wss:"
                 "ws:")
        url (str scheme "//" host ":23323")]
    (js/console.log "[UI] WebSocket URL:" url)
    url))

;; ---------------------------------------------------------------------------
;; État global
;; ---------------------------------------------------------------------------

(defonce state
  (r/atom {:connected   false
           :ws          nil
           :heading     nil      ; degrés (lowpass)
           :heading-cmd nil      ; cap commandé
           :ap-enabled  false
           :heading-err nil
           :pitch       nil
           :roll        nil
           :heel        nil
           :voltage     nil
           :servo-conn  false
           :sk-conn     false
           :imu-freq    nil}))

;; ---------------------------------------------------------------------------
;; Helpers
;; ---------------------------------------------------------------------------

(defn- fmt-deg
  "Formate un angle en '045°' (3 chiffres, avec signe pour erreur)."
  ([v] (fmt-deg v false))
  ([v signed?]
   (if (nil? v) "---"
     (let [n (js/Math.round v)]
       (if signed?
         (str (if (>= n 0) "+" "") n "°")
         (str (-> (str "00" (js/Math.abs n)) (.slice -3)) "°"))))))

(defn- fmt-1 [v]
  (if (nil? v) "--.-" (.toFixed v 1)))

(defn- parse-num [v]
  (let [n (js/parseFloat v)]
    (when-not (js/isNaN n) n)))

(defn- resolv [a]
  (loop [a a]
    (cond (> a  180) (recur (- a 360))
          (< a -180) (recur (+ a 360))
          :else a)))

;; ---------------------------------------------------------------------------
;; WebSocket
;; ---------------------------------------------------------------------------

(defn- parse-line! [line]
  (let [idx (.indexOf line "=")]
    (when (>= idx 0)
      (let [k (.substring line 0 idx)
            v (.substring line (inc idx))]
        (case k
          "signalk.heading"
          (when-let [n (parse-num v)]
            (swap! state assoc :heading n))

          "ap.heading_command"
          (when-let [n (parse-num v)]
            (swap! state assoc :heading-cmd n))

          "ap.enabled"
          (swap! state assoc :ap-enabled (= v "true"))

          "ap.heading_error"
          (when-let [n (parse-num v)]
            (swap! state assoc :heading-err n))

          "signalk.pitch"
          (when-let [n (parse-num v)]
            (swap! state assoc :pitch n))

          "signalk.roll"
          (when-let [n (parse-num v)]
            (swap! state assoc :roll n))

          "signalk.heel"
          (when-let [n (parse-num v)]
            (swap! state assoc :heel n))

          "servo.voltage"
          (when-let [n (parse-num v)]
            (swap! state assoc :voltage n))

          "servo.connected"
          (swap! state assoc :servo-conn (= v "true"))

          "signalk.connected"
          (swap! state assoc :sk-conn (= v "true"))

          nil)))))

(defn- send! [s]
  (let [ws (:ws @state)]
    (when (and ws (= (.-readyState ws) 1))
      (.send ws s))))

(defn- watch-msg [m]
  (str "watch=" (.stringify js/JSON (clj->js m)) "\n"))

(defn- subscribe! []
  (send! (watch-msg
          {"signalk.heading"     0
           "ap.heading_command"  0
           "ap.enabled"          0
           "ap.heading_error"    0
           "signalk.pitch"       0.5
           "signalk.roll"        0.5
           "signalk.heel"        0.5
           "servo.voltage"       1
           "servo.connected"     0
           "signalk.connected"   0})))

(defn connect! []
  (let [ws (js/WebSocket. ws-url)
        buf (atom "")]
    (set! (.-onopen ws)
          (fn [_]
            (js/console.log "[UI] WS connected to" ws-url)
            (swap! state assoc :connected true :ws ws)
            (subscribe!)))
    (set! (.-onmessage ws)
          (fn [evt]
            (let [data (.-data evt)]
              (js/console.log "[UI] WS recv:" data)
              (swap! buf str data)
              (loop []
                (let [s @buf
                      i (.indexOf s "\n")]
                  (when (>= i 0)
                    (let [line (.substring s 0 i)]
                      (reset! buf (.substring s (inc i)))
                      (parse-line! line)
                      (recur))))))))
    (set! (.-onclose ws)
          (fn [_]
            (js/console.log "[UI] WS closed")
            (swap! state assoc :connected false :ws nil)
            (js/setTimeout connect! 3000)))
    (set! (.-onerror ws)
          (fn [evt]
            (js/console.error "[UI] WS error:" evt)
            (.close ws)))))

;; ---------------------------------------------------------------------------
;; Commandes AP
;; ---------------------------------------------------------------------------

(defn- set-ap! [enabled?]
  (send! (str "ap.enabled=" (if enabled? "true" "false") "\n")))

(defn- adjust-heading! [delta]
  (let [cur (or (:heading-cmd @state) (:heading @state) 0)
        nxt (mod (+ cur delta) 360)]
    (send! (str "ap.heading_command=" nxt "\n"))))

;; ---------------------------------------------------------------------------
;; Composants SVG — Boussole
;; ---------------------------------------------------------------------------

(def ^:private cardinal ["N" "NE" "E" "SE" "S" "SO" "O" "NO"])

(defn- compass-rose [heading heading-cmd]
  (let [r     120    ; rayon externe
        cx    130
        cy    130
        rot   (or heading 0)
        cmd-a (or heading-cmd 0)]
    [:svg {:viewBox "0 0 260 260" :xmlns "http://www.w3.org/2000/svg"}

     ;; Fond cercle
     [:circle {:cx cx :cy cy :r r :fill "#0d1921" :stroke "#2a3f58" :stroke-width 1.5}]
     [:circle {:cx cx :cy cy :r (- r 18) :fill "none" :stroke "#1a2f42" :stroke-width 1}]

     ;; Rose des vents (tourne avec le cap)
     [:g {:transform (str "rotate(" (- rot) " " cx " " cy ")")}

      ;; Graduations toutes les 5°
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

      ;; Labels cardinaux
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

     ;; Indicateur cap commandé (triangle cyan, fixe dans le référentiel boussole mais tourne avec rot)
     (when heading-cmd
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

     ;; Ligne de foi (lubber line) — fixe, pointe vers le haut
     [:line {:x1 cx :y1 (- cy r -2) :x2 cx :y2 (- cy r 18)
             :stroke "#ff4455" :stroke-width 3 :stroke-linecap "round"}]

     ;; Centre
     [:circle {:cx cx :cy cy :r 5 :fill "#1d2a3a" :stroke "#2a3f58" :stroke-width 1.5}]]))

;; ---------------------------------------------------------------------------
;; Composants UI
;; ---------------------------------------------------------------------------

(defn- status-bar []
  (let [{:keys [connected sk-conn voltage imu-freq]} @state]
    [:div.status-bar
     [:div.dot {:class (if connected "ok" "err")}]
     [:span (if connected "connecté" "hors ligne")]
     (when sk-conn
       [:span.badge.sk "Signal K"])
     [:span.spacer]
     (when voltage
       [:span.voltage (str (.toFixed voltage 1) "V")])
     (when imu-freq
       [:span (str (.toFixed imu-freq 0) "Hz")])]))

(defn- compass-section []
  (let [{:keys [heading heading-cmd]} @state]
    [:div.compass-section
     [:div.compass-wrap
      [compass-rose heading heading-cmd]]
     [:div.heading-display
      (fmt-deg heading)]]))

(defn- sensor-grid []
  (let [{:keys [pitch roll heel heading-err]} @state
        err-class (cond
                    (nil? heading-err) ""
                    (> (js/Math.abs heading-err) 20) "err"
                    (> (js/Math.abs heading-err) 10) "warn"
                    :else "")]
    [:div.sensor-grid
     [:div.sensor-cell
      [:div.label "PITCH"]
      [:div.value (fmt-1 pitch) "°"]]
     [:div.sensor-cell
      [:div.label "ROLL"]
      [:div.value (fmt-1 roll) "°"]]
     [:div.sensor-cell
      [:div.label "GÎTE"]
      [:div.value (fmt-1 heel) "°"]]
     [:div.sensor-cell
      [:div.label "ERREUR"]
      [:div {:class (str "value " err-class)}
       (fmt-deg heading-err true)]]]))

(defn- target-row []
  (let [{:keys [heading-cmd heading-err]} @state]
    [:div.target-row
     [:span "COMMANDÉ"]
     [:span.tval (fmt-deg heading-cmd)]
     (when (and heading-err (not (nil? heading-err)))
       [:span {:class (str "err-val" (cond
                                       (> (js/Math.abs heading-err) 20) " warn"
                                       :else ""))}
        (str "Δ " (fmt-deg heading-err true))])]))

(defn- controls []
  (let [{:keys [ap-enabled connected]} @state]
    [:div.controls
     [:button.btn {:on-click #(adjust-heading! -10) :disabled (not connected)} "−10°"]
     [:button.btn {:on-click #(adjust-heading! -1)  :disabled (not connected)} "−1°"]
     [:button {:class (str "btn btn-ap" (when ap-enabled " on"))
               :on-click #(set-ap! (not ap-enabled))
               :disabled (not connected)}
      (if ap-enabled "AP ON" "AP OFF")]
     [:button.btn {:on-click #(adjust-heading! 1)   :disabled (not connected)} "+1°"]
     [:button.btn {:on-click #(adjust-heading! 10)  :disabled (not connected)} "+10°"]]))

;; ---------------------------------------------------------------------------
;; App root
;; ---------------------------------------------------------------------------

(defn app []
  [:div#app
   [status-bar]
   [compass-section]
   [sensor-grid]
   [target-row]
   [controls]])

;; ---------------------------------------------------------------------------
;; Init
;; ---------------------------------------------------------------------------

(defn ^:dev/after-load mount! []
  (rdom/render [app] (.getElementById js/document "app")))

(connect!)
(mount!)

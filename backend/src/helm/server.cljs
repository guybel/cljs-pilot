(ns helm.server
  (:require ["net" :as net]
            [helm.values :as v]
            [clojure.string :as str]))

;; WsServer résolu via js/require — (.-Server ws-lib) retourne undefined avec nbb/CJS
(def ^:private WsServer (.-Server (js/require "ws")))

;; ---------------------------------------------------------------------------
;; État
;; ---------------------------------------------------------------------------

;; watches : {name [{:conn socket-or-ws :period number-or-0 :next-time ms}]}
(defonce watches (atom {}))

;; sockets actifs (TCP)
(defonce sockets (atom #{}))

;; clients WebSocket actifs
(defonce ws-clients (atom #{}))

(def max-connections 30)
(def default-port 23322)
(def default-ws-port 23323)
(def store-period-ms (* 60 1000))

;; ---------------------------------------------------------------------------
;; Écriture sur connexion (safe — TCP socket ou WebSocket)
;; ---------------------------------------------------------------------------

(defn- write! [conn s]
  (cond
    ;; WebSocket : readyState 1 = OPEN
    (and conn (.-readyState conn) (= (.-readyState conn) 1))
    (.send conn s)
    ;; TCP socket
    (and conn (.-writable conn))
    (.write conn s)))

;; ---------------------------------------------------------------------------
;; Notification des watchers
;; ---------------------------------------------------------------------------

(defn- notify-immediate!
  "Envoie name=value aux watchers de period 0. Appelé par values/on-change!"
  [name _value]
  (when-let [ws (get @watches name)]
    (let [line (v/wire-line name)]
      (doseq [{:keys [conn period]} ws]
        (when (zero? period)
          (write! conn line))))))

(defn- poll-periodic!
  "Envoie les valeurs aux watchers périodiques dont le délai est échu."
  []
  (let [now (js/Date.now)]
    (swap! watches
           (fn [ws]
             (reduce-kv
              (fn [acc name entries]
                (assoc acc name
                       (mapv (fn [{:keys [conn period next-time] :as w}]
                               (if (and (pos? period) (>= now next-time))
                                 (do (write! conn (v/wire-line name))
                                     (assoc w :next-time (+ now (* period 1000))))
                                 w))
                             entries)))
              {}
              ws)))))

;; ---------------------------------------------------------------------------
;; Gestion des watches par connexion
;; ---------------------------------------------------------------------------

(defn- add-watch! [conn name period]
  (swap! watches
         (fn [ws]
           (let [entries (get ws name [])
                 ;; supprimer l'entrée existante pour cette connexion
                 filtered (filterv #(not= (:conn %) conn) entries)
                 new-entry {:conn conn
                            :period period
                            :next-time (+ (js/Date.now) (* period 1000))}]
             (assoc ws name (conj filtered new-entry)))))
  ;; envoi immédiat de la valeur courante si elle existe
  (when (v/get-entry name)
    (write! conn (v/wire-line name))))

(defn- remove-watch! [conn name]
  (swap! watches
         (fn [ws]
           (let [entries (filterv #(not= (:conn %) conn) (get ws name []))]
             (if (empty? entries)
               (dissoc ws name)
               (assoc ws name entries))))))

(defn- remove-all-watches! [conn]
  (swap! watches
         (fn [ws]
           (reduce-kv
            (fn [acc name entries]
              (let [filtered (filterv #(not= (:conn %) conn) entries)]
                (if (empty? filtered)
                  (dissoc acc name)
                  (assoc acc name filtered))))
            {}
            ws))))

;; ---------------------------------------------------------------------------
;; Parsing des commandes
;; ---------------------------------------------------------------------------

(defn- values-msg
  "Construit la ligne 'values={...}\n' avec les métadonnées de toutes les valeurs."
  []
  (let [entries (for [name (v/all-names)
                      :let [info (v/get-info name)]
                      :when info]
                  (str "\"" name "\":" (js/JSON.stringify (clj->js info))))]
    (str "values={" (str/join "," entries) "}\n")))

(defn- handle-watch-cmd! [conn data-str]
  (try
    (let [watch-map (js->clj (js/JSON.parse data-str) :keywordize-keys false)]
      (doseq [[name period-raw] watch-map]
        (cond
          (false? period-raw)   (remove-watch! conn name)
          (or (zero? period-raw)
              (true? period-raw)) (add-watch! conn name 0)
          (number? period-raw)   (add-watch! conn name period-raw)
          :else                  (remove-watch! conn name))))
    (catch :default e
      (write! conn (str "error=invalid watch: " e "\n")))))

(defn- handle-set-cmd! [conn name value-str]
  (let [entry (v/get-entry name)]
    (cond
      (nil? entry)
      (write! conn (str "error=unknown value: " name "\n"))

      (not (get-in entry [:info :writable]))
      (write! conn (str "error=" name " is not writable\n"))

      :else
      (let [parsed (try (js/JSON.parse value-str)
                        (catch :default _ value-str))]
        (v/set-value! name parsed)))))

(defn- handle-line! [conn line]
  (when-not (str/blank? line)
    (let [idx (.indexOf line "=")]
      (if (neg? idx)
        (write! conn (str "error=invalid message: " line "\n"))
        (let [name  (.substring line 0 idx)
              data  (.substring line (inc idx))]
          (cond
            (= name "watch")   (handle-watch-cmd! conn data)
            (= name "values")  nil ;; clients ne peuvent pas s'enregistrer
            :else              (handle-set-cmd! conn name data)))))))

;; ---------------------------------------------------------------------------
;; Connexion TCP
;; ---------------------------------------------------------------------------

(defn- on-connect! [socket]
  (if (>= (count @sockets) max-connections)
    (do (write! socket "error=too many connections\n")
        (.destroy socket))
    (do
      (swap! sockets conj socket)
      (.setEncoding socket "utf8")
      (.setNoDelay socket true)

      ;; Envoi initial : liste de toutes les valeurs avec métadonnées
      (write! socket (values-msg))

      (let [buf (atom "")]
        (.on socket "data"
             (fn [chunk]
               (swap! buf str chunk)
               (loop []
                 (let [s @buf
                       i (.indexOf s "\n")]
                   (when (>= i 0)
                     (let [line (.substring s 0 i)]
                       (reset! buf (.substring s (inc i)))
                       (handle-line! socket line)
                       (recur))))))))

      (.on socket "close"
           (fn [_]
             (swap! sockets disj socket)
             (remove-all-watches! socket)))

      (.on socket "error"
           (fn [_]
             (swap! sockets disj socket)
             (remove-all-watches! socket))))))

;; ---------------------------------------------------------------------------
;; Connexion WebSocket
;; ---------------------------------------------------------------------------

(defn- on-ws-connect! [ws-conn]
  (if (>= (+ (count @sockets) (count @ws-clients)) max-connections)
    (do (.send ws-conn "error=too many connections\n")
        (.close ws-conn))
    (do
      (swap! ws-clients conj ws-conn)

      ;; Envoi initial : liste de toutes les valeurs avec métadonnées
      (.send ws-conn (values-msg))

      (let [buf (atom "")]
        (.on ws-conn "message"
             (fn [data]
               (swap! buf str (str data))
               (loop []
                 (let [s @buf
                       i (.indexOf s "\n")]
                   (when (>= i 0)
                     (let [line (.substring s 0 i)]
                       (reset! buf (.substring s (inc i)))
                       (handle-line! ws-conn line)
                       (recur))))))))

      (.on ws-conn "close"
           (fn [_ _]
             (swap! ws-clients disj ws-conn)
             (remove-all-watches! ws-conn)))

      (.on ws-conn "error"
           (fn [_]
             (swap! ws-clients disj ws-conn)
             (remove-all-watches! ws-conn))))))

;; ---------------------------------------------------------------------------
;; Démarrage
;; ---------------------------------------------------------------------------

(defonce server-instance (atom nil))
(defonce ws-server-instance (atom nil))

(defn start!
  ([] (start! {}))
  ([{:keys [port ws-port]
     :or   {port default-port ws-port default-ws-port}}]
   ;; Brancher le hook on-change dans values
   (reset! v/on-change! notify-immediate!)

   ;; Chargement de la config
   (v/load-conf!)

   (let [srv (doto (.createServer net on-connect!)
               (.on "error" (fn [e] (js/console.error "server error:" e)))
               (.listen port (fn [] (js/console.log (str "helmpilot TCP server listening on port " port)))))]
     (reset! server-instance srv))

   ;; Serveur WebSocket
   (let [wss (new WsServer #js {:port ws-port})]
     (.on wss "connection" on-ws-connect!)
     (.on wss "error" (fn [e] (js/console.error "ws server error:" e)))
     (js/console.log (str "helmpilot WebSocket server listening on port " ws-port))
     (reset! ws-server-instance wss))

   ;; Polls périodiques (watches > 0) — toutes les 100ms
   (js/setInterval poll-periodic! 100)

   ;; Sauvegarde config — toutes les 60s
   (js/setInterval v/store-conf! store-period-ms)))

(defn broadcast-updated-values!
  "Envoie un nouveau message 'values={...}' à tous les clients connectés.
   À appeler si de nouvelles valeurs sont enregistrées après le démarrage."
  []
  (let [msg (values-msg)]
    (doseq [sock @sockets]
      (write! sock msg))
    (doseq [wsc @ws-clients]
      (write! wsc msg))))

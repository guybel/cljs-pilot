(ns helm.values)

;; Registre central de toutes les valeurs pypilot.
;;
;; Chaque entrée dans le registre est une map :
;;   {:value    <valeur courante>
;;    :info     {:type "RangeProperty" :min 0 :max 1 :writable true ...}
;;    :initial  <valeur initiale, pour ResettableValue>}
;;
;; Types reconnus : Value, JSONValue, RoundedValue, StringValue, SensorValue,
;;                  BooleanValue, BooleanProperty, Property,
;;                  RangeProperty, RangeSetting, EnumProperty, ResettableValue

(defonce registry (atom {}))

;; Hook appelé par le serveur TCP pour notifier les clients watchers.
;; Signature : (fn [name value])
(defonce on-change! (atom nil))

;; ---- Enregistrement --------------------------------------------------------

(defn register!
  "Enregistre une valeur. info-overrides : map additionnelle mergée dans :info."
  [name base-type initial & {:keys [info persistent? profiled?]}]
  (let [base-info (merge {:type base-type}
                         (when persistent? {:persistent true})
                         (when profiled?   {:persistent true :profiled true})
                         info)]
    (swap! registry assoc name
           {:value   initial
            :initial initial
            :info    base-info}))
  name)

;; Helpers de construction (miroir des constructeurs Python)

(defn value!       [name initial & opts] (apply register! name "Value"           initial opts))
(defn json-value!  [name initial & opts] (apply register! name "JSONValue"       initial opts))
(defn string-value![name initial & opts] (apply register! name "StringValue"     initial opts))
(defn sensor-value![name initial & {:keys [fmt directional] :or {fmt "%.4f"} :as opts}]
  (register! name "SensorValue" initial
             :info (cond-> {:fmt fmt} directional (assoc :directional true))
             :persistent? (:persistent? opts)))

(defn rounded-value![name initial & opts] (apply register! name "RoundedValue" initial opts))

(defn boolean-value!   [name initial & opts] (apply register! name "BooleanValue"    initial opts))
(defn boolean-property![name initial & opts] (apply register! name "BooleanProperty" initial (concat opts [:info {:writable true}])))

(defn property![name initial & opts]
  (apply register! name "Property" initial (concat opts [:info {:writable true}])))

(defn range-property!
  [name initial mn mx & opts]
  (apply register! name "RangeProperty" initial
         (concat opts [:info {:writable true :min mn :max mx}])))

(defn range-setting!
  [name initial mn mx units & opts]
  (apply register! name "RangeSetting" initial
         (concat opts [:info {:writable true :min mn :max mx :units units} :persistent? true])))

(defn enum-property!
  [name initial choices & opts]
  (apply register! name "EnumProperty" initial
         (concat opts [:info {:writable true :choices choices}])))

(defn resettable-value!
  [name initial & opts]
  (apply register! name "ResettableValue" initial
         (concat opts [:info {:writable true}])))

;; ---- Accès -----------------------------------------------------------------

(defn get-entry [name] (get @registry name))
(defn get-value [name] (get-in @registry [name :value]))
(defn get-info  [name] (get-in @registry [name :info]))
(defn all-names []     (keys @registry))

;; ---- Formatage wire --------------------------------------------------------

(defn- round4 [v]
  (.toFixed v 4))

(defn- format-value [v fmt]
  (cond
    (nil? v)     "false"
    (boolean? v) (if v "true" "false")
    (string? v)  (str "\"" v "\"")
    (vector? v)  (str "[" (clojure.string/join ", " (map #(format-value % fmt) v)) "]")
    (seq? v)     (format-value (vec v) fmt)
    (number? v)  (if fmt
                   (.replace (.toFixed v 4) #"\.?0+$" "")
                   (str v))
    :else        (str v)))

(defn get-msg
  "Retourne la représentation wire du nom=valeur (juste la valeur)."
  [name]
  (let [{:keys [value info]} (get @registry name)
        t (:type info "Value")]
    (case t
      "BooleanValue"    (if value "true" "false")
      "BooleanProperty" (if value "true" "false")
      "RangeProperty"   (round4 value)
      "RangeSetting"    (round4 value)
      "RoundedValue"    (round4 value)
      "SensorValue"     (format-value value (:fmt info "%.4f"))
      "ResettableValue" (if-let [fmt (:fmt info)] (round4 value) (str value))
      "JSONValue"       (js/JSON.stringify (clj->js value))
      ;; StringValue, Value, Property, EnumProperty
      (format-value value nil))))

(defn wire-line
  "Retourne la ligne complète 'name=value\\n'."
  [name]
  (str name "=" (get-msg name) "\n"))

;; ---- Modification ----------------------------------------------------------

(defn- valid? [info v]
  (let [t (:type info "Value")]
    (case t
      "RangeProperty" (and (number? v) (>= v (:min info)) (<= v (:max info)))
      "RangeSetting"  (and (number? v) (>= v (:min info)) (<= v (:max info)))
      "EnumProperty"  (some #(= (str %) (str v)) (:choices info))
      "BooleanProperty" true
      true)))

(defn set-value!
  "Met à jour la valeur et déclenche le hook on-change!."
  [name v]
  (when-let [entry (get @registry name)]
    (let [info  (:info entry)
          t     (:type info "Value")
          ;; ResettableValue : false → réinitialise à initial
          coerced (cond
                    (and (= t "ResettableValue") (not v)) (:initial entry)
                    (= t "BooleanProperty")               (boolean v)
                    (#{t} #{"RangeProperty" "RangeSetting"})
                    (let [n (js/parseFloat v)]
                      (when-not (js/isNaN n) n))
                    :else v)]
      (when (and coerced (valid? info coerced))
        (swap! registry assoc-in [name :value] coerced)
        (when-let [hook @on-change!]
          (hook name coerced))))))

(defn update-value!
  "set-value! uniquement si la nouvelle valeur diffère."
  [name v]
  (when (not= v (get-value name))
    (set-value! name v)))

;; ---- Persistance -----------------------------------------------------------

(def ^:private conf-path
  (str (.-HOME (.-env js/process)) "/.helmpilot/helmpilot.conf"))

;; ---- Inspection -----------------------------------------------------------

(defn snapshot
  "Retourne le registre sous forme d'une map triée {:name {:value ... :info ...}}
   Pratique depuis le REPL : (cljs.pprint/pprint (helm.values/snapshot))"
  []
  (into (sorted-map)
        (map (fn [[k {:keys [value info]}]]
               [k {:value value :type (:type info "Value")}])
             @registry)))

(defn dump!
  "Affiche toutes les valeurs courantes, triées par nom."
  []
  (doseq [[name {:keys [value]}] (into (sorted-map) @registry)]
    (js/console.log (str name " = " (get-msg name)))))

;; ---- Persistance -----------------------------------------------------------

(defn load-conf! []
  (try
    (let [fs (js/require "fs")
          content (.readFileSync fs conf-path "utf8")]
      (doseq [line (.split content "\n")]
        (when-let [[_ k v] (re-matches #"^([^=\s]+)=(.*)$" line)]
          (when (get @registry k)
            (set-value! k (try (js/JSON.parse v) (catch :default _ v)))))))
    (catch :default _)))

(defn store-conf! []
  (try
    (let [fs   (js/require "fs")
          path (js/require "path")
          dir  (.dirname path conf-path)]
      (.mkdirSync fs dir #js {:recursive true})
      (let [lines (for [[name {:keys [value info]}] @registry
                        :when (:persistent info)]
                    (str name "=" (get-msg name)))]
        (.writeFileSync fs conf-path (clojure.string/join "\n" lines))))
    (catch :default e
      (js/console.error "store-conf! error:" e))))

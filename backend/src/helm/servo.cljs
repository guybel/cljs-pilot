(ns helm.servo
  (:require [helm.values :as v]))

;; Port de pypilot/arduino_servo/arduino_servo_python.py
;;
;; Protocole binaire ArduinoServo :
;;   Envoi  : 8 sync bytes rotatifs ; chaque paquet = [lo, hi, crc8(sync,lo,hi)]
;;   CRC8   : init=0xFF, table 256 valeurs MAXIM/Dallas
;;   raw_command : à out_sync==0 → envoyer limit_courant d'abord, puis cmd
;;   Cmd normalisée [-1,1] → raw = (cmd+1)*1000 ; stop = 0x5342
;;   Réception : in_sync==0 → voltage+flags ; in_sync>0 → courant
;;   OVERCURRENT (bit 2 des flags) → stop immédiat

;; ---------------------------------------------------------------------------
;; CRC8 — init=0xFF, algo MAXIM/Dallas (polynôme 0x31)
;; ---------------------------------------------------------------------------

(def ^:private CRC8-TABLE
  (js/Uint8Array.from
    #js [0x00 0x31 0x62 0x53 0xC4 0xF5 0xA6 0x97
         0xB9 0x88 0xDB 0xEA 0x7D 0x4C 0x1F 0x2E
         0x43 0x72 0x21 0x10 0x87 0xB6 0xE5 0xD4
         0xFA 0xCB 0x98 0xA9 0x3E 0x0F 0x5C 0x6D
         0x86 0xB7 0xE4 0xD5 0x42 0x73 0x20 0x11
         0x3F 0x0E 0x5D 0x6C 0xFB 0xCA 0x99 0xA8
         0xC5 0xF4 0xA7 0x96 0x01 0x30 0x63 0x52
         0x7C 0x4D 0x1E 0x2F 0xB8 0x89 0xDA 0xEB
         0x3D 0x0C 0x5F 0x6E 0xF9 0xC8 0x9B 0xAA
         0x84 0xB5 0xE6 0xD7 0x40 0x71 0x22 0x13
         0x7E 0x4F 0x1C 0x2D 0xBA 0x8B 0xD8 0xE9
         0xC7 0xF6 0xA5 0x94 0x03 0x32 0x61 0x50
         0xBB 0x8A 0xD9 0xE8 0x7F 0x4E 0x1D 0x2C
         0x02 0x33 0x60 0x51 0xC6 0xF7 0xA4 0x95
         0xF8 0xC9 0x9A 0xAB 0x3C 0x0D 0x5E 0x6F
         0x41 0x70 0x23 0x12 0x85 0xB4 0xE7 0xD6
         0x7A 0x4B 0x18 0x29 0xBE 0x8F 0xDC 0xED
         0xC3 0xF2 0xA1 0x90 0x07 0x36 0x65 0x54
         0x39 0x08 0x5B 0x6A 0xFD 0xCC 0x9F 0xAE
         0x80 0xB1 0xE2 0xD3 0x44 0x75 0x26 0x17
         0xFC 0xCD 0x9E 0xAF 0x38 0x09 0x5A 0x6B
         0x45 0x74 0x27 0x16 0x81 0xB0 0xE3 0xD2
         0xBF 0x8E 0xDD 0xEC 0x7B 0x4A 0x19 0x28
         0x06 0x37 0x64 0x55 0xC2 0xF3 0xA0 0x91
         0x47 0x76 0x25 0x14 0x83 0xB2 0xE1 0xD0
         0xFE 0xCF 0x9C 0xAD 0x3A 0x0B 0x58 0x69
         0x04 0x35 0x66 0x57 0xC0 0xF1 0xA2 0x93
         0xBD 0x8C 0xDF 0xEE 0x79 0x48 0x1B 0x2A
         0xC1 0xF0 0xA3 0x92 0x05 0x34 0x67 0x56
         0x78 0x49 0x1A 0x2B 0xBC 0x8D 0xDE 0xEF
         0x82 0xB3 0xE0 0xD1 0x46 0x77 0x24 0x15
         0x3B 0x0A 0x59 0x68 0xFF 0xCE 0x9D 0xAC]))

(defn- crc8 [b0 b1 b2]
  (aget CRC8-TABLE
        (bit-xor (aget CRC8-TABLE
                       (bit-xor (aget CRC8-TABLE
                                      (bit-xor 0xFF b0))
                                b1))
                 b2)))

;; ---------------------------------------------------------------------------
;; Constantes protocole
;; ---------------------------------------------------------------------------

(def ^:private SYNC-BYTES  #js [0xe7 0xf9 0xc7 0x1e 0xa7 0x19 0x1c 0xb3])
(def ^:private STOP-CMD    0x5342)
(def ^:private FLAG-OVERCURRENT 0x04)

;; ---------------------------------------------------------------------------
;; État
;; ---------------------------------------------------------------------------

(defonce state
  (atom {:port          nil    ; instance SerialPort
         :out-sync      0      ; 0-7, rotation sync bytes à l'envoi
         :in-sync       0      ; 0-7, rotation sync bytes à la réception
         :in-sync-count 0      ; 0,1,2 — 2 = deux cycles complets = synchronisé
         :in-buf        #js [] ; buffer d'octets entrants
         :max-current   4.5    ; A — limite envoyée au début de chaque cycle
         :connected     false}))

;; ---------------------------------------------------------------------------
;; Enregistrement des valeurs
;; ---------------------------------------------------------------------------

(defn register-values! []
  (v/sensor-value!  "servo.command"   0)
  (v/sensor-value!  "servo.voltage"   false)
  (v/sensor-value!  "servo.current"   false)
  (v/sensor-value!  "servo.flags"     0)
  (v/boolean-value! "servo.connected" false))

;; ---------------------------------------------------------------------------
;; Envoi (host → Arduino)
;; ---------------------------------------------------------------------------

(defn- send-value! [value]
  (when-let [port (:port @state)]
    (let [out-sync (:out-sync @state)
          v   (bit-and (int value) 0xFFFF)
          lo  (bit-and v 0xFF)
          hi  (bit-and (unsigned-bit-shift-right v 8) 0xFF)
          syn (aget SYNC-BYTES out-sync)
          crc (crc8 syn lo hi)
          buf (js/Buffer.from #js [lo hi crc])]
      (.write port buf)
      (swap! state update :out-sync #(let [s (inc %)] (if (= s 8) 0 s))))))

(defn- raw-command! [raw-val]
  ;; À out_sync==0 : envoyer la limite de courant avant la commande
  (when (zero? (:out-sync @state))
    (let [limit (int (* (:max-current @state) 65536.0 0.05 (/ 1.0 1.1)))]
      (send-value! limit)))
  (send-value! raw-val))

;; ---------------------------------------------------------------------------
;; Réception / télémétrie (Arduino → host)
;; ---------------------------------------------------------------------------

(defn- process-in-buf! []
  (let [in-buf (:in-buf @state)]
    (loop []
      (when (>= (.-length in-buf) 3)
        (let [b0  (aget in-buf 0)
              b1  (aget in-buf 1)
              b2  (aget in-buf 2)
              syn (aget SYNC-BYTES (:in-sync @state))
              crc (crc8 syn b0 b1)]
          (if (= crc b2)
            (do
              ;; CRC valide — lire la télémétrie si synchronisé (2 cycles complets)
              (when (= (:in-sync-count @state) 2)
                (let [val (+ b0 (* b1 256))]
                  (if (zero? (:in-sync @state))
                    ;; Paquet voltage + flags (in_sync == 0 avant incrément)
                    (let [voltage (/ (* (unsigned-bit-shift-right val 4) 1.1 10560.0)
                                     (* 560.0 4096.0))
                          flags   (bit-and val 0xF)]
                      (v/update-value! "servo.voltage" voltage)
                      (v/update-value! "servo.flags"   flags)
                      (when (pos? (bit-and flags FLAG-OVERCURRENT))
                        (js/console.warn "[servo] OVERCURRENT — arrêt")
                        (raw-command! STOP-CMD)))
                    ;; Paquet courant (in_sync > 0)
                    (v/update-value! "servo.current"
                                     (* val 1.1 (/ 1.0 0.05) (/ 1.0 65536.0))))))
              ;; Avancer in_sync, décompter le cycle
              (.splice in-buf 0 3)
              (let [new-sync (mod (inc (:in-sync @state)) 8)]
                (swap! state assoc :in-sync new-sync)
                (when (and (zero? new-sync) (< (:in-sync-count @state) 2))
                  (swap! state update :in-sync-count inc))))

            (do
              ;; CRC invalide → resync : jeter 1 octet
              (.splice in-buf 0 1)
              (swap! state assoc :in-sync 0 :in-sync-count 0)))

          (recur))))))

(defn- handle-data! [data]
  (let [in-buf (:in-buf @state)]
    (doseq [b (array-seq data)]
      (.push in-buf b)))
  (process-in-buf!))

;; ---------------------------------------------------------------------------
;; Gestion du port
;; ---------------------------------------------------------------------------

(defn- handle-error! [e]
  ;; L'événement "close" suit toujours "error" — pas besoin de gérer ici
  (js/console.error "[servo] Erreur port série:" (.-message e)))

(defn- handle-close! []
  (js/console.log "[servo] Port série fermé")
  (v/update-value! "servo.connected" false)
  (swap! state assoc :port nil :connected false
         :out-sync 0 :in-sync 0 :in-sync-count 0))

;; ---------------------------------------------------------------------------
;; API publique
;; ---------------------------------------------------------------------------

(defn send-command!
  "Envoie une commande normalisée cmd ∈ [-1, 1] à l'Arduino.
   No-op si le port n'est pas connecté."
  [cmd]
  (when (:connected @state)
    (let [clamped (max -1.0 (min 1.0 (double cmd)))]
      (raw-command! (int (* (+ clamped 1.0) 1000))))))

(defn stop-servo! []
  (when (:connected @state)
    (raw-command! STOP-CMD)))

(defn start!
  "Ouvre le port série et démarre la communication ArduinoServo.
   cfg : {:port \"/dev/ttyUSB0\" :baud 38400}"
  [cfg]
  (register-values!)
  (let [port-path (or (:port cfg) "/dev/ttyUSB0")
        baud      (or (:baud cfg) 38400)
        fs        (js/require "fs")]
    (if-not (.existsSync fs port-path)
      (js/console.log (str "[servo] " port-path " absent — servo désactivé"))
      (try
        (let [SP (.-SerialPort (js/require "serialport"))
              p  (new SP #js {:path port-path :baudRate baud :autoOpen false})]
          (.on p "data"  handle-data!)
          (.on p "error" handle-error!)
          (.on p "close" handle-close!)
          (.open p
                 (fn [err]
                   (if err
                     (js/console.log
                      (str "[servo] " port-path " inaccessible ("
                           (.-message err) ") — servo désactivé"))
                     (do
                       (js/console.log (str "[servo] Connecté → " port-path " @" baud))
                       (swap! state assoc :port p :connected true
                              :out-sync 0 :in-sync 0 :in-sync-count 0
                              :in-buf #js [])
                       (v/update-value! "servo.connected" true))))))
        (catch :default e
          (js/console.error "[servo] Impossible de charger 'serialport':" (.-message e)))))))

(defn stop! []
  (stop-servo!)
  (when-let [port (:port @state)]
    (.close port))
  (swap! state assoc :port nil :connected false))

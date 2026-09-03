(ns helm.servo
  (:require ["http" :as http]))

(def ESP32_URL "http://192.168.42.10")

(defn send-motor-command [speed]
  "Envoie la commande au moteur via ESP32 WiFi"
  (let [url (str ESP32_URL "/motor/" speed)]
    (http/get url (fn [response]
                    (let [data (atom "")]
                      (.on response "data" (fn [chunk]
                                             (swap! data str (str chunk))))
                      (.on response "end" (fn []
                                            (js/console.log "[Servo] Motor:" @data))))))))


(defn init []
  (js/console.log "[Servo] Connecté à ESP32 WiFi: " ESP32_URL))


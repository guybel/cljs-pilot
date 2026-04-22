(ns utils.resolv)

;; Port de pypilot/resolv.py
;; Normalise un angle pour qu'il reste dans [offset-180, offset+180]

(defn resolv
  ([angle] (resolv angle 0))
  ([angle offset]
   (loop [a angle]
     (cond
       (> (- offset a) 180)  (recur (+ a 360))
       (<= (- offset a) -180) (recur (- a 360))
       :else a))))

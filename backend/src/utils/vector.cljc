(ns utils.vector
  #?(:cljs (:require-macros [utils.vector])))

;; Port de pypilot/vector.py

(defn norm [[x y z]]
  (Math/sqrt (+ (* x x) (* y y) (* z z))))

(defn normalize [v]
  (let [n (norm v)]
    (if (zero? n)
      v
      (mapv #(/ % n) v))))

(defn cross [[ax ay az] [bx by bz]]
  [(- (* ay bz) (* az by))
   (- (* az bx) (* ax bz))
   (- (* ax by) (* ay bx))])

(defn dot [[ax ay az] [bx by bz]]
  (+ (* ax bx) (* ay by) (* az bz)))

(defn sub [a b] (mapv - a b))

(defn add [a b] (mapv + a b))

(defn scale [v m] (mapv #(* % m) v))

(defn project [a b]
  (scale b (/ (dot a b) (dot b b))))

(defn dist [a b]
  (norm (sub a b)))

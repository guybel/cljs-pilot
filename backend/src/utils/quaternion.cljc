(ns utils.quaternion
  (:require [utils.vector :as v]))

;; Port de pypilot/quaternion.py

(defn angvec2quat [angle vec]
  (let [n (v/norm vec)
        fac (if (zero? n) 0 (/ (Math/sin (/ angle 2)) n))
        [vx vy vz] vec]
    [(Math/cos (/ angle 2))
     (* vx fac)
     (* vy fac)
     (* vz fac)]))

(defn angle [[q0 _ _ _]]
  (* 2 (Math/acos q0)))

(defn vec2vec2quat [a b]
  (let [n   (v/cross a b)
        fac (-> (/ (v/dot a b) (v/norm a) (v/norm b))
                (max -1)
                (min 1))
        ang (Math/acos fac)]
    (angvec2quat ang n)))

(defn multiply [[q0 q1 q2 q3] [r0 r1 r2 r3]]
  [(- (* q0 r0) (* q1 r1) (* q2 r2) (* q3 r3))
   (+ (* q0 r1) (* q1 r0) (* q2 r3) (- (* q3 r2)))
   (+ (* q0 r2) (- (* q1 r3)) (* q2 r0) (* q3 r1))
   (+ (* q0 r3) (* q1 r2) (- (* q2 r1)) (* q3 r0))])

(defn conjugate [[q0 q1 q2 q3]]
  [q0 (- q1) (- q2) (- q3)])

(defn normalize [q]
  (let [d (Math/sqrt (reduce #(+ %1 (* %2 %2)) 0 q))]
    (mapv #(/ % d) q)))

;; Fait tourner le vecteur v par le quaternion q
(defn rotvecquat [[vx vy vz] q]
  (let [w [0 vx vy vz]
        r (conjugate q)]
    (rest (multiply (multiply q w) r))))

;; Retourne [roll pitch heading] en radians
(defn toeuler [[q0 q1 q2 q3]]
  (let [roll    (Math/atan2 (+ (* 2 q2 q3) (* 2 q0 q1))
                            (- 1 (* 2 (+ (* q1 q1) (* q2 q2)))))
        pitch   (Math/asin  (min (max (* 2 (- (* q0 q2) (* q1 q3))) -1) 1))
        heading (Math/atan2 (+ (* 2 q1 q2) (* 2 q0 q3))
                            (- 1 (* 2 (+ (* q2 q2) (* q3 q3)))))]
    [roll pitch heading]))

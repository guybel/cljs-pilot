(ns dev
  (:require [nbb.repl :as repl]
            [main]
            [helm.values :as v]))

;; Démarre le backend + un socket REPL sur le port 7888.
;;
;; Connexion :   rlwrap nc localhost 7888
;; Inspection :  (helm.values/dump!)
;;               (pypilot.values/snapshot)
;;               @pypilot.values/registry

(repl/socket-repl {:port 7888})
(js/console.log "Socket REPL disponible sur le port 7888")
(js/console.log "  rlwrap nc localhost 7888")
(js/console.log "  => (helm.values/dump!)")

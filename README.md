# helmpilot (cljs-pilot)

Autopilote de voilier — port de [pypilot](https://github.com/pypilot/pypilot) en ClojureScript.

- **Backend** : [nbb](https://github.com/babashka/nbb) (Node.js + ClojureScript), serveur TCP+WebSocket compatible protocole pypilot
- **Frontend** : [Scittle](https://github.com/babashka/scittle) + Reagent, servi en HTTP par [cljs-josh](https://github.com/chr15m/cljs-josh) (PWA installable sur tablette)
- **Source de données** : serveur Signal K (actuellement) ou MPU-9250 I2C direct (prévu)

## Architecture

```
┌──────────────────┐   WebSocket ws://:23323    ┌───────────────┐
│ Frontend Scittle │ ◄────────────────────────► │ Backend nbb   │
│ (browser tablet) │   name=value\n             │ (ClojureScript)│
│   http://:8000   │                            │               │
└──────────────────┘                             └───────┬───────┘
                                                         │
                            ┌────────────────────────────┼────────────────────────┐
                            │                            │                        │
                     wss://  (SignalK)              I2C (MPU-9250)         Serial (Arduino)
                     192.168.40.100:3443          via imu_worker.js        /dev/ttyUSB0
                            │                            │                        │
                    ┌───────┴────────┐          ┌────────┴────────┐       ┌──────┴──────┐
                    │ OpenPlotter /  │          │ Attitude + Mag  │       │ Servo drive │
                    │ Signal K server│          │ (Mahony fusion) │       │             │
                    └────────────────┘          └─────────────────┘       └─────────────┘
```

## Démarrer l'app

### Prérequis

```bash
cd backend && npm install       # nbb, ws, i2c-bus, serialport
cd ..      && npm install       # cljs-josh (serveur HTTP)
```

### 1. Backend (terminal 1)

```bash
cd backend
npx nbb -cp src main.cljs
# ou : npm start
```

Logs attendus :

```
helmpilot backend démarrage (ClojureScript / nbb)
[config] Chargé: .../backend/config.edn
helmpilot WebSocket server listening on port 23323
helmpilot TCP server listening on port 23322
[SignalK] Connecté → wss://192.168.40.100:3443/signalk/v1/stream
```

### 2. Frontend (terminal 2)

```bash
npm run dev          # live-reload (dev)
# ou : npm start    # mode prod (--prod, pas de live-reload)
```

Puis ouvrir **http://localhost:8000** dans un browser, ou **http://\<ip-pi\>:8000** depuis la tablette. La boussole, pitch, roll et gîte se mettent à jour dès que le backend reçoit des données Signal K.

### Installation PWA sur tablette

Ouvrir l'URL du Pi dans Safari (iOS) ou Chrome (Android), puis :
- **iOS** : *Partager → Sur l'écran d'accueil*
- **Android** : bouton « Installer » dans le menu

L'app se lance en plein écran depuis l'icône, comme une app native.

### Déploiement sur le Pi

Le repo contient deux unit systemd : [deploy/helmpilot.service](deploy/helmpilot.service) (backend nbb) et [deploy/helmpilot-www.service](deploy/helmpilot-www.service) (frontend cljs-josh). Sur le Pi, après `git pull` :

```bash
cd ~/cljs-pilot && npm install
cd backend     && npm install && cd ..
sudo cp deploy/helmpilot.service deploy/helmpilot-www.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now helmpilot.service helmpilot-www.service
```

## Configuration

Tout se passe dans [backend/config.edn](backend/config.edn) :

```edn
{:server  {:port 23322 :ws-port 23323}
 :imu     {:source :signalk}           ; :boatimu | :signalk | :both
 :signalk {:enabled true
           :host "192.168.40.100"
           :port 3443
           :url  "wss://192.168.40.100:3443/signalk/v1/stream"}
 :servo   {:port "/dev/ttyUSB0" :baud 38400}
 :gains   {:P 0.003 :I 0.0 :D 0.09 :DD 0.075 :PR 0.005 :FF 0.6}}
```

## Phases du projet

| Phase | Statut | Contenu |
|---|---|---|
| 1. Mock IMU + serveur TCP | ✅ | Simulation + protocole pypilot |
| 2. IMU réel + Signal K | ✅ | MPU-9250 via I2C, client Signal K WebSocket |
| 3. Servo Arduino | ⏳ | Contrôle moteur de barre via série |
| 4. NMEA / GPS | ⏳ | Intégration navigation |
| 5. Calibration IMU | ⏳ | voir ci-dessous |

## Calibration IMU (futur)

Quand le MPU-9250 sera branché sur le Pi Zero 2W, la calibration (hard/soft iron compass + accel bias/scale) **réutilisera le code Python pypilot sans modification**.

Principe : le `CalibrationProcess` de pypilot parle déjà le protocole TCP `name=value\n` que ce backend implémente. Il suffit de le lancer en parallèle :

```bash
# Sur le Pi, en parallèle du backend CLJS
python3 -m pypilot.calibration_fit
```

Côté CLJS il faudra (non fait) :
- Enregistrer `imu.compass.calibration.*` et `imu.accel.calibration.*` dans [boatimu.cljs](backend/src/helm/boatimu.cljs)
- Appliquer ces coefficients dans [imu_worker.js](backend/imu_worker.js) avant la fusion Mahony

Ne fonctionne qu'avec source `:boatimu` (Signal K livre déjà des données fusionnées, pas les samples bruts).

## Arborescence

```
cljs-pilot/
├── backend/                  # Backend nbb
│   ├── main.cljs             # Point d'entrée
│   ├── config.edn            # Configuration
│   ├── imu_worker.js         # Worker MPU-9250 + Mahony
│   └── src/helm/
│       ├── server.cljs       # TCP + WebSocket
│       ├── values.cljs       # Registre central des valeurs
│       ├── autopilot.cljs    # Boucle de contrôle
│       ├── boatimu.cljs      # IMU matériel
│       ├── signalk.cljs      # Client Signal K
│       ├── servo.cljs        # Servo Arduino
│       ├── config.cljs       # Lecteur config.edn
│       └── pilots/           # PID basic
├── www/                      # Frontend servi par cljs-josh
│   ├── index.html
│   ├── main.cljs             # UI Reagent (boussole, capteurs, contrôles AP)
│   ├── style.css
│   ├── manifest.json         # PWA
│   ├── icon-192.png, icon-512.png
│   └── vendor/               # Scittle, Reagent, React (préminifiés)
├── deploy/
│   └── helmpilot-www.service # Unit systemd pour le Pi
└── package.json              # cljs-josh
```

## Dépannage

- **`[config] config.edn introuvable`** → lance depuis `backend/`, pas depuis la racine.
- **`[SignalK] Désactivé`** → `:signalk :enabled` est à `false` dans `config.edn`.
- **Frontend `WS closed` en boucle** → backend pas démarré ou pas sur le même réseau. Vérifie dans la console DevTools la ligne `[UI] WebSocket URL:`. La WS pointe sur le même hostname que la page HTTP, donc servir le frontend depuis le Pi évite tout problème de cross-host.
- **Tablette ne charge pas l'app** → vérifie que `helmpilot-www.service` tourne (`systemctl status helmpilot-www`) et que le port 8000 n'est pas filtré par un firewall.
- **Pas de GÎTE affichée** → filtre IIR lent, ~30-60 s pour converger au démarrage.

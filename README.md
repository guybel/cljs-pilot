# helmpilot (cljs-pilot)

Autopilote de voilier — port de [pypilot](https://github.com/pypilot/pypilot) en ClojureScript.

- **Backend** : [nbb](https://github.com/babashka/nbb) (Node.js + ClojureScript), serveur TCP+WebSocket compatible protocole pypilot
- **Frontend** : [Scittle](https://github.com/babashka/scittle) + Reagent dans une fenêtre [Neutralino](https://neutralino.js.org/)
- **Source de données** : serveur Signal K (actuellement) ou MPU-9250 I2C direct (prévu)

## Architecture

```
┌──────────────────┐   WebSocket ws://:23323    ┌───────────────┐
│ Frontend Scittle │ ◄────────────────────────► │ Backend nbb   │
│ (Neutralino)     │   name=value\n             │ (ClojureScript)│
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
cd ..      && npm install       # neu CLI
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
npx neu run
```

Une fenêtre Neutralino s'ouvre (clic droit → Inspect pour la console DevTools). La boussole, pitch, roll et gîte se mettent à jour dès que le backend reçoit des données Signal K.

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
├── resources/                # Frontend servi par Neutralino
│   ├── index.html
│   ├── main.cljs             # UI Reagent (boussole, capteurs, contrôles AP)
│   └── style.css
├── bin/                      # Binaires Neutralino
└── neutralino.config.json
```

## Dépannage

- **`[config] config.edn introuvable`** → lance depuis `backend/`, pas depuis la racine.
- **`[SignalK] Désactivé`** → `:signalk :enabled` est à `false` dans `config.edn`.
- **Frontend `WS closed` en boucle** → backend pas démarré ou pas sur le même réseau. Vérifie dans la console DevTools la ligne `[UI] WebSocket URL:`.
- **Pas de GÎTE affichée** → filtre IIR lent, ~30-60 s pour converger au démarrage.

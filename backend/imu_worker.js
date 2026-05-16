'use strict';
// IMU Worker — lit le BNO055 (Bosch, fusion 9-DOF en hardware) via I2C
// et envoie à 20 Hz le quaternion fusionné, les Euler, l'accel/gyro/mag bruts
// et le statut de calibration au thread principal via postMessage.
//
// Si I2C n'est pas disponible, bascule automatiquement en simulation.
//
// Le BNO055 en mode NDOF fait la fusion en hardware : pas de Mahony, pas de
// calibration manuelle au démarrage. La calibration est continue et exposée
// sur 4 niveaux (sys/gyr/acc/mag, 0-3) via le registre CALIB_STAT.

const { parentPort } = require('worker_threads');
const fs = require('fs');

// ---------------------------------------------------------------------------
// Constantes BNO055 (datasheet rev 1.4)
// ---------------------------------------------------------------------------
const BNO055_ADDR    = 0x29;     // ADR pin HIGH (board Adafruit/CJMCU par défaut)
const REG_CHIP_ID    = 0x00;
const REG_ACCEL_DATA = 0x08;     // début bloc 32 octets : accel+mag+gyro+euler+quat
const REG_CALIB_STAT = 0x35;
const REG_UNIT_SEL   = 0x3B;
const REG_OPR_MODE   = 0x3D;
const REG_PWR_MODE   = 0x3E;
const REG_SYS_TRIG   = 0x3F;

const CHIP_ID        = 0xA0;
const MODE_CONFIG    = 0x00;
const MODE_NDOF      = 0x0C;     // fusion 9-DOF, mag fast-calib activé
const PWR_NORMAL     = 0x00;
const UNIT_DEFAULT   = 0x00;     // accel m/s², gyro dps, euler degrés, temp °C

// Échelles datasheet (mode unités par défaut)
const QUAT_SCALE  = 1 / 16384.0; // Q14 sur 16 bits signés
const EULER_SCALE = 1 / 16.0;    // 1° = 16 LSB
const ACCEL_SCALE = 1 / 100.0;   // 1 m/s² = 100 LSB
const GYRO_SCALE  = 1 / 16.0;    // 1 dps = 16 LSB
const MAG_SCALE   = 1 / 16.0;    // 1 µT = 16 LSB
const DEG2RAD     = Math.PI / 180.0;
const G_TO_MS2    = 9.80665;

// ---------------------------------------------------------------------------
// Initialisation I2C
// ---------------------------------------------------------------------------
let bus = null;
let useRealIMU = false;
let lastCalibSent = -1;

function sleepMs(ms) {
  const end = Date.now() + ms;
  while (Date.now() < end) { /* busy wait */ }
}

function busInit() {
  try {
    fs.accessSync('/dev/i2c-1', fs.constants.R_OK | fs.constants.W_OK);
  } catch (e) {
    parentPort.postMessage({ type: 'status', msg: `Pas d'accès R/W à /dev/i2c-1 (${e.message}) — mode simulation` });
    return;
  }

  try {
    const i2c = require('i2c-bus');
    bus = i2c.openSync(1);

    const who = bus.readByteSync(BNO055_ADDR, REG_CHIP_ID);
    if (who !== CHIP_ID) {
      throw new Error(`CHIP_ID=0x${who.toString(16)} (attendu 0x${CHIP_ID.toString(16)})`);
    }

    // CONFIG → NORMAL → NDOF. Délais conformes datasheet section 3.6.
    bus.writeByteSync(BNO055_ADDR, REG_OPR_MODE, MODE_CONFIG);
    sleepMs(25);
    bus.writeByteSync(BNO055_ADDR, REG_PWR_MODE, PWR_NORMAL);
    sleepMs(10);
    bus.writeByteSync(BNO055_ADDR, REG_SYS_TRIG, 0x00); // pas d'oscillateur externe
    sleepMs(10);
    bus.writeByteSync(BNO055_ADDR, REG_UNIT_SEL, UNIT_DEFAULT);
    sleepMs(10);
    bus.writeByteSync(BNO055_ADDR, REG_OPR_MODE, MODE_NDOF);
    sleepMs(25); // datasheet : 7-19 ms après changement de mode

    useRealIMU = true;
    parentPort.postMessage({ type: 'status', msg: 'BNO055 initialisé en mode NDOF — calibration auto en cours (bouge le bateau pour finaliser sys/mag)' });
  } catch (e) {
    useRealIMU = false;
    parentPort.postMessage({ type: 'status', msg: `Erreur init BNO055 (${e.message}) — mode simulation` });
  }
}

// ---------------------------------------------------------------------------
// Lecture BNO055 — un seul transfert I2C de 32 octets
// ---------------------------------------------------------------------------
function readAll() {
  const buf = Buffer.alloc(32);
  bus.readI2cBlockSync(BNO055_ADDR, REG_ACCEL_DATA, 32, buf);

  // 0x08-0x0D : accel (m/s²)
  const ax = buf.readInt16LE(0)  * ACCEL_SCALE;
  const ay = buf.readInt16LE(2)  * ACCEL_SCALE;
  const az = buf.readInt16LE(4)  * ACCEL_SCALE;

  // 0x0E-0x13 : magnéto (µT)
  const mx = buf.readInt16LE(6)  * MAG_SCALE;
  const my = buf.readInt16LE(8)  * MAG_SCALE;
  const mz = buf.readInt16LE(10) * MAG_SCALE;

  // 0x14-0x19 : gyro (dps → rad/s)
  const gx = buf.readInt16LE(12) * GYRO_SCALE * DEG2RAD;
  const gy = buf.readInt16LE(14) * GYRO_SCALE * DEG2RAD;
  const gz = buf.readInt16LE(16) * GYRO_SCALE * DEG2RAD;

  // 0x1A-0x1F : Euler (degrés) — heading, roll, pitch
  const eHeading = buf.readInt16LE(18) * EULER_SCALE;
  const eRoll    = buf.readInt16LE(20) * EULER_SCALE;
  const ePitch   = buf.readInt16LE(22) * EULER_SCALE;

  // 0x20-0x27 : quaternion Q14 (W, X, Y, Z)
  const qw = buf.readInt16LE(24) * QUAT_SCALE;
  const qx = buf.readInt16LE(26) * QUAT_SCALE;
  const qy = buf.readInt16LE(28) * QUAT_SCALE;
  const qz = buf.readInt16LE(30) * QUAT_SCALE;

  return {
    accel:   [ax / G_TO_MS2, ay / G_TO_MS2, az / G_TO_MS2], // converti en g pour rester compatible imu.accel
    gyro:    [gx, gy, gz],         // rad/s (boatimu.cljs convertit en °/s pour imu.gyro)
    compass: [mx, my, mz],         // µT
    euler:   [eHeading, eRoll, ePitch],
    quat:    [qw, qx, qy, qz],
  };
}

// CALIB_STAT : bits 7-6 sys, 5-4 gyr, 3-2 acc, 1-0 mag (datasheet section 3.10)
function readCalib() {
  const stat = bus.readByteSync(BNO055_ADDR, REG_CALIB_STAT);
  return {
    sys: (stat >> 6) & 0x03,
    gyr: (stat >> 4) & 0x03,
    acc: (stat >> 2) & 0x03,
    mag: stat & 0x03,
    raw: stat,
  };
}

// ---------------------------------------------------------------------------
// Simulation (cap qui dérive doucement)
// ---------------------------------------------------------------------------
let simHeading = 180;
let simRate    = 0;

function simulateStep(dt) {
  simRate += (Math.random() - 0.5) * 2 - simRate * 0.05;
  simRate = Math.max(-8, Math.min(8, simRate));
  simHeading = (simHeading + simRate * dt + 360) % 360;

  // Quaternion représentant une rotation pure autour de Z d'un angle = -heading
  // (convention pypilot : heading positif = rotation horaire vue du dessus).
  const hr   = simHeading * DEG2RAD;
  const half = -hr / 2;
  const qw = Math.cos(half);
  const qz = Math.sin(half);

  return {
    accel:   [0, 0, 1],
    gyro:    [0, 0, simRate * DEG2RAD],
    compass: [Math.cos(hr), Math.sin(hr), 0],
    euler:   [simHeading, 0, 0],
    quat:    [qw, 0, 0, qz],
  };
}

// ---------------------------------------------------------------------------
// Boucle principale (20 Hz)
// ---------------------------------------------------------------------------
const TARGET_MS = 50;

function step() {
  const now = Date.now();
  let frame;

  if (useRealIMU) {
    try {
      frame = readAll();
      // Statut de calibration : on envoie un message status seulement quand il change
      const c = readCalib();
      if (c.raw !== lastCalibSent) {
        lastCalibSent = c.raw;
        parentPort.postMessage({
          type: 'status',
          msg:  `BNO055 calib sys=${c.sys}/3 gyr=${c.gyr}/3 acc=${c.acc}/3 mag=${c.mag}/3`,
        });
      }
      parentPort.postMessage({
        type:        'data',
        fusionQPose: frame.quat,
        accel:       frame.accel,
        gyro:        frame.gyro,
        compass:     frame.compass,
        euler:       frame.euler,
        calibration: { sys: c.sys, gyr: c.gyr, acc: c.acc, mag: c.mag },
        timestamp:   now,
      });
    } catch (e) {
      parentPort.postMessage({ type: 'status', msg: `Erreur lecture BNO055: ${e.message}` });
      useRealIMU = false;
    }
  }

  if (!useRealIMU) {
    frame = simulateStep(TARGET_MS / 1000.0);
    parentPort.postMessage({
      type:        'data',
      fusionQPose: frame.quat,
      accel:       frame.accel,
      gyro:        frame.gyro,
      compass:     frame.compass,
      euler:       frame.euler,
      calibration: { sys: 0, gyr: 0, acc: 0, mag: 0 },
      timestamp:   now,
    });
  }

  const elapsed = Date.now() - now;
  const sleep   = TARGET_MS - elapsed;
  if (sleep > 1) setTimeout(step, sleep);
  else           setImmediate(step);
}

busInit();
step();

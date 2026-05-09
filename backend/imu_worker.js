'use strict';
// IMU Worker — lit l'ICM-20948 via I2C (synchrone) et envoie les données
// fusionnées (quaternion Mahony) au thread principal via postMessage.
//
// Si I2C n'est pas disponible, bascule automatiquement en simulation.

const { parentPort } = require('worker_threads');

// ---------------------------------------------------------------------------
// Constantes ICM-20948 / AK09916
// ---------------------------------------------------------------------------
const ICM_ADDR     = 0x68;
const AK09916_ADDR = 0x0C;
const WHO_AM_I_VAL = 0xEA;

// Sélecteur de banque (présent dans toutes les banques)
const REG_BANK_SEL = 0x7F;

// Banque 0
const B0_WHO_AM_I    = 0x00;
const B0_USER_CTRL   = 0x03;
const B0_PWR_MGMT_1  = 0x06;
const B0_PWR_MGMT_2  = 0x07;
const B0_INT_PIN_CFG = 0x0F;
const B0_ACCEL_XOUT_H = 0x2D; // 12 octets : accel(6) + gyro(6)

// Banque 2
const B2_GYRO_CONFIG_1   = 0x01;
const B2_ACCEL_CONFIG    = 0x14;

// Échelles par défaut
const ACCEL_SCALE = 16384.0;            // ±2g → 1 g = 16384 LSB
const GYRO_SCALE  = 131.0;              // ±250 °/s → 1 °/s = 131 LSB
const DEG2RAD     = Math.PI / 180.0;
const MAG_SCALE   = 0.15;               // µT par LSB (AK09916, 16-bit signé)

// ---------------------------------------------------------------------------
// État filtre Mahony
// ---------------------------------------------------------------------------
let q0 = 1, q1 = 0, q2 = 0, q3 = 0;
let ix = 0, iy = 0, iz = 0;       // intégrales Mahony
const Kp = 10.0;
const Ki = 0.0;

function mahony(ax, ay, az, gx, gy, gz, mx, my, mz, dt) {
  // Normaliser accéléromètre
  let norm = Math.sqrt(ax*ax + ay*ay + az*az);
  if (norm === 0) return;
  ax /= norm; ay /= norm; az /= norm;

  // Direction de la gravité estimée (depuis le quaternion courant)
  const vx = 2*(q1*q3 - q0*q2);
  const vy = 2*(q0*q1 + q2*q3);
  const vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Erreur = produit vectoriel entre accel mesuré et estimé
  let ex = ay*vz - az*vy;
  let ey = az*vx - ax*vz;
  let ez = ax*vy - ay*vx;

  // Contribution magnétomètre (si disponible)
  if (mx !== null && mx !== undefined) {
    norm = Math.sqrt(mx*mx + my*my + mz*mz);
    if (norm > 0) {
      mx /= norm; my /= norm; mz /= norm;
      // Référence champ magnétique dans le repère capteur
      const hx = 2*(mx*(0.5 - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
      const hy = 2*(mx*(q1*q2 + q0*q3) + my*(0.5 - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
      const bx = Math.sqrt(hx*hx + hy*hy);
      const bz = 2*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5 - q1*q1 - q2*q2));
      // Direction magnétique estimée
      const wx = 2*(bx*(0.5 - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
      const wy = 2*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3));
      const wz = 2*(bx*(q1*q3 + q0*q2) + bz*(0.5 - q1*q1 - q2*q2));
      ex += my*wz - mz*wy;
      ey += mz*wx - mx*wz;
      ez += mx*wy - my*wx;
    }
  }

  // Intégrale
  ix += Ki * ex * dt;
  iy += Ki * ey * dt;
  iz += Ki * ez * dt;

  // Gyro corrigé
  gx += Kp * ex + ix;
  gy += Kp * ey + iy;
  gz += Kp * ez + iz;

  // Intégration quaternion (1er ordre)
  q0 += 0.5 * (-q1*gx - q2*gy - q3*gz) * dt;
  q1 += 0.5 * ( q0*gx + q2*gz - q3*gy) * dt;
  q2 += 0.5 * ( q0*gy - q1*gz + q3*gx) * dt;
  q3 += 0.5 * ( q0*gz + q1*gy - q2*gx) * dt;

  norm = Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm === 0) { q0 = 1; q1 = q2 = q3 = 0; return; }
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

// ---------------------------------------------------------------------------
// Lecture ICM-20948
// ---------------------------------------------------------------------------
let bus = null;
let useRealIMU = false;
let magReady = false;

// Biais de calibration appliqués à toutes les lectures (réel - biais).
// Mesurés au démarrage pendant 2 s à l'arrêt — voir calibrateAtRest().
let biasAx = 0, biasAy = 0, biasAz = 0;
let biasGx = 0, biasGy = 0, biasGz = 0;

function sleepMs(ms) {
  const end = Date.now() + ms;
  while (Date.now() < end) { /* busy wait */ }
}

function selectBank(b) {
  bus.writeByteSync(ICM_ADDR, REG_BANK_SEL, (b & 0x03) << 4);
}

function busInit() {
  const fs = require('fs');

  // Vérifier les droits d'accès AVANT de charger l'addon natif i2c-bus.
  // Sur une machine sans RPi, le module natif peut segfaulter lors de son
  // initialisation même si on attrape l'erreur JS.
  try {
    fs.accessSync('/dev/i2c-1', fs.constants.R_OK | fs.constants.W_OK);
  } catch (e) {
    parentPort.postMessage({ type: 'status', msg: `Pas d'accès R/W à /dev/i2c-1 (${e.message}) — mode simulation` });
    return;
  }

  try {
    const i2c = require('i2c-bus');
    bus = i2c.openSync(1);

    // Vérifier l'identité du capteur
    selectBank(0);
    const who = bus.readByteSync(ICM_ADDR, B0_WHO_AM_I);
    if (who !== WHO_AM_I_VAL) {
      throw new Error(`WHO_AM_I=0x${who.toString(16)} (attendu 0x${WHO_AM_I_VAL.toString(16)})`);
    }

    // Reset puis réveil
    bus.writeByteSync(ICM_ADDR, B0_PWR_MGMT_1, 0x80); // device reset
    sleepMs(100);
    selectBank(0);
    bus.writeByteSync(ICM_ADDR, B0_PWR_MGMT_1, 0x01); // auto clock, sleep off
    sleepMs(20);
    bus.writeByteSync(ICM_ADDR, B0_PWR_MGMT_2, 0x00); // accel + gyro on

    // Configuration accel + gyro (banque 2)
    selectBank(2);
    // GYRO_CONFIG_1 : DLPFCFG=0 (197 Hz), FS_SEL=00 (±250 dps), FCHOICE=1 (DLPF on)
    bus.writeByteSync(ICM_ADDR, B2_GYRO_CONFIG_1, 0x01);
    // ACCEL_CONFIG : DLPFCFG=0 (246 Hz), FS_SEL=00 (±2g), FCHOICE=1 (DLPF on)
    bus.writeByteSync(ICM_ADDR, B2_ACCEL_CONFIG, 0x01);

    // Bypass I2C pour parler directement à l'AK09916 (banque 0)
    selectBank(0);
    bus.writeByteSync(ICM_ADDR, B0_USER_CTRL,   0x00); // I2C master désactivé
    bus.writeByteSync(ICM_ADDR, B0_INT_PIN_CFG, 0x02); // BYPASS_EN

    // Init AK09916 : reset + mode continu 4 (100 Hz)
    try {
      bus.writeByteSync(AK09916_ADDR, 0x32, 0x01); // CNTL3 soft reset
      sleepMs(10);
      bus.writeByteSync(AK09916_ADDR, 0x31, 0x08); // CNTL2 = continuous mode 4
      sleepMs(10);
      magReady = true;
    } catch (e) {
      parentPort.postMessage({ type: 'status', msg: `AK09916 indisponible (${e.message}) — sans magnétomètre` });
      magReady = false;
    }

    useRealIMU = true;
    parentPort.postMessage({ type: 'status', msg: `ICM-20948 initialisé sur I2C bus 1 (mag ${magReady ? 'OK' : 'KO'})` });
  } catch (e) {
    useRealIMU = false;
    parentPort.postMessage({ type: 'status', msg: `Erreur init I2C (${e.message}) — mode simulation` });
  }
}

function readIMUSync() {
  const buf = Buffer.alloc(12);
  bus.readI2cBlockSync(ICM_ADDR, B0_ACCEL_XOUT_H, 12, buf);
  return {
    ax: buf.readInt16BE(0)  / ACCEL_SCALE - biasAx,
    ay: buf.readInt16BE(2)  / ACCEL_SCALE - biasAy,
    az: buf.readInt16BE(4)  / ACCEL_SCALE - biasAz,
    gx: buf.readInt16BE(6)  / GYRO_SCALE * DEG2RAD - biasGx,
    gy: buf.readInt16BE(8)  / GYRO_SCALE * DEG2RAD - biasGy,
    gz: buf.readInt16BE(10) / GYRO_SCALE * DEG2RAD - biasGz,
  };
}

// Calibration "à l'arrêt" : moyenne 2 s de lectures, en déduire les biais.
// - Gyro : la moyenne au repos *est* le biais (devrait lire 0 dps).
// - Accel : on force la norme du vecteur mesuré à 1 g dans la direction
//   actuelle. Workaround pour module au MEMS Y défaillant — l'angle d'attitude
//   reste utilisable tant que le bateau n'est pas trop incliné par rapport à
//   l'orientation de calibration.
function calibrateAtRest(durationMs = 2000) {
  if (!useRealIMU) return;
  parentPort.postMessage({ type: 'status', msg: `Calibration au repos (${durationMs / 1000}s) — ne bouge pas le bateau` });

  const intervalMs = 20; // 50 Hz
  const N = Math.floor(durationMs / intervalMs);
  let sAx = 0, sAy = 0, sAz = 0, sGx = 0, sGy = 0, sGz = 0;
  let count = 0;

  for (let i = 0; i < N; i++) {
    try {
      const d = readIMUSync(); // biais encore à 0, donc retourne brut
      sAx += d.ax; sAy += d.ay; sAz += d.az;
      sGx += d.gx; sGy += d.gy; sGz += d.gz;
      count++;
    } catch (e) { /* skip */ }
    sleepMs(intervalMs);
  }

  if (count < N * 0.8) {
    parentPort.postMessage({ type: 'status', msg: `Calibration incomplète (${count}/${N}) — biais ignorés` });
    return;
  }

  const mAx = sAx / count, mAy = sAy / count, mAz = sAz / count;
  biasGx = sGx / count;
  biasGy = sGy / count;
  biasGz = sGz / count;

  const norm = Math.sqrt(mAx * mAx + mAy * mAy + mAz * mAz);
  if (norm < 0.1) {
    parentPort.postMessage({ type: 'status', msg: `Calibration: |a|=${norm.toFixed(3)}g, capteur muet — biais accel ignorés` });
    return;
  }

  // bias = mesure - direction_unitaire → après soustraction on obtient un
  // vecteur de norme 1 g pointant dans la direction mesurée.
  biasAx = mAx - mAx / norm;
  biasAy = mAy - mAy / norm;
  biasAz = mAz - mAz / norm;

  // Détection saturation : si un axe est à ±2 g pendant la calib, le MEMS
  // sature et la compensation sera fausse en navigation.
  const sat = Math.max(Math.abs(mAx), Math.abs(mAy), Math.abs(mAz)) >= 1.95;

  parentPort.postMessage({
    type: 'status',
    msg: `Calibration OK: |a|brut=${norm.toFixed(3)}g, biais accel=(${biasAx.toFixed(3)}, ${biasAy.toFixed(3)}, ${biasAz.toFixed(3)})g, biais gyro=(${biasGx.toFixed(3)}, ${biasGy.toFixed(3)}, ${biasGz.toFixed(3)})rad/s${sat ? ' — ATTENTION axe saturé, compensation peu fiable' : ''}`,
  });
}

function readCompassSync() {
  if (!magReady) return null;
  // ST1 : bit 0 = DRDY
  const st1 = bus.readByteSync(AK09916_ADDR, 0x10);
  if ((st1 & 0x01) === 0) return null;
  // 8 octets depuis HXL : HX(L,H), HY(L,H), HZ(L,H), TMPS, ST2.
  // Lire jusqu'à ST2 inclus est nécessaire pour libérer la mesure.
  const buf = Buffer.alloc(8);
  bus.readI2cBlockSync(AK09916_ADDR, 0x11, 8, buf);
  if (buf[7] & 0x08) return null; // HOFL : overflow
  return {
    mx: buf.readInt16LE(0) * MAG_SCALE,
    my: buf.readInt16LE(2) * MAG_SCALE,
    mz: buf.readInt16LE(4) * MAG_SCALE,
  };
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
let simHeading = 180;  // cap de départ
let simRate = 0;       // taux de rotation simulé (°/s)

function simulateStep(dt) {
  simRate += (Math.random() - 0.5) * 2 - simRate * 0.05;
  simRate = Math.max(-8, Math.min(8, simRate));
  simHeading = (simHeading + simRate * dt + 360) % 360;

  const hr = simHeading * DEG2RAD;
  const ax = 0, ay = 0, az = 1;
  const gx = 0, gy = 0, gz = simRate * DEG2RAD;
  const mx = Math.cos(hr), my = Math.sin(hr), mz = 0;
  return { ax, ay, az, gx, gy, gz, mx, my, mz };
}

// ---------------------------------------------------------------------------
// Boucle principale du worker (20 Hz)
// ---------------------------------------------------------------------------
const TARGET_MS = 50;
let lastTime = Date.now();

function step() {
  const now = Date.now();
  const dt  = Math.min((now - lastTime) / 1000.0, 0.1);
  lastTime  = now;

  let accel, gyro, compass, mx, my, mz;

  if (useRealIMU) {
    try {
      const d = readIMUSync();
      const c = readCompassSync();
      accel   = [d.ax, d.ay, d.az];
      gyro    = [d.gx, d.gy, d.gz];
      compass = c ? [c.mx, c.my, c.mz] : null;
      mx = c ? c.mx : null;
      my = c ? c.my : null;
      mz = c ? c.mz : null;
      mahony(d.ax, d.ay, d.az, d.gx, d.gy, d.gz, mx, my, mz, dt);
    } catch (e) {
      parentPort.postMessage({ type: 'status', msg: `Erreur lecture IMU: ${e.message}` });
      useRealIMU = false;
    }
  }

  if (!useRealIMU) {
    const s = simulateStep(dt);
    accel   = [s.ax, s.ay, s.az];
    gyro    = [s.gx, s.gy, s.gz];
    compass = [s.mx, s.my, s.mz];
    mahony(s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.mx, s.my, s.mz, dt);
  }

  parentPort.postMessage({
    type:        'data',
    fusionQPose: [q0, q1, q2, q3],
    accel,
    gyro,
    compass,
    timestamp:   now,
  });

  const elapsed = Date.now() - now;
  const sleep   = TARGET_MS - elapsed;
  if (sleep > 1) setTimeout(step, sleep);
  else           setImmediate(step);
}

busInit();
calibrateAtRest();
lastTime = Date.now();
step();

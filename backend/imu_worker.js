'use strict';
// IMU Worker — lit le MPU-9250 via I2C (synchrone) et envoie les données
// fusionnées (quaternion Mahony) au thread principal via postMessage.
//
// Si I2C n'est pas disponible, bascule automatiquement en simulation.

const { parentPort } = require('worker_threads');

// ---------------------------------------------------------------------------
// Constantes MPU-9250 / AK8963
// ---------------------------------------------------------------------------
const MPU_ADDR     = 0x68;
const AK8963_ADDR  = 0x0C;

// Registres MPU-9250
const REG_SMPLRT_DIV   = 0x19;
const REG_CONFIG        = 0x1A;
const REG_GYRO_CONFIG   = 0x1B;
const REG_ACCEL_CONFIG  = 0x1C;
const REG_INT_PIN_CFG   = 0x37;
const REG_USER_CTRL     = 0x6A;
const REG_PWR_MGMT_1    = 0x6B;
const REG_ACCEL_XOUT_H  = 0x3B; // 14 bytes : accel(6) + temp(2) + gyro(6)

// Échelles par défaut
const ACCEL_SCALE = 16384.0;             // ±2g → 1 g = 16384 LSB
const GYRO_SCALE  = 131.0;              // ±250 °/s → 1 °/s = 131 LSB (deg)
const DEG2RAD     = Math.PI / 180.0;
const MAG_SCALE   = 4912.0 / 32760.0;  // µT par LSB (16-bit AK8963)

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
// Lecture MPU-9250
// ---------------------------------------------------------------------------
let bus = null;
let useRealIMU = false;
let magReady = false;

function busInit() {
  const fs = require('fs');

  // Vérifier les droits d'accès AVANT de charger l'addon natif i2c-bus.
  // Sur une machine sans RPi, le module natif peut segfaulter lors de son
  // initialisation même si on attrape l'erreur JS.
  try {
    fs.accessSync('/dev/i2c-1', fs.constants.R_OK | fs.constants.W_OK);
  } catch (e) {
    parentPort.postMessage({ type: 'status', msg: `Pas d'accès R/W à /dev/i2c-1 (${e.message}) — mode simulation` });
    return; // ne charge pas i2c-bus
  }

  try {
    const i2c = require('i2c-bus');
    bus = i2c.openSync(1);

    // Réveil MPU-9250
    bus.writeByteSync(MPU_ADDR, REG_PWR_MGMT_1,   0x00); // wake up
    bus.writeByteSync(MPU_ADDR, REG_ACCEL_CONFIG,  0x00); // ±2g
    bus.writeByteSync(MPU_ADDR, REG_GYRO_CONFIG,   0x00); // ±250 °/s
    bus.writeByteSync(MPU_ADDR, REG_SMPLRT_DIV,    0x04); // ~200 Hz ODR
    bus.writeByteSync(MPU_ADDR, REG_CONFIG,         0x03); // DLPF 41 Hz

    // Activer le bypass I2C pour accéder à l'AK8963 directement
    bus.writeByteSync(MPU_ADDR, REG_USER_CTRL,     0x00); // disable I2C master
    bus.writeByteSync(MPU_ADDR, REG_INT_PIN_CFG,   0x02); // bypass enable

    // Init AK8963
    bus.writeByteSync(AK8963_ADDR, 0x0A, 0x00);           // power down
    const wait = Date.now() + 15;
    while (Date.now() < wait) { /* busy wait 15ms */ }
    bus.writeByteSync(AK8963_ADDR, 0x0A, 0x16);           // 16-bit, continu mode 2 (100 Hz)
    magReady = true;

    useRealIMU = true;
    parentPort.postMessage({ type: 'status', msg: 'MPU-9250 initialisé sur I2C bus 1' });
  } catch (e) {
    useRealIMU = false;
    parentPort.postMessage({ type: 'status', msg: `Erreur init I2C (${e.message}) — mode simulation` });
  }
}

function readIMUSync() {
  const buf = Buffer.alloc(14);
  bus.readI2cBlockSync(MPU_ADDR, REG_ACCEL_XOUT_H, 14, buf);
  return {
    ax:  buf.readInt16BE(0)  / ACCEL_SCALE,
    ay:  buf.readInt16BE(2)  / ACCEL_SCALE,
    az:  buf.readInt16BE(4)  / ACCEL_SCALE,
    gx:  buf.readInt16BE(8)  / GYRO_SCALE * DEG2RAD,
    gy:  buf.readInt16BE(10) / GYRO_SCALE * DEG2RAD,
    gz:  buf.readInt16BE(12) / GYRO_SCALE * DEG2RAD,
  };
}

function readCompassSync() {
  if (!magReady) return null;
  const buf = Buffer.alloc(7);
  bus.readI2cBlockSync(AK8963_ADDR, 0x03, 7, buf);
  if (buf[6] & 0x08) return null; // overflow (HOFL bit)
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
lastTime = Date.now();
step();

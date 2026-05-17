'use strict';
// Validation rapide du BNO055 après vérif du câblage.
//
// Usage sur le Pi :
//   sudo systemctl stop helmpilot.service        # libère /dev/i2c-1
//   node backend/imu_check.js
//   sudo systemctl start helmpilot.service
//
// Pose le module à plat sur la table avant de lancer.

const i2c = require('i2c-bus');

const ADDR           = 0x29;  // BNO055, ADR=HIGH (Adafruit/CJMCU par défaut)
const REG_CHIP_ID    = 0x00;
const REG_ACCEL_DATA = 0x08;  // accel X/Y/Z LSB,MSB (m/s² × 100)
const REG_GYRO_DATA  = 0x14;  // gyro X/Y/Z LSB,MSB (dps × 16)
const REG_TEMP       = 0x34;  // °C signé 8 bits
const REG_CALIB_STAT = 0x35;
const REG_UNIT_SEL   = 0x3B;
const REG_OPR_MODE   = 0x3D;
const REG_PWR_MODE   = 0x3E;
const REG_SYS_TRIG   = 0x3F;
const REG_PAGE_ID    = 0x07;

const CHIP_ID        = 0xA0;
const MODE_CONFIG    = 0x00;
const MODE_NDOF      = 0x0C;
const PWR_NORMAL     = 0x00;
const UNIT_DEFAULT   = 0x00;  // accel m/s², gyro dps, temp °C

const ACCEL_SCALE = 1 / 100.0; // 1 m/s² = 100 LSB
const GYRO_SCALE  = 1 / 16.0;  // 1 dps  = 16 LSB
const G_TO_MS2    = 9.80665;

const GREEN = '\x1b[32m', RED = '\x1b[31m', YEL = '\x1b[33m', RST = '\x1b[0m';
const ok   = (m) => console.log(`${GREEN}✓${RST} ${m}`);
const fail = (m) => console.log(`${RED}✗${RST} ${m}`);
const warn = (m) => console.log(`${YEL}!${RST} ${m}`);

function sleepMs(ms) { const end = Date.now() + ms; while (Date.now() < end) {} }

function safeWrite(bus, reg, val, retries = 5) {
  for (let i = 0; i < retries; i++) {
    try { bus.writeByteSync(ADDR, reg, val); return true; }
    catch (e) { sleepMs(20); }
  }
  return false;
}

function std(vals, m) {
  const s = vals.reduce((acc, v) => acc + (v - m) * (v - m), 0) / vals.length;
  return Math.sqrt(s);
}

function main() {
  let bus;
  try { bus = i2c.openSync(1); }
  catch (e) { fail(`Impossible d'ouvrir /dev/i2c-1 : ${e.message}`); process.exit(1); }

  // 1. Présence + identité
  let who;
  try { who = bus.readByteSync(ADDR, REG_CHIP_ID); }
  catch (e) { fail(`Pas de réponse à 0x${ADDR.toString(16)} : ${e.message} — vérifie ADR pin (HIGH=0x29) et câblage SDA/SCL`); process.exit(1); }
  if (who === CHIP_ID) ok(`CHIP_ID = 0x${who.toString(16)} (BNO055)`);
  else { fail(`CHIP_ID = 0x${who.toString(16)}, attendu 0x${CHIP_ID.toString(16)}`); process.exit(1); }

  // 2. Stress test lectures (6 octets = bloc accel)
  let nOk = 0, nErr = 0;
  const buf6 = Buffer.alloc(6);
  for (let i = 0; i < 200; i++) {
    try { bus.readI2cBlockSync(ADDR, REG_ACCEL_DATA, 6, buf6); nOk++; }
    catch (e) { nErr++; }
  }
  if (nErr === 0) ok('200 lectures I2C : 0 erreur');
  else fail(`200 lectures : ${nErr} erreur(s) → câblage suspect`);

  // 3. Stress test écritures sur PAGE_ID (registre inoffensif)
  nOk = 0; nErr = 0;
  for (let i = 0; i < 50; i++) {
    try { bus.writeByteSync(ADDR, REG_PAGE_ID, 0); nOk++; }
    catch (e) { nErr++; }
  }
  if (nErr === 0) ok('50 écritures I2C : 0 erreur');
  else fail(`50 écritures : ${nErr} erreur(s) → câblage suspect (SDA ou pull-up)`);

  // 4. Init : CONFIG → NORMAL → NDOF (datasheet section 3.6)
  if (!safeWrite(bus, REG_OPR_MODE, MODE_CONFIG)) { fail('Passage en MODE_CONFIG impossible'); process.exit(1); }
  sleepMs(25);
  safeWrite(bus, REG_PWR_MODE, PWR_NORMAL);
  sleepMs(10);
  safeWrite(bus, REG_SYS_TRIG, 0x00);              // pas d'oscillateur externe
  sleepMs(10);
  safeWrite(bus, REG_UNIT_SEL, UNIT_DEFAULT);
  sleepMs(10);
  safeWrite(bus, REG_OPR_MODE, MODE_NDOF);
  sleepMs(25);                                     // datasheet : 7-19 ms après changement de mode

  // 5. 100 échantillons à plat — l'accel BNO055 contient la gravité (mode NDOF, registres 0x08-0x0D)
  console.log('\nLecture 100 échantillons à plat (3 s)…');
  const samples = [];
  const buf12 = Buffer.alloc(12);
  for (let i = 0; i < 100; i++) {
    try {
      bus.readI2cBlockSync(ADDR, REG_ACCEL_DATA, 12, buf12);
      // BNO055 = little-endian (contrairement à l'ICM-20948 qui est big-endian)
      const ax_ms2 = buf12.readInt16LE(0)  * ACCEL_SCALE;
      const ay_ms2 = buf12.readInt16LE(2)  * ACCEL_SCALE;
      const az_ms2 = buf12.readInt16LE(4)  * ACCEL_SCALE;
      // Convertit en g pour rendre le verdict lisible
      samples.push({
        ax: ax_ms2 / G_TO_MS2,
        ay: ay_ms2 / G_TO_MS2,
        az: az_ms2 / G_TO_MS2,
        gx: buf12.readInt16LE(6)  * GYRO_SCALE,
        gy: buf12.readInt16LE(8)  * GYRO_SCALE,
        gz: buf12.readInt16LE(10) * GYRO_SCALE,
      });
    } catch (e) { /* ignore, on compte ce qu'on a */ }
    sleepMs(30);
  }

  if (samples.length < 80) { fail(`Seulement ${samples.length}/100 lectures réussies`); process.exit(1); }

  const n = samples.length;
  const avg = (k) => samples.reduce((a, s) => a + s[k], 0) / n;
  const ax_m = avg('ax'), ay_m = avg('ay'), az_m = avg('az');
  const gx_m = avg('gx'), gy_m = avg('gy'), gz_m = avg('gz');
  const norm = Math.sqrt(ax_m * ax_m + ay_m * ay_m + az_m * az_m);

  // Température : 1 °C par LSB, signé 8 bits
  let tempC = null;
  try {
    const tr = bus.readByteSync(ADDR, REG_TEMP);
    tempC = tr > 127 ? tr - 256 : tr;
  } catch (_) {}

  const ax_std = std(samples.map(s => s.ax), ax_m);
  const ay_std = std(samples.map(s => s.ay), ay_m);
  const az_std = std(samples.map(s => s.az), az_m);

  const f = (v) => (v >= 0 ? '+' : '') + v.toFixed(3);
  console.log(`\n  ax = ${f(ax_m)} g  (σ = ${ax_std.toFixed(4)})`);
  console.log(`  ay = ${f(ay_m)} g  (σ = ${ay_std.toFixed(4)})`);
  console.log(`  az = ${f(az_m)} g  (σ = ${az_std.toFixed(4)})`);
  console.log(`  |a| = ${norm.toFixed(3)} g  (attendu : 1.000 g)`);
  console.log(`  gyro repos : gx=${f(gx_m)}  gy=${f(gy_m)}  gz=${f(gz_m)} dps`);
  if (tempC !== null) console.log(`  temp = ${tempC} °C`);

  // BNO055 sature à ±4g en mode NDOF par défaut, donc la borne physique au repos est ±1g
  const saturated = samples.some(s => Math.abs(s.ax) > 3.99 || Math.abs(s.ay) > 3.99 || Math.abs(s.az) > 3.99);
  if (saturated) fail('Saturation détectée → MEMS donne >4g au repos = défaut hardware');

  // Verdict accel
  if (norm >= 0.95 && norm <= 1.05) ok(`|a| dans la plage attendue → accéléromètre OK`);
  else if (norm >= 0.85 && norm <= 1.15) warn(`|a| = ${norm.toFixed(3)} g, légèrement hors plage. Vérifie que le module est bien à plat.`);
  else fail(`|a| = ${norm.toFixed(3)} g — anormal. Le MEMS est probablement HS.`);

  const axes = { X: ax_m, Y: ay_m, Z: az_m };
  const bad = Object.entries(axes).filter(([_, v]) => Math.abs(v) > 1.2).map(([k]) => k);
  if (bad.length) fail(`Axe(s) hors borne physique (>1.2g au repos) : ${bad.join(', ')}`);

  if (Math.max(ax_std, ay_std, az_std) > 0.05) warn("Bruit élevé sur l'accéléromètre (σ>0.05g) — vérifier alim 3V3 propre");
  if (Math.max(Math.abs(gx_m), Math.abs(gy_m), Math.abs(gz_m)) > 5) warn('Biais gyro >5 dps — laisse 10-20 s immobile pour que la calib auto BNO055 stabilise');

  // 6. Statut calibration BNO055 (sys/gyr/acc/mag, 0-3)
  console.log('');
  try {
    const stat = bus.readByteSync(ADDR, REG_CALIB_STAT);
    const c = { sys: (stat >> 6) & 3, gyr: (stat >> 4) & 3, acc: (stat >> 2) & 3, mag: stat & 3 };
    const line = `sys=${c.sys}/3  gyr=${c.gyr}/3  acc=${c.acc}/3  mag=${c.mag}/3`;
    if (c.acc === 3) ok(`Calibration accel : ${line}`);
    else             warn(`Calibration en cours : ${line} — bouge le module dans plusieurs orientations`);
  } catch (e) { warn(`Lecture CALIB_STAT impossible : ${e.message}`); }

  bus.closeSync();
}

main();

'use strict';
// Validation rapide de l'ICM-20948 après vérif du câblage.
//
// Usage sur le Pi :
//   sudo systemctl stop imu-socket.service imu-fifo.service
//   node backend/imu_check.js
//   sudo systemctl start imu-fifo.service imu-socket.service
//
// Pose le module à plat sur la table avant de lancer.

const i2c = require('i2c-bus');

const ADDR = 0x68;
const REG_BANK_SEL = 0x7F;
const WHO_AM_I = 0x00;
const PWR_MGMT_1 = 0x06;
const PWR_MGMT_2 = 0x07;
const ACCEL_XOUT_H = 0x2D;
const ACCEL_CONFIG = 0x14;
const GYRO_CONFIG_1 = 0x01;

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

function selectBank(bus, b) { return safeWrite(bus, REG_BANK_SEL, (b & 0x03) << 4); }

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
  try { who = bus.readByteSync(ADDR, WHO_AM_I); }
  catch (e) { fail(`Pas de réponse à 0x${ADDR.toString(16)} : ${e.message}`); process.exit(1); }
  if (who === 0xEA) ok(`WHO_AM_I = 0x${who.toString(16)} (ICM-20948)`);
  else { fail(`WHO_AM_I = 0x${who.toString(16)}, attendu 0xea`); process.exit(1); }

  // 2. Stress test lectures
  let nOk = 0, nErr = 0;
  const buf6 = Buffer.alloc(6);
  for (let i = 0; i < 200; i++) {
    try { bus.readI2cBlockSync(ADDR, ACCEL_XOUT_H, 6, buf6); nOk++; }
    catch (e) { nErr++; }
  }
  if (nErr === 0) ok('200 lectures I2C : 0 erreur');
  else fail(`200 lectures : ${nErr} erreur(s) → câblage suspect`);

  // 3. Stress test écritures (souvent là que ça casse)
  nOk = 0; nErr = 0;
  for (let i = 0; i < 50; i++) {
    try { bus.writeByteSync(ADDR, REG_BANK_SEL, 0); nOk++; }
    catch (e) { nErr++; }
  }
  if (nErr === 0) ok('50 écritures I2C : 0 erreur');
  else fail(`50 écritures : ${nErr} erreur(s) → câblage suspect (SDA ou pull-up)`);

  // 4. Reset + wake + ±2g (sensibilité max, écrête à 2g)
  if (!safeWrite(bus, PWR_MGMT_1, 0x80)) { fail('Reset PWR_MGMT_1 impossible'); process.exit(1); }
  sleepMs(100);
  selectBank(bus, 0);
  safeWrite(bus, PWR_MGMT_1, 0x01);
  safeWrite(bus, PWR_MGMT_2, 0x00);
  sleepMs(50);
  selectBank(bus, 2);
  safeWrite(bus, ACCEL_CONFIG, 0x00);  // ±2g, DLPF off
  safeWrite(bus, GYRO_CONFIG_1, 0x00); // ±250 dps, DLPF off
  selectBank(bus, 0);
  sleepMs(100);

  // 5. 100 échantillons à plat
  console.log('\nLecture 100 échantillons à plat (3 s)…');
  const samples = [];
  const buf14 = Buffer.alloc(14);
  for (let i = 0; i < 100; i++) {
    try {
      bus.readI2cBlockSync(ADDR, ACCEL_XOUT_H, 14, buf14);
      samples.push({
        ax: buf14.readInt16BE(0)  / 16384.0,
        ay: buf14.readInt16BE(2)  / 16384.0,
        az: buf14.readInt16BE(4)  / 16384.0,
        gx: buf14.readInt16BE(6)  / 131.0,
        gy: buf14.readInt16BE(8)  / 131.0,
        gz: buf14.readInt16BE(10) / 131.0,
        tr: buf14.readInt16BE(12),
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
  const temp = avg('tr') / 333.87 + 21.0;

  const ax_std = std(samples.map(s => s.ax), ax_m);
  const ay_std = std(samples.map(s => s.ay), ay_m);
  const az_std = std(samples.map(s => s.az), az_m);

  const f = (v) => (v >= 0 ? '+' : '') + v.toFixed(3);
  console.log(`\n  ax = ${f(ax_m)} g  (σ = ${ax_std.toFixed(4)})`);
  console.log(`  ay = ${f(ay_m)} g  (σ = ${ay_std.toFixed(4)})`);
  console.log(`  az = ${f(az_m)} g  (σ = ${az_std.toFixed(4)})`);
  console.log(`  |a| = ${norm.toFixed(3)} g  (attendu : 1.000 g)`);
  console.log(`  gyro repos : gx=${f(gx_m)}  gy=${f(gy_m)}  gz=${f(gz_m)} dps`);
  console.log(`  temp = ${temp.toFixed(1)} °C\n`);

  const saturated = samples.some(s => Math.abs(s.ax) > 1.99 || Math.abs(s.ay) > 1.99 || Math.abs(s.az) > 1.99);
  if (saturated) fail('Saturation détectée en ±2g → MEMS donne >2g au repos = défaut hardware');

  // Verdict
  if (norm >= 0.95 && norm <= 1.05) ok(`|a| dans la plage attendue → accéléromètre OK`);
  else if (norm >= 0.85 && norm <= 1.15) warn(`|a| = ${norm.toFixed(3)} g, légèrement hors plage. Vérifie que le module est bien à plat.`);
  else fail(`|a| = ${norm.toFixed(3)} g — anormal. Le MEMS est probablement HS.`);

  const axes = { X: ax_m, Y: ay_m, Z: az_m };
  const bad = Object.entries(axes).filter(([_, v]) => Math.abs(v) > 1.2).map(([k]) => k);
  if (bad.length) fail(`Axe(s) hors borne physique (>1.2g au repos) : ${bad.join(', ')}`);

  if (Math.max(ax_std, ay_std, az_std) > 0.05) warn("Bruit élevé sur l'accéléromètre (σ>0.05g) — vérifier alim 3V3 propre");
  if (Math.max(Math.abs(gx_m), Math.abs(gy_m), Math.abs(gz_m)) > 5) warn('Biais gyro >5 dps — calibration zéro à refaire (pas un défaut HW)');

  bus.closeSync();
}

main();

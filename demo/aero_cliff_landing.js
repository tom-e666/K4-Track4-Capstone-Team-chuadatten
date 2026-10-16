/* Cứu hộ khí động — lõi mô phỏng (SIM CORE) + ứng dụng 3D three.js. Cần ../viewer/vendor/three.min.js nạp trước. */
// ==SIM-CORE-BEGIN== (2D dọc: x tiến, y lên; dùng chung với bản test headless)
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
// Mọi thông số là GIẢ ĐỊNH để minh hoạ thuật toán, không phải số liệu xe thật (kể cả xe SUV mẫu).
const P = {
  g: 9.81, rho: 1.225, Ik: 1.45, wheelR: 0.76,
  // khí động: thân xe + cánh gió sau dựng đứng làm phanh khí
  bodyCdA: 0.75, bodySideCdA: 3.6, wingCdA: 2.6, wingRate: 2.5,
  // 4 jet gầm (thổi xuống) tổng thrTW × trọng lượng; 4 jet nóc (thổi lên) tổng thrTopTW × trọng lượng
  thrTW: 2.5, thrTopTW: 1.0, thrArm: 1.4, thrSpool: 6, gasDv: 110,
  // 4 jet hông (2 trái, 2 phải) gắn ngang mép nóc, thổi ra ngoài → mô-men LẬT NGANG (roll); mỗi bên tối đa sideTW × trọng lượng
  sideTW: 0.35, sideArm: 0.65, IkRoll: 0.6, rollTol: 15,
  // vòng tư thế: góc → tốc độ xoay mong muốn (bão hoà wMax) → mô-men
  kTh: 3, kW: 6, wMax: 3, rightTol: 15, rightW: 0.5,
  // IMU: gia tốc riêng < ffG (g) liên tục ffDetect (s) → xác nhận rơi tự do
  ffDetect: 0.25, ffG: 0.3,
  // dẫn đường suicide burn
  ignMargin: 0.8, lat: 0.12, hTd: 0.35, vTd: 1.0, levelH: 45,
  sensFov: 50,   // bộ cảm biến ở cả gầm lẫn nóc; chỉ đo được khi một mặt hướng xuống trong ±sensFov°
  sens: {
    lidar: { max: 200, sd: 0.05, hz: 20 },
    radar: { max: 250, sd: 0.5, sdV: 0.15, hz: 20 },
    cam: { max: 120, sdRel: 0.05, hz: 30 },
  },
  vnSafe: 3.5, vnHard: 7, peSafe: 8, peHard: 15,
  // người ngồi trong xe: đầu cách trọng tâm ~0,7 m; quãng giảm tải khi chạm đất = giảm xóc + đệm ghế (bánh chạm trước)
  // hoặc vùng biến dạng thân xe (thân/nóc chạm trước). Ngưỡng nguy cơ chỉ là mốc tham khảo gần đúng, KHÔNG phải tiêu chuẩn y khoa.
  occ: { rHead: 0.7, strokeWheel: 0.3, strokeBody: 0.5, bands: [[3, 'rất thấp'], [10, 'thấp'], [20, 'trung bình'], [50, 'cao']] },
};
const cfg = { H: 180, v0: 28, wind: 0, slope: 15, mass: 2500, spin: 220, roll: 150 };   // SUV điện cỡ trung ~2,5 t (giả định)

const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const lerp = (a, b, t) => a + (b - a) * t;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const smooth = (t) => t * t * (3 - 2 * t);
const ms = () => cfg.mass / 1800;

// Địa hình: đỉnh đèo (x<0) cao H; chân vực là sườn dốc rồi thung lũng phẳng.
const hillTop = () => (cfg.slope > 0 ? Math.min(cfg.H * 0.4, 70) : 0);
const hillEnd = () => (cfg.slope > 0 ? hillTop() / Math.tan(cfg.slope * D2R) : 0);
function groundY(x) { return x < 0 ? cfg.H : Math.max(0, hillTop() - Math.tan(cfg.slope * D2R) * x); }
function slopeAt(x) { return x >= 0 && x < hillEnd() ? -cfg.slope * D2R : 0; }

// Hệ toạ độ thân xe (m), gốc tại trọng tâm. 0-1: điểm chạm bánh; còn lại: thân xe.
const CONTACT_PTS = [[1.45, -0.76], [-1.45, -0.76], [2.37, -0.35], [-2.37, -0.35], [0.4, 0.76], [-1.5, 0.74]];   // SUV dài ~4,75 m, cao ~1,67 m

function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function gauss(r) { const u = Math.max(1e-9, r()), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

function makeCar(ctrl) {
  return {
    ctrl, x: -70, y: cfg.H + P.wheelR, vx: cfg.v0, vy: 0, th: 0, w: 0, t: 0, ay: 0,
    phase: 'DRIVE', wing: 0, bf: 0, br: 0, tf: 0, tr: 0, sl: 0, sr: 0, gas: 1, upFace: true, rl: 0, p: 0, tAir: 0, gs: 1,
    sens: { lidar: { r: NaN, ok: false, t: -1 }, radar: { r: NaN, vr: NaN, ok: false, t: -1 }, cam: { r: NaN, ok: false, t: -1 } },
    est: { h: NaN, vz: 0, ok: false, z: NaN, sd: NaN },
    hIgn: NaN, hG: NaN, vG: NaN, aReq: 0, thRef: 0, xTd: NaN, slopeTd: 0, tti: Infinity, thr: 0,
    freePath: null, freeVn: NaN, tDetect: NaN, tIgn: NaN,
    vt: 0, result: null, events: [], ae: null, rnd: rng(7),
    gx: 0, gy: 0, gz: 1, gHead: 0, gPeak: 1,
  };
}

function aero(c) {
  const vax = c.vx + cfg.wind, vay = c.vy;
  const V = Math.hypot(vax, vay) || 1e-6, q = 0.5 * P.rho * V * V;
  const ux = vax / V, uy = vay / V, gam = Math.atan2(vay, vax), a = wrap(c.th - gam);
  const D = q * ms() * (P.bodyCdA + (P.bodySideCdA - P.bodyCdA) * Math.abs(Math.sin(a)) + P.wingCdA * c.wing);
  let M = -q * 0.4 * ms() * c.w;                     // damping khí động
  const Mknown = M;
  const W = cfg.mass * P.g, Bp = P.thrTW * W / 2, Tp = P.thrTopTW * W / 2;
  const BF = c.bf * Bp, BR = c.br * Bp, TF = c.tf * Tp, TR = c.tr * Tp;
  const T = (BF + BR - TF - TR) * Math.cos(c.rl);    // lực dọc trục +y thân, chiếu vào mặt phẳng dọc (xe lật ngang → hụt lực)
  M += P.thrArm * (BF - BR - TF + TR);               // gầm-trước + nóc-sau = ngóc mũi; gầm-sau + nóc-trước = chúi mũi
  const tx = -Math.sin(c.th) * T, ty = Math.cos(c.th) * T;
  const Fx0 = -D * ux, Fy0 = -D * uy - W;
  // roll: jet hông trái thổi sang trái → đẩy nóc sang phải (+), jet phải ngược lại; damping khí động.
  // Lực ngang của jet hông không đưa vào quỹ đạo (mô hình 2,5D: roll là bậc tự do riêng).
  const Sp = P.sideTW * W, MrKnown = -q * 0.25 * ms() * c.p;
  const Mr = MrKnown + P.sideArm * (c.sl - c.sr) * Sp;
  return { V, q, gam, a, D, Mknown, M, T, Fx0, Fy0, Fx: Fx0 + tx, Fy: Fy0 + ty, Mr, MrKnown,
    drag: [-D * ux, -D * uy], weight: [0, -W], thrust: [tx, ty] };
}

function hAglOf(c) { return c.y - P.wheelR - groundY(c.x); }
// tốc độ tiếp cận mặt đất ḣ: trên sườn dốc mặt đất "lùi" xuống dưới xe khi xe bay ngang
function hDotOf(c) { return c.vy - Math.tan(slopeAt(c.x)) * c.vx; }
function log(c, msg) { c.events.push({ t: c.t, msg }); }

// ---------- CẢM BIẾN + HỢP NHẤT ----------
// Mỗi cảm biến đo khoảng cách thẳng đứng tới mặt đất ở tần số riêng, có nhiễu; radar đo thêm vận tốc Doppler.
function sense(c, dt) {
  const r = c.rnd, h = hAglOf(c), S = P.sens, s = c.sens;
  const tilt = Math.cos(c.th) * Math.cos(c.rl);                           // cos góc giữa trục "lên" của xe và phương đứng
  const faceOk = Math.abs(tilt) >= Math.cos(P.sensFov * D2R);              // mặt gầm hoặc mặt nóc đang nhìn xuống đất
  c.upFace = tilt >= 0;
  if (!faceOk) { s.lidar.ok = s.radar.ok = s.cam.ok = false; }
  let fresh = false;
  const tick = (o, hz) => c.t - o.t >= 1 / hz - 1e-9;
  if (faceOk && tick(s.lidar, S.lidar.hz)) { s.lidar.t = c.t; s.lidar.ok = faceOk && h <= S.lidar.max; s.lidar.r = s.lidar.ok ? h + S.lidar.sd * gauss(r) : NaN; fresh = true; }
  if (faceOk && tick(s.radar, S.radar.hz)) { s.radar.t = c.t; s.radar.ok = faceOk && h <= S.radar.max; s.radar.r = s.radar.ok ? h + S.radar.sd * gauss(r) : NaN; s.radar.vr = s.radar.ok ? hDotOf(c) + S.radar.sdV * gauss(r) : NaN; fresh = true; }
  if (faceOk && tick(s.cam, S.cam.hz)) { s.cam.t = c.t; s.cam.ok = faceOk && h <= S.cam.max; s.cam.r = s.cam.ok ? h * (1 + S.cam.sdRel * gauss(r)) : NaN; fresh = true; }

  const e = c.est;
  // dự báo bằng IMU (gia tốc đo được) giữa các lần đo
  const tanG = Math.tan(slopeAt(c.x));               // độ dốc bên dưới (từ bản đồ địa hình lidar/camera)
  if (e.ok) { e.h += (e.vz - tanG * c.vx) * dt; e.vz += c.ay * dt; }
  if (!fresh) return;
  // hợp nhất trọng số nghịch đảo phương sai
  let wsum = 0, zsum = 0;
  if (s.lidar.ok) { const w = 1 / (S.lidar.sd ** 2); wsum += w; zsum += w * s.lidar.r; }
  if (s.radar.ok) { const w = 1 / (S.radar.sd ** 2); wsum += w; zsum += w * s.radar.r; }
  if (s.cam.ok) { const sd = Math.max(0.05, S.cam.sdRel * s.cam.r); const w = 1 / (sd * sd); wsum += w; zsum += w * s.cam.r; }
  if (!wsum) return;
  const z = zsum / wsum; e.z = z; e.sd = Math.sqrt(1 / wsum);
  if (!e.ok) {
    e.ok = true; e.h = z; e.vz = s.radar.ok ? s.radar.vr + tanG * c.vx : c.vy;
    log(c, `Cảm biến bắt được mặt đất: h≈${z.toFixed(1)} m (${[s.lidar.ok && 'lidar', s.radar.ok && 'radar', s.cam.ok && 'camera'].filter(Boolean).join(' + ')})`);
    return;
  }
  const innov = z - e.h;
  e.h += 0.5 * innov; e.vz += 0.8 * innov;              // bộ lọc α-β
  if (s.radar.ok) e.vz += 0.3 * (s.radar.vr + tanG * c.vx - e.vz);    // Doppler (ḣ) + dốc → vận tốc đứng
}

// ---------- BỘ ĐIỀU KHIỂN ----------
function control(c, dt) {
  const ae = aero(c);
  sense(c, dt);
  const e = c.est, m = cfg.mass, W = m * P.g, Tmax = P.thrTW * W;
  const vy = e.ok ? e.vz : c.vy;
  // Dẫn đường theo pháp tuyến của MẶT PHẲNG ĐẤT tại điểm chạm dự báo (địa hình phía trước từ lidar/camera):
  // d = khoảng cách vuông góc tới mặt phẳng, vz = vận tốc dọc pháp tuyến (âm = đang lao vào).
  let h = Infinity, vz = vy, xTd = c.x, tgo = Infinity, phi = 0;
  if (e.ok) for (let k = 0; k < 3; k++) {
    phi = slopeAt(xTd);
    const planeAtX = groundY(xTd) + Math.tan(phi) * (c.x - xTd);
    h = (e.h + groundY(c.x) - planeAtX) * Math.cos(phi);
    vz = -c.vx * Math.sin(phi) + vy * Math.cos(phi);
    tgo = 2 * Math.max(h, 0) / (Math.max(-vz, 0) + P.vTd);
    xTd = c.x + c.vx * Math.min(tgo, 12);
  }
  c.hG = h; c.vG = vz; c.tti = tgo; c.xTd = xTd;
  c.slopeTd = slopeAt(xTd);
  const nx = -Math.sin(c.slopeTd), ny = Math.cos(c.slopeTd);

  const cosT = Math.max(Math.cos(c.th - c.slopeTd), 0.5);
  const aMax = Math.max(Tmax * cosT / m - P.g * ny, 0.5);   // giảm tốc pháp tuyến tối đa jet làm được
  c.aReq = e.ok && vz < -P.vTd ? (vz * vz - P.vTd * P.vTd) / (2 * Math.max(h - P.hTd, 0.05)) : 0;
  c.hIgn = (vz * vz - P.vTd * P.vTd) / (2 * P.ignMargin * aMax) + P.hTd + Math.max(-vz, 0) * P.lat;

  // tư thế mục tiêu: đứng thẳng; gần đất thì khớp góc dốc tại điểm chạm (+2° ngóc mũi). wrap() chọn chiều xoay ngắn nhất.
  const p = e.ok ? smooth(clamp(1 - h / P.levelH, 0, 1)) : 0;
  c.thRef = (c.slopeTd + 2 * D2R) * p;
  const err = wrap(c.thRef - c.th), errR = wrap(-c.rl);
  // xe chưa đứng thẳng → cộng thời gian lật vào độ cao kích hoạt (kích sớm hơn)
  const flip = (e, w) => (Math.abs(e) > 0.3 ? Math.abs(e) / P.wMax + Math.abs(w) / 6 + 0.3 : 0);
  const tFlip = Math.max(flip(err, c.w), flip(errR, c.p));
  c.hIgn += Math.max(-vz, 0) * tFlip;

  if (c.phase === 'RIGHTING' && Math.abs(err) < P.rightTol * D2R && Math.abs(c.w) < P.rightW && Math.abs(errR) < P.rollTol * D2R && Math.abs(c.p) < P.rightW) {
    c.phase = 'ARMED'; log(c, `Đã dập xoay, lật xe về thế đứng (pitch ${(c.th * R2D).toFixed(0)}°, lật ngang ${(wrap(c.rl) * R2D).toFixed(0)}°) → chờ độ cao kích hoạt`);
  }
  if ((c.phase === 'ARMED' || c.phase === 'RIGHTING') && e.ok && h <= c.hIgn) {
    if (c.phase === 'RIGHTING') log(c, `Chưa lật xong nhưng đã tới độ cao kích hoạt → đốt và lật đồng thời`);
    c.phase = 'BURN'; c.tIgn = c.t;
    log(c, `KÍCH HOẠT CỨU HỘ @ d⊥=${h.toFixed(1)} m tới mặt đất: v⊥=${(-vz).toFixed(1)} m/s, cần ${(c.aReq / P.g).toFixed(2)} g → jet khí lạnh thổi xuống`);
  }

  // vòng tư thế: góc → tốc độ xoay mong muốn (bão hoà) → mô-men (bù damping khí động đã biết)
  const I = P.Ik * m;
  const wDes = clamp(P.kTh * err, -P.wMax, P.wMax);
  const Mreq = I * P.kW * (wDes - c.w) - ae.Mknown;

  // lực dọc pháp tuyến cần cho suicide burn; chiếu về trục thân: dương = jet gầm, âm = jet nóc (khi xe còn ngửa)
  let Fc = 0;
  const cosB = Math.cos(c.th - c.slopeTd) * Math.cos(c.rl);
  if (c.phase === 'BURN' && Math.abs(cosB) > 0.2) {
    const a0 = (ae.Fx0 * nx + ae.Fy0 * ny) / m;
    const aDes = vz < -P.vTd ? c.aReq : 2 * (-P.vTd - vz);
    Fc = Math.max(0, m * (aDes - a0)) / cosB;
  }

  // phân bổ: đang hãm → mô-men lấy từ chênh lệch trước/sau của chính bộ jet đang đẩy; phần thiếu (bão hoà về 0)
  // mới chuyển sang bộ đối diện. Không hãm (đang chống lộn) → CẶP CHÉO gầm-trước + nóc-sau / gầm-sau + nóc-trước:
  // hai lực triệt tiêu nhau → mô-men thuần, không đẩy lệch quỹ đạo.
  const Bp = P.thrTW * W / 2, Tp = P.thrTopTW * W / 2, half = Mreq / (2 * P.thrArm);
  let bf = 0, br = 0, tf = 0, tr = 0;
  if (Fc > 0) {
    bf = Fc / 2 + half; br = Fc / 2 - half;
    if (bf < 0) { tf = -bf; bf = 0; }          // cần chúi mũi hơn nữa → nóc-trước
    if (br < 0) { tr = -br; br = 0; }          // cần ngóc mũi hơn nữa → nóc-sau
  } else if (Fc < 0) {
    tf = -Fc / 2 - half; tr = -Fc / 2 + half;
    if (tf < 0) { bf = -tf; tf = 0; }
    if (tr < 0) { br = -tr; tr = 0; }
  } else if (half > 0) { bf = half; tr = half; } else { br = -half; tf = -half; }
  // vòng lật ngang (cùng kiểu 2 tầng): góc → tốc độ lật mong muốn → mô-men → jet hông trái/phải
  const Ir = P.IkRoll * m, Sp = P.sideTW * W;
  const pDes = clamp(P.kTh * errR, -P.wMax, P.wMax);
  const MrReq = Ir * P.kW * (pDes - c.p) - ae.MrKnown;
  let sl = 0, sr = 0;
  if (MrReq > 0) sl = MrReq / P.sideArm; else sr = -MrReq / P.sideArm;
  if (c.gas <= 0) bf = br = tf = tr = sl = sr = 0;
  const sp = P.thrSpool * dt, go = (cur, F, cap) => clamp(cur + clamp(Math.min(F / cap, 1) - cur, -sp, sp), 0, 1);
  c.bf = go(c.bf, bf, Bp); c.br = go(c.br, br, Bp); c.tf = go(c.tf, tf, Tp); c.tr = go(c.tr, tr, Tp);
  c.sl = go(c.sl, sl, Sp); c.sr = go(c.sr, sr, Sp);
  c.thr = (c.bf + c.br) / 2; c.thrTop = (c.tf + c.tr) / 2;
  const prevGas = c.gas;
  c.gas = Math.max(0, c.gas - (((c.bf + c.br) * Bp + (c.tf + c.tr) * Tp + (c.sl + c.sr) * Sp) * dt) / (m * P.gasDv));
  if (prevGas > 0 && c.gas <= 0) log(c, 'Hết khí nén!');
  c.wing = Math.min(1, c.wing + P.wingRate * dt);       // cánh gió sau dựng đứng: phanh khí
}

// Người trong xe cảm nhận GIA TỐC RIÊNG (không gồm trọng lực) chiếu lên trục thân xe:
// Gz dọc cột sống (+ = bị ép xuống ghế), Gx trước-sau (+ = bị đẩy vào lưng ghế); khi xe lộn, đầu chịu thêm ly tâm ω²·r.
function occupant(c, ae) {
  const fx = ae.Fx / cfg.mass, fy = (ae.Fy + cfg.mass * P.g) / cfg.mass;
  const ct = Math.cos(c.th), st = Math.sin(c.th);
  const up = (-fx * st + fy * ct) / P.g;            // phần vuông góc thân trong mặt phẳng dọc, chia theo góc lật ngang
  c.gx = (fx * ct + fy * st) / P.g;
  c.gz = up * Math.cos(c.rl);
  c.gy = -up * Math.sin(c.rl);                      // ngang người (+ = bị đẩy sang phải)
  c.gHead = (c.w * c.w + c.p * c.p) * P.occ.rHead / P.g;
  c.gPeak = Math.max(c.gPeak, Math.hypot(c.gx, c.gy, c.gz) + c.gHead);
}
function occRisk(g) { for (const [lim, name] of P.occ.bands) if (g < lim) return name; return 'rất cao, khó sống sót'; }

function touchdown(c, hitIdx) {
  const phi = slopeAt(c.x);
  const nx = -Math.sin(phi), ny = Math.cos(phi);
  const vn = -(c.vx * nx + c.vy * ny);
  const pe = wrap(c.th - phi) * R2D;
  const rollDeg = wrap(c.rl) * R2D;
  const wheelFirst = hitIdx < 2 && Math.abs(rollDeg) < 25;      // lật ngang > 25° → hông/nóc chạm trước
  // tải lên người khi chạm: giảm tốc đều trên quãng giảm tải → a = v⊥²/(2·quãng)
  const gLoad = 1 + Math.max(vn, 0) ** 2 / (2 * (wheelFirst ? P.occ.strokeWheel : P.occ.strokeBody) * P.g);
  c.gPeak = Math.max(c.gPeak, gLoad);
  let grade = 'CRASH';
  if (wheelFirst && vn <= P.vnSafe && Math.abs(pe) <= P.peSafe && Math.abs(rollDeg) <= P.peSafe) grade = 'SAFE';
  else if (wheelFirst && vn <= P.vnHard && Math.abs(pe) <= P.peHard && Math.abs(rollDeg) <= P.peHard) grade = 'HARD';
  c.result = { grade, vn, pe, rollDeg, gLoad, wheelFirst, V: Math.hypot(c.vx, c.vy), x: c.x, t: c.t };
  log(c, `Chạm đất: ${grade} · v⊥=${vn.toFixed(2)} m/s · Δpitch=${pe.toFixed(1)}° · lật ngang ${rollDeg.toFixed(1)}° · ${wheelFirst ? 'bánh chạm trước' : 'thân chạm trước'}`);
  log(c, `Người trong xe: tải đỉnh ${gLoad.toFixed(1)} g → nguy cơ chấn thương ${occRisk(gLoad)}${gLoad > 4 ? ' · túi khí bung' : ''}`);
  c.vt = c.vx * Math.cos(phi) + c.vy * Math.sin(phi);
  c.bf = c.br = c.tf = c.tr = c.sl = c.sr = c.thr = c.thrTop = 0;
  if (grade === 'CRASH') { c.phase = 'CRASH'; c.w = clamp(c.w, -3, 3) + (pe > 0 ? 2 : -2); }
  else { c.phase = 'ROLLOUT'; c.w = 0; }
}

function contact(c) {
  const cs = Math.cos(c.th), sn = Math.sin(c.th);
  let best = -1, depth = 0;
  CONTACT_PTS.forEach(([px, py], i) => {
    const wx = c.x + px * cs - py * sn, wy = c.y + px * sn + py * cs;
    const d = groundY(wx) - wy;
    if (d > depth) { depth = d; best = i; }
  });
  if (best >= 0) touchdown(c, best);
}

// IMU xác nhận rơi tự do → mô phỏng nhanh quỹ đạo không cứu hộ để biết mức nguy hiểm
function detectAndPredict(c) {
  c.tDetect = c.t;
  log(c, `IMU: gia tốc riêng ${c.gs.toFixed(2)} g suốt ${P.ffDetect} s → XÁC NHẬN RƠI TỰ DO (h=${hAglOf(c).toFixed(0)} m)`);
  const f = Object.assign(makeCar(false), { x: c.x, y: c.y, vx: c.vx, vy: c.vy, th: c.th, w: c.w, rl: c.rl, p: c.p, t: c.t, phase: 'FREEFALL' });
  const path = [[c.x, c.y]];
  for (let i = 0; i < 120 * 60 && !f.result; i++) { step(f, 1 / 120); if (i % 6 === 0) path.push([f.x, f.y]); }
  path.push([f.x, f.y]);
  c.freePath = path; c.freeVn = f.result ? f.result.vn : NaN; c.freeG = f.result ? f.result.gLoad : NaN;
  c.freeTurns = Math.abs(c.w) * (f.t - c.tLaunch) / (2 * Math.PI);
  c.freeRollTurns = Math.abs(c.p) * (f.t - c.tLaunch) / (2 * Math.PI);
  log(c, `DỰ ĐOÁN không cứu hộ: lộn ~${c.freeTurns.toFixed(1)} vòng + lật ngang ~${c.freeRollTurns.toFixed(1)} vòng, chạm đất sau ${(f.t - c.t).toFixed(1)} s, v⊥≈${c.freeVn.toFixed(0)} m/s, người ~${c.freeG.toFixed(0)} g → ARM cứu hộ: bật lidar/radar/camera, dựng cánh phanh khí`);
}

function step(c, dt) {
  c.t += dt;
  if (c.phase === 'DRIVE') {
    c.x += c.vx * dt;
    if (c.ctrl && !c.warned && c.x > -40) { c.warned = true; log(c, 'Xuống dốc đèo: phanh quá nhiệt, mất phanh trước cua tay áo'); }
    if (c.x > 2.4) {
      c.phase = 'FREEFALL';
      c.w = -cfg.spin * D2R;                         // húc lan can + trọng tâm qua mép → xe lộn vòng chúi mũi
      c.p = cfg.roll * D2R;                          // húc lệch một bên → xe lật ngang
      c.drop0 = cfg.H - groundY(c.x + 30); c.tLaunch = c.t;
      if (c.ctrl) log(c, 'Không vào được cua → húc văng lan can, lao khỏi vực');
    }
    return;
  }
  if (c.phase === 'ROLLOUT' || c.phase === 'CRASH') {
    const phi = slopeAt(c.x);
    if (c.phase === 'ROLLOUT') {
      c.vt = Math.max(0, c.vt + (-P.g * Math.sin(phi) - 5) * dt);
      c.th += (phi - c.th) * Math.min(1, 12 * dt);
      c.rl *= 1 - Math.min(1, 12 * dt); c.p = 0;
      c.wing = Math.max(0, c.wing - 0.6 * dt);
    } else {
      c.vt *= 1 - Math.min(1, 1.6 * dt);
      c.w *= 1 - Math.min(1, 0.8 * dt);
      c.th += c.w * dt;
    }
    c.x += c.vt * Math.cos(phi) * dt;
    c.y = groundY(c.x) + (c.phase === 'CRASH' ? 0.9 : P.wheelR);
    c.vx = c.vt * Math.cos(phi); c.vy = c.vt * Math.sin(phi);
    c.gz = 1; c.gx = c.phase === 'ROLLOUT' && c.vt > 0 ? -0.5 : 0; c.gHead = 0;
    return;
  }
  if (c.ctrl && (c.phase === 'RIGHTING' || c.phase === 'ARMED' || c.phase === 'BURN')) control(c, dt);
  const ae = aero(c);
  c.ae = ae;
  const I = P.Ik * cfg.mass;
  c.ay = ae.Fy / cfg.mass;
  c.vx += (ae.Fx / cfg.mass) * dt;
  c.vy += c.ay * dt;
  c.w += (ae.M / I) * dt;
  c.th = wrap(c.th + c.w * dt);
  c.p += (ae.Mr / (P.IkRoll * cfg.mass)) * dt;
  c.rl = wrap(c.rl + c.p * dt);
  c.x += c.vx * dt;
  c.y += c.vy * dt;
  c.gs = Math.hypot(ae.Fx, ae.Fy + cfg.mass * P.g) / (cfg.mass * P.g);
  occupant(c, ae);
  if (c.ctrl && c.phase === 'FREEFALL') {
    c.tAir = c.gs < P.ffG ? c.tAir + dt : 0;
    if (c.tAir >= P.ffDetect) { detectAndPredict(c); c.phase = 'RIGHTING'; log(c, `Con quay: lộn ${(c.w * R2D).toFixed(0)}°/s, lật ngang ${(c.p * R2D).toFixed(0)}°/s → CHỐNG LỘN: cặp jet chéo gầm/nóc + jet hông trái/phải`); }
  }
  contact(c);
}
// ==SIM-CORE-END==

(function () {
  "use strict";
  const T = window.THREE;
  const $ = (id) => document.getElementById(id);
  if (!T) { $("body").innerHTML = "<p>Không tải được three.js (../viewer/vendor/three.min.js). Mở file từ trong repo.</p>"; return; }
  const REDUCED = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const fmt = (x, d = 1) => (isFinite(x) ? x.toFixed(d).replace(".", ",") : "–");
  const ease = (u) => { u = clamp(u, 0, 1); return u < .5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2; };
  const AIR = (p) => p === 'FREEFALL' || p === 'RIGHTING' || p === 'ARMED' || p === 'BURN';

  // ============================================================ rollout (tính trước → scrub được như viewer)
  const REC = 4, SIM_DT = 1 / 240, FRAME_DT = REC * SIM_DT;
  const opt = { ctrl: true, vec: true, paths: true, sensors: true, slowmo: true };
  let RUN = null;

  function snap(c) {
    const ae = AIR(c.phase) ? c.ae : null, s = c.sens, e = c.est;
    return {
      t: c.t, x: c.x, y: c.y, th: c.th, vx: c.vx, vy: c.vy, phase: c.phase, gs: c.phase === 'DRIVE' ? 1 : c.gs, tAir: c.tAir,
      gx: c.gx, gy: c.gy, gz: c.gz, rl: c.rl, p: c.p, sl: c.sl, sr: c.sr, gHead: c.gHead, gPeak: c.gPeak, det: isFinite(c.tDetect), wing: c.wing, bf: c.bf, br: c.br, tf: c.tf, tr: c.tr, thr: c.thr, thrTop: c.thrTop || 0, gas: c.gas, w: c.w, upFace: Math.cos(c.th) >= 0,
      thRef: c.thRef, xTd: c.xTd, slopeTd: c.slopeTd, tti: c.tti, hG: c.hG, vG: c.vG, aReq: c.aReq, hIgn: c.hIgn, h: hAglOf(c),
      eh: e.ok ? e.h : NaN, evz: e.ok ? e.vz : NaN, esd: e.sd,
      li: s.lidar.ok ? s.lidar.r : NaN, ra: s.radar.ok ? s.radar.r : NaN, rv: s.radar.ok ? s.radar.vr : NaN, ca: s.cam.ok ? s.cam.r : NaN,
      V: ae ? ae.V : Math.hypot(c.vx, c.vy), F: ae ? { D: ae.drag, W: ae.weight, T: ae.thrust } : null,
    };
  }

  function simulate() {
    const m = makeCar(opt.ctrl);
    const F = [];
    let tEnd = Infinity;
    for (let i = 0; i < 240 * 150; i++) {
      if (i % REC === 0) F.push({ m: snap(m) });
      step(m, SIM_DT);
      if (m.result && tEnd === Infinity) tEnd = Math.max(m.t, m.result.t) + 3;
      if (m.t > tEnd) break;
    }
    const chapStart = [0, NaN, NaN, NaN, NaN, NaN];
    F.forEach((f) => { const k = chapterOf(f.m); for (let j = 1; j <= k; j++) if (isNaN(chapStart[j])) chapStart[j] = f.m.t; });
    return { F, m, events: m.events, res: m.result,
      freePath: m.freePath, chapStart, tMax: F[F.length - 1].m.t };
  }

  function chapterOf(s) {
    if (s.phase === 'DRIVE') return 0;
    if (s.phase === 'FREEFALL') return 1;
    if (s.phase === 'RIGHTING') return 2;
    if (s.phase === 'ARMED') return 3;
    if (s.phase === 'BURN') return 4;
    return 5;
  }

  // ============================================================ palette (CSS token → màu three)
  const KEYS = ["sc-bg", "sc-ground", "sc-road", "sc-mark", "sc-rock", "sc-rock2", "sc-grass", "sc-sky", "sc-gnd-light", "sc-cab", "sc-wheel",
    "sc-tree", "sc-mtn", "vf-red", "drl", "plume", "taillight", "ego", "blue", "yellow", "red", "green", "teal", "gold", "grey", "purple", "ink", "ink-3"];
  const PAL = {}, MATS = {};
  function readPal() { const cs = getComputedStyle(document.documentElement); KEYS.forEach((k) => { PAL[k] = new T.Color(cs.getPropertyValue("--" + k).trim() || "#888"); }); }
  const isDark = () => PAL["sc-bg"].getHSL({}).l < .3;
  function mat(key, o, Type) { const m = new (Type || T.MeshLambertMaterial)(Object.assign({}, o || {})); (MATS[key] = MATS[key] || []).push(m); if (PAL[key]) m.color.copy(PAL[key]); return m; }
  readPal();

  // ============================================================ renderer, scene, lights
  const stage = $("stage"), labels = $("labels");
  const renderer = new T.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
  stage.insertBefore(renderer.domElement, labels);
  const scene = new T.Scene(); scene.background = new T.Color(); scene.fog = new T.Fog(0xffffff, 220, 1100);
  const camera = new T.PerspectiveCamera(38, 1, 0.3, 4000);
  const hemi = new T.HemisphereLight(0xffffff, 0x888888, 0.6); scene.add(hemi);
  const sun = new T.DirectionalLight(0xffffff, 0.7); sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048); { const sc = sun.shadow.camera; sc.left = -60; sc.right = 60; sc.top = 60; sc.bottom = -60; sc.near = 10; sc.far = 400; }
  sun.shadow.bias = -0.0008; scene.add(sun); scene.add(sun.target);
  const fill = new T.DirectionalLight(0xffffff, 0.28); scene.add(fill); scene.add(fill.target);   // đèn phụ theo camera: nhìn từ phía sau xe không bị tối
  function applyPal() {
    readPal();
    Object.keys(MATS).forEach((k) => MATS[k].forEach((m) => m.color.copy(PAL[k])));
    scene.background.copy(PAL["sc-bg"]); scene.fog.color.copy(PAL["sc-bg"]);
    hemi.color.copy(PAL["sc-sky"]); hemi.groundColor.copy(PAL["sc-gnd-light"]);
    hemi.intensity = isDark() ? 0.62 : 0.55; sun.intensity = isDark() ? 0.55 : 0.68;
    smokeMat.color.copy(PAL["plume"]); smokeMat.opacity = isDark() ? 0.22 : 0.4;
    drawPlot();
  }

  // ============================================================ camera rig (orbit + follow + tween, như viewer)
  const SIDE = Math.PI / 2;   // nhìn ngang từ phía +z; yaw = π là nhìn từ phía sau xe
  const rig = { tx: -60, ty: cfg.H, tz: 0, dist: 48, yaw: SIDE + 0.95, pitch: 0.42, follow: true, lead: 10 };
  let tween = null;
  function camTo(p, dur) {
    const from = { tx: rig.tx, ty: rig.ty, tz: rig.tz, dist: rig.dist, yaw: rig.yaw, pitch: rig.pitch, lead: rig.lead };
    const to = Object.assign({}, from, p);
    const dy = to.yaw - from.yaw; to.yaw = from.yaw + Math.atan2(Math.sin(dy), Math.cos(dy));
    tween = { from, to, t0: performance.now(), dur: REDUCED ? 1 : (dur || 1400) };
  }
  function stepRig(dtReal, target) {
    if (tween) {
      const u = ease((performance.now() - tween.t0) / tween.dur);
      ["dist", "yaw", "pitch", "lead"].forEach((k) => { rig[k] = tween.from[k] + (tween.to[k] - tween.from[k]) * u; });
      if (!rig.follow) ["tx", "ty", "tz"].forEach((k) => { rig[k] = tween.from[k] + (tween.to[k] - tween.from[k]) * u; });
      if (u >= 1) tween = null;
    }
    if (rig.follow && target) {
      // seek/scrub xa → nhảy thẳng tới xe thay vì trôi chậm qua cả trăm mét
      const far = Math.hypot(target.x + rig.lead - rig.tx, target.y - rig.ty) > 60;
      const k = far ? 1 : 1 - Math.exp(-dtReal * 6);
      rig.tx += (target.x + rig.lead - rig.tx) * k; rig.ty += (target.y - rig.ty) * k; rig.tz += (0 - rig.tz) * k;
    }
    const cp = Math.cos(rig.pitch);
    camera.position.set(rig.tx + rig.dist * cp * Math.cos(rig.yaw), rig.ty + rig.dist * Math.sin(rig.pitch), rig.tz + rig.dist * cp * Math.sin(rig.yaw));
    camera.lookAt(rig.tx, rig.ty + 1, rig.tz);
    fill.position.copy(camera.position); fill.target.position.set(rig.tx, rig.ty, rig.tz);
  }
  function setFollow(on) { rig.follow = on; }
  (function pointer() {
    const el = renderer.domElement, ptrs = {}; let last = null, pinch = 0;
    el.addEventListener("pointerdown", (e) => { ptrs[e.pointerId] = [e.clientX, e.clientY]; el.setPointerCapture(e.pointerId); last = [e.clientX, e.clientY]; tween = null; });
    el.addEventListener("pointermove", (e) => {
      if (!ptrs[e.pointerId] || camMode === "onboard" || camMode === "ground" || INTERIOR(camMode)) return;
      const ids = Object.keys(ptrs);
      if (ids.length === 2) {
        ptrs[e.pointerId] = [e.clientX, e.clientY];
        const a = ptrs[ids[0]], b = ptrs[ids[1]], d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) rig.dist = clamp(rig.dist * pinch / d, 6, 900); pinch = d; return;
      }
      const dx = e.clientX - last[0], dy = e.clientY - last[1]; last = [e.clientX, e.clientY]; ptrs[e.pointerId] = last;
      if (e.shiftKey || e.buttons === 2) {
        if (rig.follow) setFollow(false);
        const s = rig.dist * 0.0016;
        rig.tx -= dx * s * Math.sin(rig.yaw); rig.tz += dx * s * Math.cos(rig.yaw); rig.ty += dy * s;
      } else { rig.yaw += dx * 0.006; rig.pitch = clamp(rig.pitch + dy * 0.004, -0.3, 1.48); }
    });
    const up = (e) => { delete ptrs[e.pointerId]; pinch = 0; };
    el.addEventListener("pointerup", up); el.addEventListener("pointercancel", up);
    el.addEventListener("contextmenu", (e) => e.preventDefault());
    el.addEventListener("wheel", (e) => { e.preventDefault(); tween = null; rig.dist = clamp(rig.dist * Math.exp(e.deltaY * 0.0012), 6, 900); }, { passive: false });
  })();

  // ============================================================ nhiều góc camera
  const MODES = {
    side: { dist: 40, yaw: SIDE, pitch: 0.08, lead: 0 },
    rear: { dist: 22, yaw: Math.PI, pitch: 0.14, lead: 0 },
    front: { dist: 22, yaw: 0.0001, pitch: 0.12, lead: 0 },
    top: { dist: 70, yaw: SIDE, pitch: 1.45, lead: 0 },
    drone: { dist: 34, yaw: SIDE + 0.6, pitch: 0.3, lead: 0 },
  };
  let camMode = "auto", pipOn = true;
  const camOn = new T.PerspectiveCamera(70, 1, 0.1, 4000), camGround = new T.PerspectiveCamera(45, 1, 0.3, 4000), camChase = new T.PerspectiveCamera(50, 1, 0.3, 4000);
  const camPov = new T.PerspectiveCamera(78, 1, 0.03, 4000), camCabin = new T.PerspectiveCamera(80, 1, 0.03, 4000);
  const INTERIOR = (m) => m === "pov" || m === "cabin";
  const _a = new T.Vector3(), _b = new T.Vector3(), _up = new T.Vector3();
  function poseFixed(s) {
    // gắn ở mũi xe, nhìn về trước-xuống; trục "lên" của camera theo thân xe (cả lộn lẫn lật ngang)
    car.updateMatrixWorld(true);
    _up.set(0, 1, 0).applyQuaternion(car.quaternion);
    camOn.position.copy(car.localToWorld(_a.set(2.25, 0.1, 0))); camOn.up.copy(_up); camOn.lookAt(car.localToWorld(_b.set(14, -2.8, 0)));
    // đặt dưới đất gần điểm chạm cuối, ngước lên nhìn xe
    const xg = RUN.res ? RUN.res.x : (RUN.freePath ? RUN.freePath[RUN.freePath.length - 1][0] : 60), gx = xg - 14;
    camGround.position.set(gx, groundY(gx) + 1.6, 24); camGround.lookAt(s.x, s.y, 0);
    // bám đuôi
    camChase.position.set(s.x - 16, s.y + 4, 7); camChase.lookAt(s.x + 2, s.y, 0);
    // camera trong xe: gắn vào thân xe (lộn theo xe). POV = mắt người lái (rung theo đầu); cabin = từ táp-lô nhìn về 2 người
    const drv = car.userData.people[0];
    drv.eye.getWorldPosition(_a); camPov.position.copy(_a); camPov.up.copy(_up);
    camPov.lookAt(car.localToWorld(_b.set(12, -0.6 + drv.lean * -3, -0.42)));
    camCabin.position.copy(car.localToWorld(_a.set(0.62, 0.62, 0.0))); camCabin.up.copy(_up);
    camCabin.lookAt(car.localToWorld(_b.set(-0.6, 0.34, 0)));
  }
  const activeCam = () => (camMode === "onboard" ? camOn : camMode === "ground" ? camGround : camMode === "pov" ? camPov : camMode === "cabin" ? camCabin : camera);
  function setCamMode(m) {
    camMode = m; $("camSel").value = m; rig.follow = true;
    if (m === "auto") camTo(Object.assign({}, CH[chap].cam)); else if (MODES[m]) camTo(Object.assign({}, MODES[m]));
  }
  function prepMain() {
    const r = stage.getBoundingClientRect(), ac = activeCam();
    ac.aspect = r.width / Math.max(1, r.height);
    // rail nổi che bên trái → dời tâm khung nhìn sang phần sân khấu còn trống
    if (r.width > 860) ac.setViewOffset(r.width, r.height, -216, 0, r.width, r.height); else ac.clearViewOffset();
    ac.updateProjectionMatrix();
  }
  const PIPS = [{ cam: camCabin, name: "trong cabin" }, { cam: camPov, name: "mắt người lái" }, { cam: camChase, name: "bám đuôi" }];
  const PW = 260, PH = 146, PG = 10, PB = 88;
  function layoutPip() {
    const r = stage.getBoundingClientRect();
    $("pip").innerHTML = pipOn && r.width > 860 ? PIPS.map((p, i) => PB + i * (PH + PG) + PH > r.height - 60 ? "" :
      `<div class="pipbox" style="right:16px;bottom:${PB + i * (PH + PG)}px;width:${PW}px;height:${PH}px"><span>${p.name}</span></div>`).join("") : "";
  }
  function renderViews() {
    const r = stage.getBoundingClientRect(), W = r.width, H = r.height;
    renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H);
    renderer.render(scene, activeCam());
    if (!pipOn || W <= 860) return;
    renderer.setScissorTest(true);
    PIPS.forEach((p, i) => {
      const y = PB + i * (PH + PG); if (y + PH > H - 60) return;
      p.cam.aspect = PW / PH; p.cam.clearViewOffset(); p.cam.updateProjectionMatrix();
      renderer.setViewport(W - 16 - PW, y, PW, PH); renderer.setScissor(W - 16 - PW, y, PW, PH);
      renderer.render(scene, p.cam);
    });
    renderer.setScissorTest(false);
  }

  // ============================================================ địa hình đèo (dựng lại khi đổi H / dốc)
  const BOX = new T.BoxGeometry(1, 1, 1);
  const terrain = new T.Group(); scene.add(terrain);
  const ground = new T.Mesh(new T.PlaneGeometry(4000, 1200).rotateX(-Math.PI / 2), mat("sc-ground")); ground.receiveShadow = true; ground.position.x = 600; scene.add(ground);
  const rail = (function () {
    const g = new T.Group(); scene.add(g); let hitPieces = [];
    return {
      build(H, D) {
        while (g.children.length) g.remove(g.children[0]);
        hitPieces = [];
        for (let z = -D / 2 + 1; z < 14; z += 2.5) {
          const post = new T.Mesh(BOX, mat("sc-cab")); post.scale.set(0.15, 0.9, 0.15); post.position.set(-2.6, H + 0.55, z);
          const beam = new T.Mesh(BOX, mat("grey")); beam.scale.set(0.12, 0.3, 2.5); beam.position.set(-2.6, H + 0.85, z + 1.25);
          const piece = new T.Group(); piece.add(post, beam); g.add(piece);
          if (Math.abs(z + 1.25) < 3.5) hitPieces.push(piece);
        }
      },
      update(x) {
        const k = clamp((x + 2.6) / 6, 0, 1);   // xe húc qua → các đoạn văng ra ngoài vực
        hitPieces.forEach((p, i) => { p.position.set(10 * k, -4 * k * k * (1 + i % 2), (i - 1) * 2 * k); p.rotation.set(0, 0.6 * k * (i - 1), -1.4 * k); });
      },
    };
  })();
  function buildTerrain() {
    while (terrain.children.length) { const o = terrain.children.pop(); if (o.geometry && o.geometry !== BOX) o.geometry.dispose(); }
    const H = cfg.H, D = 160;
    const cliff = new T.Mesh(BOX, mat("sc-rock")); cliff.scale.set(600, H, D); cliff.position.set(-300, H / 2, 0); cliff.receiveShadow = true; terrain.add(cliff);
    for (let i = 0; i < 9; i++) {
      const r = new T.Mesh(BOX, mat("sc-rock2")); const hh = H * (0.08 + 0.1 * ((i * 37) % 5) / 5);
      r.scale.set(2 + (i % 3), hh, D / 9 - 1); r.position.set(-1 - (i % 2), H * ((i * 0.13) % 0.8) + hh / 2, -D / 2 + (i + 0.5) * D / 9); terrain.add(r);
    }
    const top = new T.Mesh(BOX, mat("sc-grass")); top.scale.set(600, 0.2, D); top.position.set(-300, H + 0.1, 0); top.receiveShadow = true; terrain.add(top);
    // đường đèo: đoạn thẳng đổ dốc tới cua tay áo, rồi bẻ gắt chạy dọc mép vực (theo −z)
    const RW = 7.5, ZX = -7.75;
    const road = new T.Mesh(BOX, mat("sc-road")); road.scale.set(300, 0.06, RW); road.position.set(-150 + ZX - RW / 2, H + 0.23, 0); road.receiveShadow = true; terrain.add(road);
    const road2 = new T.Mesh(BOX, mat("sc-road")); road2.scale.set(RW, 0.06, D / 2 + RW / 2); road2.position.set(ZX, H + 0.23, -(D / 2 + RW / 2) / 2 + RW / 2); road2.receiveShadow = true; terrain.add(road2);
    for (let x = -296; x < ZX - RW; x += 9) { const m = new T.Mesh(BOX, mat("sc-mark")); m.scale.set(4.5, 0.02, 0.18); m.position.set(x, H + 0.27, 0); terrain.add(m); }
    for (let z = -RW; z > -D / 2; z -= 9) { const m = new T.Mesh(BOX, mat("sc-mark")); m.scale.set(0.18, 0.02, 4.5); m.position.set(ZX, H + 0.27, z); terrain.add(m); }
    for (let i = -1; i <= 1; i++) { const sg = new T.Mesh(BOX, mat("yellow", {}, T.MeshBasicMaterial)); sg.scale.set(0.1, 0.9, 1.2); sg.position.set(-1.6, H + 1.4, i * 3.2 - 6); terrain.add(sg);
      const pole = new T.Mesh(BOX, mat("sc-cab")); pole.scale.set(0.1, 1.1, 0.1); pole.position.set(-1.6, H + 0.55, i * 3.2 - 6); terrain.add(pole); }
    rail.build(H, D);
    for (let i = 0; i < 70; i++) {
      const x = -20 - ((i * 53) % 280), z = ((i * 97) % 150) - 75;
      if (Math.abs(z) < 7 || (x > -16 && z < 4)) continue;
      const hgt = 5 + (i % 4) * 1.6;
      const tr = new T.Mesh(new T.ConeGeometry(1.4 + (i % 3) * 0.4, hgt, 8), mat("sc-tree")); tr.position.set(x, H + 0.2 + hgt / 2, z); tr.castShadow = true; terrain.add(tr);
    }
    for (let i = 0; i < 14; i++) {
      const hgt = H * (1.1 + ((i * 7) % 5) * 0.35) + 60, rad = hgt * 0.9;
      const m = new T.Mesh(new T.ConeGeometry(rad, hgt, 7), mat("sc-mtn")); m.position.set(-700 + i * 190, hgt / 2 - 2, -380 - ((i * 61) % 3) * 120); terrain.add(m);
    }
    if (cfg.slope > 0) {
      const sh = new T.Shape(); sh.moveTo(0, 0); sh.lineTo(0, hillTop()); sh.lineTo(hillEnd(), 0); sh.lineTo(0, 0);
      const geo = new T.ExtrudeGeometry(sh, { depth: D, bevelEnabled: false }); geo.translate(0, 0, -D / 2);
      const hill = new T.Mesh(geo, mat("sc-grass")); hill.receiveShadow = true; terrain.add(hill);
    }
    for (let x = 25; x < 1600; x += 25) {
      const m = new T.Mesh(BOX, mat(x % 100 ? "sc-mark" : "ink-3", {}, T.MeshBasicMaterial));
      m.scale.set(x % 100 ? 0.25 : 0.5, 0.04, x % 100 ? 10 : 24); m.position.set(x, groundY(x) + 0.05, 0); m.rotation.z = slopeAt(x); terrain.add(m);
    }
  }

  // ============================================================ xe SUV điện (dáng lấy cảm hứng VF8): thân đỏ, nóc/kính đen, ốp gầm đen,
  // dải đèn chữ V ở đầu xe; thêm phần cứu hộ giả định: cánh lướt gió nóc bung làm phanh khí, jet khí lạnh ở gầm và nóc
  const WHEEL = new T.CylinderGeometry(0.38, 0.38, 0.27, 24).rotateX(Math.PI / 2);
  const RIM = new T.CylinderGeometry(0.27, 0.27, 0.285, 20).rotateX(Math.PI / 2);
  function extrude(pts, depth, bevel) {
    const sh = new T.Shape(); pts.forEach(([x, y], i) => (i ? sh.lineTo(x, y) : sh.moveTo(x, y)));
    const geo = new T.ExtrudeGeometry(sh, { depth, bevelEnabled: !!bevel, bevelSize: bevel || 0, bevelThickness: bevel || 0, bevelSegments: 2, curveSegments: 4 });
    geo.translate(0, 0, -depth / 2); return geo;
  }
  function suv(key) {
    const root = new T.Group(), g = new T.Group(); g.position.y = -P.wheelR; root.add(g);   // gốc root = trọng tâm của sim
    const W = 1.93, body = mat(key), dark = mat("sc-cab"), trim = mat("sc-wheel");
    // thân dưới (màu xe): mũi thấp vuốt, hông cao, đuôi cụt
    const shell = new T.Mesh(extrude([[2.3, 0.42], [2.36, 0.72], [2.22, 0.9], [1.95, 0.98], [1.0, 1.06], [-1.9, 1.08], [-2.3, 1.0], [-2.34, 0.6], [-2.3, 0.42]], W - 0.14, 0.07), body);
    g.add(shell);
    // nhà kính + nóc đen (two-tone), dáng coupe-SUV
    const glass = new T.Mesh(extrude([[1.0, 1.04], [0.2, 1.5], [-1.2, 1.56], [-1.95, 1.36], [-2.15, 1.07]], W - 0.36, 0.05), mat("sc-cab", { transparent: true, opacity: 0.55 }));
    // 2 người ngồi hàng trước (lái bên trái = −z). Thân + đầu nằm trong một nhóm để dịch/nghiêng theo tải lên người.
    const people = [];
    [-0.42, 0.42].forEach((z, i) => {
      const seat = new T.Mesh(BOX, dark); seat.scale.set(0.5, 0.12, 0.48); seat.position.set(-0.3, 0.8, z); g.add(seat);
      const back = new T.Mesh(BOX, dark); back.scale.set(0.12, 0.62, 0.48); back.position.set(-0.58, 1.1, z); back.rotation.z = 0.18; g.add(back);
      const person = new T.Group(); person.position.set(-0.3, 0.82, z); g.add(person);   // gốc = hông
      const spine = new T.Group(); person.add(spine);
      const torso = new T.Mesh(new T.CylinderGeometry(0.15, 0.18, 0.5, 10), mat("gold")); torso.position.set(-0.05, 0.27, 0); torso.rotation.z = 0.18; spine.add(torso);
      const neck = new T.Group(); neck.position.set(-0.09, 0.47, 0); spine.add(neck);
      const head = new T.Mesh(new T.SphereGeometry(0.11, 14, 12), mat("gold")); head.position.set(0.02, 0.12, 0); neck.add(head);
      const eye = new T.Object3D(); eye.position.set(0.1, 0.14, 0); neck.add(eye);       // điểm mắt cho camera POV
      const thigh = new T.Mesh(BOX, mat("gold")); thigh.scale.set(0.42, 0.14, 0.3); thigh.position.set(0.2, 0.04, 0); person.add(thigh);
      const belt = new T.Mesh(BOX, mat("ink-3")); belt.scale.set(0.04, 0.55, 0.06); belt.position.set(0.08, 0.3, 0); belt.rotation.x = (i ? -1 : 1) * 0.55; spine.add(belt);
      people.push({ person, spine, neck, eye, dx: 0, dy: 0, lean: 0, tilt: 0 });
    });
    // ốp trần + trụ A (đặc) để từ trong cabin không nhìn xuyên ra vòi jet nóc
    const liner = new T.Mesh(BOX, dark); liner.scale.set(1.5, 0.04, W - 0.42); liner.position.set(-0.5, 1.47, 0); g.add(liner);
    [-1, 1].forEach((s) => { const pil = new T.Mesh(BOX, dark); pil.scale.set(0.07, 0.55, 0.06); pil.position.set(0.62, 1.26, s * (W / 2 - 0.22)); pil.rotation.z = 0.98; g.add(pil); });
    const dash = new T.Mesh(BOX, dark); dash.scale.set(0.4, 0.22, W - 0.3); dash.position.set(0.72, 1.0, 0); g.add(dash);
    const wheelS = new T.Mesh(new T.TorusGeometry(0.17, 0.025, 8, 24).rotateY(Math.PI / 2), mat("ink-3")); wheelS.position.set(0.42, 1.12, -0.42); wheelS.rotation.z = -0.45; g.add(wheelS);
    g.add(glass);
    // ốp gầm + vòm bánh đen
    const clad = new T.Mesh(BOX, trim); clad.scale.set(4.62, 0.2, W + 0.02); clad.position.set(0, 0.4, 0); g.add(clad);
    [1.45, -1.45].forEach((x) => {
      const arch = new T.Mesh(new T.CylinderGeometry(0.5, 0.5, W + 0.04, 20, 1, true, -Math.PI / 2, Math.PI).rotateX(Math.PI / 2), mat("sc-wheel", { side: T.DoubleSide }));
      arch.rotation.z = Math.PI / 2; arch.position.set(x, 0.38, 0); g.add(arch);
    });
    // mặt ca-lăng đen + dải đèn ngày chữ V + cụm đèn
    const fascia = new T.Mesh(BOX, dark); fascia.scale.set(0.1, 0.36, W - 0.3); fascia.position.set(2.36, 0.56, 0); g.add(fascia);
    const drl = mat("drl", {}, T.MeshBasicMaterial);
    [-1, 1].forEach((s) => {
      const bar = new T.Mesh(BOX, drl); bar.scale.set(0.04, 0.035, 0.86); bar.position.set(2.31, 0.84, s * 0.42); bar.rotation.x = s * 0.13; g.add(bar);
      const lamp = new T.Mesh(BOX, drl); lamp.scale.set(0.05, 0.07, 0.22); lamp.position.set(2.28, 0.78, s * 0.78); g.add(lamp);
      const mirror = new T.Mesh(BOX, dark); mirror.scale.set(0.22, 0.12, 0.16); mirror.position.set(0.85, 1.1, s * (W / 2 + 0.06)); g.add(mirror);
      const belt = new T.Mesh(BOX, mat("grey")); belt.scale.set(2.9, 0.025, 0.02); belt.position.set(-0.55, 1.08, s * (W / 2 - 0.17)); g.add(belt);
    });
    const vtip = new T.Mesh(BOX, drl); vtip.scale.set(0.05, 0.1, 0.06); vtip.position.set(2.32, 0.8, 0); g.add(vtip);
    // đèn hậu dạng dải ngang
    const tl = new T.Mesh(BOX, mat("taillight", {}, T.MeshBasicMaterial)); tl.scale.set(0.04, 0.05, W - 0.2); tl.position.set(-2.33, 0.93, 0); g.add(tl);
    // bánh xe: lốp đen + mâm khí động 5 chấu
    [1.45, -1.45].forEach((x) => [-1, 1].forEach((s) => {
      const w = new T.Mesh(WHEEL, trim); w.position.set(x, 0.38, s * (W / 2 - 0.12)); g.add(w);
      const rim = new T.Mesh(RIM, mat("ink-3")); rim.position.copy(w.position); g.add(rim);
      for (let k = 0; k < 5; k++) { const sp = new T.Mesh(BOX, mat("sc-wheel")); sp.scale.set(0.05, 0.24, 0.02); sp.position.set(x, 0.38, s * (W / 2 - 0.12 + 0.145)); sp.rotation.z = k * Math.PI * 2 / 5; sp.translateY(0.1); g.add(sp); }
    }));
    // cánh lướt gió nóc: bình thường nằm ngang, bung lên ~65° làm phanh khí
    const wingPivot = new T.Group(); wingPivot.position.set(-1.6, 1.55, 0); g.add(wingPivot);
    const plate = new T.Mesh(BOX, mat(key)); plate.scale.set(0.6, 0.04, W - 0.3); plate.position.x = -0.3; wingPivot.add(plate);
    // lidar trên nóc (như ego của viewer)
    const lid = new T.Mesh(new T.CylinderGeometry(0.13, 0.15, 0.13, 14), dark); lid.position.set(0.15, 1.6, 0); g.add(lid);
    const ring = new T.Mesh(new T.TorusGeometry(0.15, 0.022, 6, 20).rotateX(Math.PI / 2), mat("ego", {}, T.MeshBasicMaterial)); ring.position.set(0.15, 1.64, 0); g.add(ring);
    // jet khí lạnh: 4 ở gầm (bf/br, thổi xuống) + 4 ở nóc (tf/tr, thổi lên) — trước/sau × trái/phải
    const jets = [];
    const mkPlume = () => {
      const plume = new T.Group();
      const outer = new T.Mesh(new T.ConeGeometry(0.34, 1, 16, 1, true).translate(0, -0.5, 0), mat("plume", { transparent: true, opacity: 0.35, depthWrite: false, side: T.DoubleSide }, T.MeshBasicMaterial));
      const inner = new T.Mesh(new T.CylinderGeometry(0.12, 0.05, 1, 12, 1, true).translate(0, -0.5, 0), mat("plume", { transparent: true, opacity: 0.75, depthWrite: false }, T.MeshBasicMaterial));
      plume.add(outer, inner); return plume;
    };
    [[1.75, 'bf', 0.22, 0.08, 1, 0.8], [-1.85, 'br', 0.22, 0.08, 1, 0.8], [0.55, 'tf', 1.56, 1.66, -1, 0.55], [-1.15, 'tr', 1.6, 1.7, -1, 0.55]].forEach(([x, k, ny, py, dir, zz]) => [-1, 1].forEach((s) => {
      const noz = new T.Mesh(new T.CylinderGeometry(dir > 0 ? 0.1 : 0.14, dir > 0 ? 0.15 : 0.1, 0.2, 12), mat(dir > 0 ? "grey" : "ink-3")); noz.position.set(x, ny, s * zz); g.add(noz);
      const plume = mkPlume(); plume.position.set(x, py, s * zz); if (dir < 0) plume.rotation.x = Math.PI; g.add(plume);
      jets.push({ k, plume, lp: new T.Vector3(x, py - P.wheelR, s * zz), ld: new T.Vector3(0, -dir, 0) });
    }));
    // 4 jet hông: 2 bên trái (sl, z < 0) + 2 bên phải (sr, z > 0), ngang mép nóc, thổi ra ngoài
    [[1.4, 1.0], [-1.3, 1.22]].forEach(([x, y]) => [-1, 1].forEach((s) => {   // trước: mép chắn bùn (không che tầm nhìn người lái); sau: mép nóc
      const zz = s * (W / 2 + 0.04);
      const noz = new T.Mesh(new T.CylinderGeometry(0.09, 0.13, 0.16, 12).rotateX(Math.PI / 2), mat("ink-3")); noz.position.set(x, y, zz); g.add(noz);
      const plume = mkPlume(); plume.position.set(x, y, zz + s * 0.06); plume.rotation.x = -s * Math.PI / 2; g.add(plume);
      jets.push({ k: s < 0 ? 'sl' : 'sr', plume, lp: new T.Vector3(x, y - P.wheelR, zz + s * 0.06), ld: new T.Vector3(0, 0, s) });
    }));
    root.traverse((o) => { if (o.isMesh) o.castShadow = true; });
    jets.forEach((j) => j.plume.traverse((o) => { o.castShadow = false; }));
    root.userData = { wingPivot, jets, people };
    return root;
  }
  const car = suv("vf-red"); scene.add(car);

  // khói khí lạnh: phun xuống, chạm đất thì loang ra (như ảnh)
  const NSM = 700, smokePos = new Float32Array(NSM * 3), smokeVel = new Float32Array(NSM * 3), smokeLife = new Float32Array(NSM);
  for (let i = 0; i < NSM; i++) smokePos[i * 3 + 1] = -9999;
  const smokeGeo = new T.BufferGeometry(); smokeGeo.setAttribute("position", new T.BufferAttribute(smokePos, 3));
  const puff = (() => { const c = document.createElement("canvas"); c.width = c.height = 64; const x = c.getContext("2d"); const g = x.createRadialGradient(32, 32, 0, 32, 32, 32);
    g.addColorStop(0, "rgba(255,255,255,1)"); g.addColorStop(0.5, "rgba(255,255,255,.45)"); g.addColorStop(1, "rgba(255,255,255,0)"); x.fillStyle = g; x.fillRect(0, 0, 64, 64); return new T.CanvasTexture(c); })();
  const smokeMat = new T.PointsMaterial({ size: 3.2, map: puff, transparent: true, opacity: 0.4, depthWrite: false, sizeAttenuation: true });
  const smoke = new T.Points(smokeGeo, smokeMat); smoke.frustumCulled = false; scene.add(smoke);
  let smokeHead = 0;
  const _sp = new T.Vector3(), _sd = new T.Vector3();
  function stepSmoke(dt, s) {
    if (dt <= 0) return;
    if (s && opt.ctrl && AIR(s.phase)) {
      car.updateMatrixWorld(true);
      car.userData.jets.forEach((j) => {
        const f = s[j.k]; if (f < 0.05) return;
        const n = Math.ceil(f * 3);
        const wp = car.localToWorld(_sp.copy(j.lp)), wd = _sd.copy(j.ld).applyQuaternion(car.quaternion);
        for (let q = 0; q < n; q++) {
          const i = smokeHead = (smokeHead + 1) % NSM, v = 18 + 14 * f;
          smokePos[i * 3] = wp.x; smokePos[i * 3 + 1] = wp.y; smokePos[i * 3 + 2] = wp.z;
          smokeVel[i * 3] = s.vx * 0.6 + wd.x * v + (Math.random() - .5) * 2; smokeVel[i * 3 + 1] = s.vy * 0.6 + wd.y * v; smokeVel[i * 3 + 2] = wd.z * v + (Math.random() - .5) * 2;
          smokeLife[i] = 1.6 + Math.random();
        }
      });
    }
    for (let i = 0; i < NSM; i++) {
      if (smokeLife[i] <= 0) { smokePos[i * 3 + 1] = -9999; continue; }
      smokeLife[i] -= dt;
      const gy = groundY(smokePos[i * 3]) + 0.4;
      if (smokePos[i * 3 + 1] <= gy) {   // chạm đất → loang ngang
        smokePos[i * 3 + 1] = gy;
        if (smokeVel[i * 3 + 1] < 0) { const sp = Math.abs(smokeVel[i * 3 + 1]) * 0.7, a = Math.random() * Math.PI * 2; smokeVel[i * 3] += Math.cos(a) * sp; smokeVel[i * 3 + 2] += Math.sin(a) * sp; smokeVel[i * 3 + 1] = 1.5; }
      }
      const k = Math.exp(-2.2 * dt);
      smokeVel[i * 3] *= k; smokeVel[i * 3 + 1] = smokeVel[i * 3 + 1] * k + 0.6 * dt; smokeVel[i * 3 + 2] *= k;
      smokePos[i * 3] += smokeVel[i * 3] * dt; smokePos[i * 3 + 1] += smokeVel[i * 3 + 1] * dt; smokePos[i * 3 + 2] += smokeVel[i * 3 + 2] * dt;
    }
    smokeGeo.attributes.position.needsUpdate = true;
  }

  // vector lực
  const arrows = {};
  [["D", "gold"], ["W", "grey"], ["T", "teal"]].forEach(([k, c]) => { const a = new T.ArrowHelper(new T.Vector3(0, 1, 0), new T.Vector3(), 1, PAL[c].getHex(), 0.9, 0.5); a.userData.key = c; scene.add(a); arrows[k] = a; });

  // đường & vòng đánh dấu
  function lineObj(color, dashed, opacity) {
    const m = dashed ? new T.LineDashedMaterial({ dashSize: 2.5, gapSize: 2, transparent: true, opacity: opacity || 1 })
      : new T.LineBasicMaterial({ transparent: true, opacity: opacity || 1 });
    (MATS[color] = MATS[color] || []).push(m); m.color.copy(PAL[color]);
    const l = new T.Line(new T.BufferGeometry(), m); l.frustumCulled = false; scene.add(l); return l;
  }
  function segObj(color, opacity) {
    const m = new T.LineBasicMaterial({ transparent: true, opacity: opacity || 1 }); (MATS[color] = MATS[color] || []).push(m); m.color.copy(PAL[color]);
    const l = new T.LineSegments(new T.BufferGeometry(), m); l.frustumCulled = false; scene.add(l); return l;
  }
  const freeLine = lineObj("gold", true), planLine = lineObj("teal", false, 0.22), doneLine = lineObj("teal", false);
  const ignLine = lineObj("teal", true, 0.9);
  const lidarSeg = segObj("blue"), camSeg = segObj("gold", 0.8);
  const radarCone = new T.Mesh(new T.ConeGeometry(1, 1, 24, 1, true).translate(0, -0.5, 0), mat("purple", { transparent: true, opacity: 0.16, depthWrite: false, side: T.DoubleSide }, T.MeshBasicMaterial));
  radarCone.rotation.x = 0; scene.add(radarCone);
  const ringMat = (k, o) => mat(k, { transparent: true, opacity: o, side: T.DoubleSide }, T.MeshBasicMaterial);
  const tdRing = new T.Mesh(new T.RingGeometry(1.6, 2.2, 32).rotateX(-Math.PI / 2), ringMat("teal", 0.85)); scene.add(tdRing);
  const freeRing = new T.Mesh(new T.RingGeometry(1.8, 2.6, 32).rotateX(-Math.PI / 2), ringMat("gold", 0.8)); scene.add(freeRing);
  const v3 = (x, y, z) => new T.Vector3(x, y, z || 0);
  function setLine(l, pts, dashed) {
    l.geometry.dispose(); l.geometry = new T.BufferGeometry().setFromPoints(pts.map((p) => (p.isVector3 ? p : v3(p[0], p[1], p[2] || 0))));
    if (dashed) l.computeLineDistances();
  }

  // ============================================================ nhãn nổi
  const LBL = {};
  function lbl(id) { if (!LBL[id]) { const d = document.createElement("div"); d.className = "lbl"; labels.appendChild(d); LBL[id] = d; } return LBL[id]; }
  const _v = new T.Vector3();
  function place(id, cls, text, x, y, z) {
    const el = lbl(id); el.className = "lbl " + (cls || "");
    if (INTERIOR(camMode)) { el.hidden = true; return; }
    _v.set(x, y, z || 0).project(activeCam());
    const r = renderer.domElement.getBoundingClientRect();
    if (_v.z > 1 || Math.abs(_v.x) > 1.2 || Math.abs(_v.y) > 1.2) { el.hidden = true; return; }
    el.hidden = false; el.textContent = text;
    el.style.left = ((_v.x + 1) / 2 * r.width) + "px"; el.style.top = ((1 - _v.y) / 2 * r.height - 6) + "px";
  }
  function hide(id) { if (LBL[id]) LBL[id].hidden = true; }

  // ============================================================ các bước
  const sensRow = (name, v, ok) => ok ? fmt(v, 2) + " m" : "ngoài tầm";
  const CH = [
    { short: "1 · Đường đèo", title: "Xuống đèo, mất phanh",
      cam: { dist: 48, yaw: SIDE + 0.95, pitch: 0.42, lead: 10 },
      html: () => `<p>Xe đổ dốc dài trên đường đèo, mép đường là vực sâu <b>${fmt(cfg.H - groundY(5), 0)} m</b>. Chiếc SUV điện <b>~${fmt(cfg.mass / 1000, 1)} tấn</b> phanh quá nhiệt, lao vào <b>cua tay áo</b> ở <b>${fmt(cfg.v0 * 3.6, 0)} km/h</b>.</p>
        <p>Quãng phanh cần <span class="k">v²/(2·0,8g) ≈ ${fmt(cfg.v0 * cfg.v0 / (2 * 0.8 * P.g), 0)} m</span> nhưng phanh đã mất. Xe <span class="c-red">húc văng lan can</span> và lao khỏi mép vực.</p>
        <p class="try">Hệ thống cứu hộ <b>chưa làm gì</b>: nó chỉ được phép kích hoạt khi chắc chắn xe đang rơi.</p>`,
      read: [["Tốc độ", () => fmt(cfg.v0 * 3.6, 0) + " km/h"], ["Cách mép vực", (s) => s.x < 0 ? fmt(-s.x, 1) + " m" : "đã rơi"], ["Độ sâu vực", () => fmt(cfg.H - groundY(5), 0) + " m"], ["IMU", (s) => fmt(s.gs, 2) + " g"]] },
    { short: "2 · Rơi tự do", title: "IMU xác nhận rơi tự do",
      cam: { dist: 55, yaw: SIDE + 0.35, pitch: 0.16, lead: 6 },
      html: () => `<p>Khi rơi tự do, cảm biến gia tốc đo <b>gia tốc riêng ≈ 0 g</b>. Hệ thống yêu cầu <span class="k">|a_riêng| &lt; ${fmt(P.ffG, 1)} g</span> liên tục <b>${fmt(P.ffDetect, 2)} s</b> để không kích hoạt nhầm khi xe chỉ xóc nảy.</p>
        <p>Xác nhận xong, nó mô phỏng nhanh đường rơi <b>không cứu hộ</b>: quỹ đạo <span class="c-gold">cam đứt</span>, chạm đất với <b>v⊥ ≈ ${fmt(RUN.m.freeVn, 0)} m/s</b> (ngưỡng an toàn ${fmt(P.vnSafe, 1)} m/s). Hệ thống <b>ARM</b>: bật cảm biến khoảng cách, dựng cánh gió, giữ thân ngang. Jet <b>chưa</b> phụt.</p>`,
      read: [["IMU a_riêng", (s) => fmt(s.gs, 2) + " g"], ["Con quay ω", (s) => fmt(s.w * R2D, 0) + " °/s"], ["Thời gian < ngưỡng", (s) => fmt(Math.min(s.tAir, P.ffDetect), 2) + " / " + fmt(P.ffDetect, 2) + " s"], ["v⊥ nếu không cứu", () => fmt(RUN.m.freeVn, 1) + " m/s"], ["Trạng thái", (s) => s.phase === 'FREEFALL' ? "đang xác nhận…" : "ARMED"]] },
    { short: "3 · Chống lộn", title: "Dập xoay, lật về thế đứng",
      cam: { dist: 26, yaw: SIDE + 0.25, pitch: 0.12, lead: 0 },
      html: () => `<p>Húc lan can làm xe <b>lộn vòng ${fmt(cfg.spin, 0)} °/s</b> và <b>lật ngang ${fmt(cfg.roll, 0)} °/s</b>. Dự đoán của hệ thống: nếu không xử lý, xe lộn <b>~${fmt(RUN.m.freeTurns, 1)} vòng</b> và lật ngang <b>~${fmt(RUN.m.freeRollTurns, 1)} vòng</b> rồi đập xuống bằng nóc, hông hoặc mũi.</p>
        <p>Hệ thống dùng <b>cặp jet chéo</b>: <span class="k">gầm-trước + nóc-sau</span> tạo mô-men ngóc mũi, <span class="k">gầm-sau + nóc-trước</span> tạo mô-men chúi mũi. Hai lực trong cặp triệt tiêu nhau nên chỉ còn <b>mô-men thuần</b>, không đẩy lệch quỹ đạo.</p>
        <p><b>Lật ngang</b> do <b>4 jet hông</b> xử lý (2 trái, 2 phải, gắn ngang mép nóc): jet bên trái thổi ra ngoài đẩy nóc sang phải, jet bên phải ngược lại. Vòng điều khiển giống hệt pitch: <span class="k">p* = clamp(${P.kTh}·wrap(−φᵣ), ±${P.wMax})</span> → <span class="k">Mᵣ = Iᵣ·${P.kW}·(p* − p)</span>. Khi xe đang nghiêng ngang, lực jet gầm bị hụt theo <span class="k">cos φᵣ</span>.</p>
        <p>Vòng tư thế 2 tầng: <span class="k">ω* = clamp(${P.kTh}·wrap(θref − θ), ±${P.wMax} rad/s)</span> → <span class="k">M = I·${P.kW}·(ω* − ω)</span>. <span class="k">wrap()</span> chọn chiều lật ngắn nhất kể cả khi xe đang ngửa bụng. Xong khi cả hai trục đều |góc| &lt; ${P.rightTol}° và |tốc độ xoay| &lt; ${fmt(P.rightW, 1)} rad/s.</p>
        <p class="try">Cảm biến đặt ở <b>cả gầm và nóc</b>; chỉ đo được khi một mặt nhìn xuống trong ±${P.sensFov}°. Lúc xe đang lộn, ước lượng độ cao chạy bằng IMU giữa các lần đo.</p>`,
      read: [["Lộn ω / lật ngang p", (s) => fmt(s.w * R2D, 0) + " / " + fmt(s.p * R2D, 0) + " °/s"], ["Pitch θ / lật ngang φᵣ", (s) => fmt(wrap(s.th) * R2D, 0) + "° / " + fmt(wrap(s.rl) * R2D, 0) + "°"], ["Tư thế", (s) => { const c = Math.cos(s.th) * Math.cos(s.rl); return c < -0.2 ? "NGỬA BỤNG" : c < 0.6 ? "nghiêng gắt" : "gần đứng"; }],
        ["Jet hông trái / phải", (s) => fmt(s.sl * 100, 0) + " / " + fmt(s.sr * 100, 0) + " %"],
        ["Jet gầm trước / sau", (s) => fmt(s.bf * 100, 0) + " / " + fmt(s.br * 100, 0) + " %"], ["Jet nóc trước / sau", (s) => fmt(s.tf * 100, 0) + " / " + fmt(s.tr * 100, 0) + " %"], ["Cảm biến", (s) => isFinite(s.li) || isFinite(s.ra) ? (s.upFace ? "mặt gầm" : "mặt nóc") : "mất (đang lộn)"]] },
    { short: "4 · Cảm biến", title: "Lidar, radar, camera đo đất",
      cam: { dist: 42, yaw: SIDE + 0.55, pitch: 0.06, lead: 2 },
      html: () => `<p>Ba cảm biến đo khoảng cách tới mặt đất bên dưới: <span class="c-blue">lidar</span> (chính xác ±${fmt(P.sens.lidar.sd, 2)} m, tầm ${P.sens.lidar.max} m), <span class="c-purple" style="color:var(--purple)">radar</span> (±${fmt(P.sens.radar.sd, 1)} m, tầm ${P.sens.radar.max} m, thêm vận tốc Doppler) và <span class="c-gold">camera</span> (độ sâu ±${Math.round(P.sens.cam.sdRel * 100)}%, tầm ${P.sens.cam.max} m).</p>
        <p>Hợp nhất trọng số nghịch đảo phương sai <span class="k">h = Σ(hᵢ/σᵢ²) / Σ(1/σᵢ²)</span>, rồi lọc α-β với gia tốc IMU giữa các lần đo. Địa hình phía trước cho ra mặt phẳng tại <b>điểm chạm dự báo</b>; từ đó tính khoảng cách vuông góc <b>d⊥</b> và vận tốc lao vào <b>v⊥</b>.</p>
        <p>Độ cao kích hoạt (vạch <span class="c-teal">teal đứt</span>): <span class="k">h_kích = v⊥²/(2·${fmt(P.ignMargin, 1)}·a_max) + v⊥·t_trễ + h_td</span>. Trong lúc chờ, jet chỉ nhích chênh lệch trước/sau để giữ thân ngang.</p>`,
      read: [["Lidar", (s) => sensRow("l", s.li, isFinite(s.li))], ["Radar · Doppler", (s) => isFinite(s.ra) ? fmt(s.ra, 1) + " m · " + fmt(s.rv, 1) + " m/s" : "ngoài tầm"], ["Camera", (s) => sensRow("c", s.ca, isFinite(s.ca))],
        ["Hợp nhất h", (s) => isFinite(s.eh) ? fmt(s.eh, 2) + " ± " + fmt(s.esd, 2) + " m" : "–"], ["d⊥ / h_kích", (s) => fmt(s.hG, 1) + " / " + fmt(s.hIgn, 1) + " m"], ["v⊥", (s) => fmt(-s.vG, 1) + " m/s"], ["TTI", (s) => isFinite(s.tti) && s.tti < 60 ? fmt(s.tti, 2) + " s" : "–"]] },
    { short: "5 · Cứu hộ", title: "Kích hoạt jet khí lạnh",
      cam: { dist: 24, yaw: SIDE + 1.25, pitch: 0.1, lead: 0 },
      html: () => `<p>Khi <b>d⊥ ≤ h_kích</b>, 4 jet gầm thổi xuống (<b>suicide burn</b>): đốt muộn nhất có thể để tiết kiệm khí, nhưng vẫn còn ${Math.round((1 - P.ignMargin) * 100)}% lực dự phòng.</p>
        <p>Mỗi bước tính lại gia tốc cần <span class="k">a_req = (v⊥² − ${fmt(P.vTd, 0)}) / (2·(d⊥ − h_td))</span> và lực đẩy <span class="k">T = m·(a_req − a₀) / cos(θ − φ)</span>, với a₀ là trọng lực + cản khí dọc pháp tuyến. Cặp jet chéo tiếp tục giữ thân khớp góc dốc φ (+2°). Nếu tới độ cao kích hoạt mà xe chưa lật xong (độ cao kích hoạt đã cộng sẵn thời gian lật), hệ thống <b>vừa đốt vừa lật</b>; khi xe còn ngửa, lực hãm do jet nóc gánh. <b>Cánh gió sau dựng đứng</b> thêm lực cản khí động.</p>`,
      read: [["d⊥", (s) => fmt(s.hG, 1) + " m"], ["v⊥", (s) => fmt(-s.vG, 2) + " m/s"], ["a_req", (s) => fmt(s.aReq / P.g, 2) + " g"], ["Lực đẩy / trọng lượng", (s) => fmt(s.thr * P.thrTW, 2)], ["Jet gầm trước / sau", (s) => fmt(s.bf * 100, 0) + " / " + fmt(s.br * 100, 0) + " %"], ["Jet nóc trước / sau", (s) => fmt(s.tf * 100, 0) + " / " + fmt(s.tr * 100, 0) + " %"], ["Khí nén còn", (s) => fmt(s.gas * 100, 0) + "%"], ["θ − φ", (s) => fmt((s.th - s.slopeTd) * R2D, 1) + "°"]] },
    { short: "6 · Chạm đất", title: "Chạm đất và chấm điểm",
      cam: { dist: 32, yaw: SIDE + 0.75, pitch: 0.28, lead: -2 },
      html: () => { const r = RUN.res;
        return `<p>Lúc điểm đầu tiên chạm đất: <b>an toàn</b> nếu bánh chạm trước, <span class="k">v⊥ ≤ ${fmt(P.vnSafe, 1)} m/s</span> <span class="k">|θ − φ| ≤ ${P.peSafe}°</span> và <span class="k">|lật ngang| ≤ ${P.peSafe}°</span>; <b>cứng</b> nếu ≤ ${P.vnHard} m/s và ≤ ${P.peHard}°; còn lại là <span class="c-red">đâm/lật</span>.</p>
        ${r ? `<p>Xe có cứu hộ: <b class="${r.grade === 'SAFE' ? 'c-green' : r.grade === 'HARD' ? 'c-gold' : 'c-red'}">${r.grade}</b> · v⊥ ${fmt(r.vn, 2)} m/s · Δpitch ${fmt(r.pe, 1)}° · lật ngang ${fmt(r.rollDeg, 1)}°.</p>` : ""}
        ${RUN.m.freePath ? `<p>So với dự đoán không cứu hộ: v⊥ <b class="c-red">${fmt(RUN.m.freeVn, 0)} m/s</b>, người ~<b class="c-red">${fmt(RUN.m.freeG, 0)} g</b> (${occRisk(RUN.m.freeG)}).</p>` : ""}`; },
      read: [["Tải lên người khi chạm", () => RUN.res ? fmt(RUN.res.gLoad, 1) + " g · " + occRisk(RUN.res.gLoad) : "–"], ["Nếu không cứu hộ (dự đoán)", () => RUN.m.freePath ? fmt(RUN.m.freeG, 0) + " g · " + occRisk(RUN.m.freeG) : "–"], ["Thời gian rơi", () => RUN.res ? fmt(RUN.res.t - RUN.chapStart[1], 1) + " s" : "–"], ["Thời gian đốt jet", () => RUN.res && isFinite(RUN.chapStart[4]) ? fmt(RUN.res.t - RUN.chapStart[4], 1) + " s" : "–"], ["Khí nén dùng", (s) => fmt((1 - s.gas) * 100, 0) + "%"]] },
  ];

  let chap = 0, t = 0, playing = false, speed = 1, userSeek = false, plotKey = "h", logCount = -1;

  function renderChapters() { $("chapters").innerHTML = CH.map((c, i) => `<button type="button" data-i="${i}" ${i === chap ? 'aria-current="step"' : ''}>${c.short}</button>`).join(""); }
  $("chapters").addEventListener("click", (e) => { const b = e.target.closest("button"); if (b) goChapter(+b.dataset.i, true); });
  const slider = (k, label, mn, mx, st, u) => `<label class="sl"><span>${label}</span><output>${cfg[k]} ${u}</output><input id="s_${k}" data-u="${u}" type="range" min="${mn}" max="${mx}" step="${st}" value="${cfg[k]}"></label>`;
  const tog = (k, label) => `<label class="tog"><input id="t_${k}" type="checkbox" ${opt[k] ? "checked" : ""}>${label}</label>`;

  function renderBody() {
    const c = CH[chap];
    $("body").innerHTML = `
      <div class="eyebrow">Bước ${chap + 1} / ${CH.length}</div>
      <h2>${c.title}</h2>
      ${c.html()}
      <div class="read">${c.read.map(([k], i) => `<span>${k}</span><b id="r${i}">–</b>`).join("")}</div>
      <div class="occ" aria-live="polite">
        <div class="occ-head"><span class="occ-dot" id="occDot"></span>Người trong xe <span class="note" style="margin-left:auto">2 người · thắt dây an toàn</span></div>
        <div class="occ-state" id="occState">–</div>
        <div class="read" style="background:none;padding:0"><span>Dọc cột sống Gz</span><b id="occGz">–</b><span>Trước–sau Gx</span><b id="occGx">–</b><span>Ly tâm ở đầu (lộn vòng)</span><b id="occGh">–</b><span>Tải đỉnh đến lúc này</span><b id="occPk">–</b></div>
        <div class="gbar" title="tải tổng hiện tại, thang 0–10 g"><i id="occBar"></i></div>
        <p class="note">Mốc nguy cơ (tham khảo gần đúng, không phải tiêu chuẩn y khoa): &lt; 3 g rất thấp · 3–10 g thấp · 10–20 g trung bình · 20–50 g cao · ≥ 50 g khó sống sót.</p>
      </div>
      <div class="legend">
        <span><i style="background:var(--teal)"></i>xe cứu hộ / quỹ đạo</span>
        <span class="c-gold"><i class="dash"></i><span style="color:var(--ink-2)">rơi tự do dự báo</span></span>
        <span><i style="background:var(--blue)"></i>lidar</span><span><i style="background:var(--purple)"></i>radar</span><span><i style="background:var(--gold)"></i>camera</span>
      </div>
      <div class="plotbox">
        <div class="plothead"><span>biểu đồ</span><select id="plotSel">
          <option value="h">độ cao: thật · hợp nhất · kích hoạt</option><option value="sens">sai số từng cảm biến</option>
          <option value="v">v⊥ & a_req</option><option value="thr">jet gầm · jet nóc · khí nén</option><option value="th">pitch θ, θref, dốc</option><option value="w">lộn ω & lật ngang p (°/s)</option><option value="occ">tải lên người (g)</option></select></div>
        <canvas id="plot" width="640" height="360" aria-label="Biểu đồ theo thời gian"></canvas>
      </div>
      <details class="stu" ${chap === 0 ? "open" : ""}><summary>Kịch bản</summary><div class="stugrid">
        ${slider("H", "Độ cao vực", 60, 400, 10, "m")}${slider("v0", "Tốc độ khi lao ra", 10, 45, 1, "m/s")}
        ${slider("wind", "Gió ngược (+) / xuôi (−)", -15, 15, 1, "m/s")}${slider("slope", "Dốc chân vực", 0, 30, 1, "°")}
        ${slider("mass", "Khối lượng", 1800, 3000, 50, "kg")}${slider("spin", "Lộn (chúi mũi) sau va chạm", 0, 400, 10, "°/s")}${slider("roll", "Lật ngang sau va chạm", 0, 400, 10, "°/s")}
        <div class="togs">${tog("ctrl", "Cứu hộ")}${tog("sensors", "Tia cảm biến")}${tog("vec", "Vector lực")}${tog("paths", "Quỹ đạo")}${tog("slowmo", "Chậm khi đốt jet")}</div>
        <p class="note">Lật ngang là một bậc tự do riêng (mô hình 2,5D): lực ngang của jet hông chưa đưa vào quỹ đạo. Mô hình xe là hình khối đơn giản lấy cảm hứng từ dáng một mẫu SUV điện (VF8), không dùng dữ liệu hay thông số của hãng; mẫu xe đó <b>không</b> có hệ thống cứu hộ này. Ý tưởng jet khí lạnh lấy cảm hứng từ "gói SpaceX" từng được công bố cho Tesla Roadster thế hệ 2 (chưa xác minh). Mọi số liệu ở đây (jet gầm ${P.thrTW}× và jet nóc ${P.thrTopTW}× trọng lượng, khí đủ Δv ${P.gasDv} m/s, sai số cảm biến) là giả định để minh hoạ thuật toán.</p>
      </div></details>
      <details class="stu"><summary>Nhật ký quyết định</summary><div id="log"></div></details>
      <div class="navrow"><button class="btn" id="prev" ${chap === 0 ? "disabled" : ""}>← Trước</button><button class="btn" id="next" ${chap === CH.length - 1 ? "disabled" : ""}>Tiếp →</button></div>`;
    $("prev").onclick = () => goChapter(chap - 1, true);
    $("next").onclick = () => goChapter(chap + 1, true);
    const sel = $("plotSel"); sel.value = plotKey; sel.onchange = () => { plotKey = sel.value; drawPlot(); };
    ["H", "v0", "wind", "slope", "mass", "spin", "roll"].forEach((k) => { const el = $("s_" + k); el.oninput = () => { cfg[k] = +el.value; el.previousElementSibling.textContent = el.value + " " + el.dataset.u; rebuild(); }; });
    Object.keys(opt).forEach((k) => { const el = $("t_" + k); if (el) el.onchange = () => { opt[k] = el.checked; if (k === "ctrl") rebuild(); }; });
    logCount = -1; drawPlot();
  }

  function goChapter(i, seek) {
    chap = clamp(i, 0, CH.length - 1);
    renderChapters(); renderBody(); markLoop();
    if (seek) { let ts = RUN.chapStart[chap]; if (!isFinite(ts)) ts = chap >= 1 ? RUN.tMax : 0; t = ts; userSeek = true; }
    if (camMode === "auto") camTo(Object.assign({}, CH[chap].cam));
  }
  function markLoop() { document.querySelectorAll(".loop .node").forEach((n) => { const k = +n.dataset.n; n.classList.toggle("on", k === chap); n.classList.toggle("done", k < chap); }); }

  // ============================================================ biểu đồ (trong rail)
  function drawPlot() {
    const cv = $("plot"); if (!cv || !RUN) return;
    const ctx = cv.getContext("2d"), W = cv.width, H = cv.height;
    const cs = getComputedStyle(document.documentElement), col = (k) => cs.getPropertyValue("--" + k).trim();
    ctx.clearRect(0, 0, W, H);
    const series = {
      h: [["h thật", col("teal"), (s) => s.h], ["h hợp nhất", col("blue"), (s) => s.eh], ["h_kích", col("gold"), (s) => s.phase === 'ARMED' ? s.hIgn : NaN]],
      sens: [["lidar", col("blue"), (s) => s.li - s.h], ["radar", col("purple"), (s) => s.ra - s.h], ["camera", col("gold"), (s) => s.ca - s.h]],
      v: [["v⊥ m/s", col("teal"), (s) => -s.vG], ["a_req m/s²", col("gold"), (s) => s.phase === 'BURN' ? s.aReq : NaN]],
      thr: [["gầm %", col("blue"), (s) => s.thr * 100], ["nóc %", col("purple"), (s) => s.thrTop * 100], ["khí %", col("teal"), (s) => s.gas * 100]],
      w: [["lộn ω °/s", col("red"), (s) => s.w * R2D], ["lật ngang p °/s", col("purple"), (s) => s.p * R2D]],
      occ: [["Gz", col("teal"), (s) => s.gz], ["Gx", col("blue"), (s) => s.gx], ["ly tâm đầu", col("red"), (s) => s.gHead]],
      th: [["θ", col("teal"), (s) => wrap(s.th) * R2D], ["θref", col("yellow"), (s) => s.thRef * R2D], ["dốc", col("green"), (s) => s.slopeTd * R2D]],
    }[plotKey];
    const F = RUN.F.filter((f) => AIR(f.m.phase));
    const l = 52, r = W - 14, tp = 34, b = H - 30;
    ctx.font = "600 20px " + cs.getPropertyValue("--f-mono"); ctx.textBaseline = "middle";
    let lx = l; series.forEach(([n, c]) => { ctx.fillStyle = c; ctx.fillRect(lx, 14, 18, 4); ctx.fillStyle = col("ink-2"); ctx.fillText(n, lx + 24, 16); lx += ctx.measureText(n).width + 46; });
    if (F.length < 2) return;
    let mn = Infinity, mx = -Infinity;
    F.forEach((f) => series.forEach(([, , g]) => { const v = g(f.m); if (isFinite(v)) { mn = Math.min(mn, v); mx = Math.max(mx, v); } }));
    if (!isFinite(mn)) return;
    if (mx - mn < 1) { mx += .5; mn -= .5; }
    const pad = (mx - mn) * .08; mn -= pad; mx += pad;
    const t0 = F[0].m.t, t1 = F[F.length - 1].m.t;
    const X = (tt) => l + (tt - t0) / (t1 - t0) * (r - l), Y = (v) => b - (v - mn) / (mx - mn) * (b - tp);
    ctx.strokeStyle = col("line"); ctx.lineWidth = 1; ctx.fillStyle = col("ink-3"); ctx.textAlign = "right"; ctx.font = "600 16px " + cs.getPropertyValue("--f-mono");
    for (let i = 0; i <= 3; i++) { const v = mn + (mx - mn) * i / 3, y = Y(v); ctx.beginPath(); ctx.moveTo(l, y); ctx.lineTo(r, y); ctx.stroke(); ctx.fillText(Math.abs(mx - mn) < 6 ? v.toFixed(1) : v.toFixed(0), l - 6, y); }
    ctx.textAlign = "left";
    [2, 3, 4].forEach((k) => { const ts = RUN.chapStart[k]; if (!isFinite(ts) || ts < t0 || ts > t1) return; ctx.strokeStyle = col("line"); ctx.beginPath(); ctx.moveTo(X(ts), tp); ctx.lineTo(X(ts), b); ctx.stroke(); ctx.fillStyle = col("ink-3"); ctx.fillText(CH[k].short.split(" · ")[1], X(ts) + 4, b + 16); });
    series.forEach(([, c, g]) => {
      ctx.strokeStyle = c; ctx.lineWidth = plotKey === "sens" ? 1.5 : 2.5; ctx.beginPath(); let on = false;
      F.forEach((f) => { const v = g(f.m); if (!isFinite(v)) { on = false; return; } on ? ctx.lineTo(X(f.m.t), Y(v)) : ctx.moveTo(X(f.m.t), Y(v)); on = true; });
      ctx.stroke();
    });
    if (t >= t0 && t <= t1) { ctx.strokeStyle = col("ink"); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(X(t), tp); ctx.lineTo(X(t), b); ctx.stroke(); }
  }

  // ============================================================ dựng lại khi đổi kịch bản
  function rebuild() {
    RUN = simulate();
    buildTerrain();
    const F = RUN.F;
    setLine(planLine, F.filter((f) => f.m.phase !== 'DRIVE').map((f) => [f.m.x, f.m.y]));
    if (RUN.freePath && RUN.freePath.length > 1) setLine(freeLine, RUN.freePath, true);
    const fp = RUN.freePath && RUN.freePath[RUN.freePath.length - 1];
    if (fp) { freeRing.position.set(fp[0], groundY(fp[0]) + 0.08, 0); freeRing.rotation.z = slopeAt(fp[0]); }
    $("time").max = RUN.tMax.toFixed(2);
    const colors = ["grey", "gold", "red", "blue", "teal", "green"];
    $("ticks").innerHTML = RUN.chapStart.map((ts, i) => isFinite(ts) && i ? `<i style="left:${ts / RUN.tMax * 100}%;background:var(--${colors[i]})" title="${CH[i].title}"></i>` : "").join("");
    t = 0; chap = 0;
    renderChapters(); renderBody(); markLoop();
    if (camMode === "auto") camTo(Object.assign({}, CH[0].cam), 600);
    $("toast").hidden = true;
  }

  // ============================================================ cập nhật cảnh theo thời gian t
  function poseCar(obj, s, rescue) {
    obj.position.set(s.x, s.y, 0); obj.rotation.order = "ZXY"; obj.rotation.set(s.rl || 0, 0, s.th);   // pitch trong mặt phẳng, rồi lật ngang quanh trục dọc xe
    const u = obj.userData;
    u.wingPivot.rotation.z = rescue ? 1.1 * s.wing : 0;      // cánh dựng đứng = phanh khí
    // người: Gz < 1 → nhổm khỏi ghế (dây an toàn giữ), Gz > 1 → lún xuống; Gx < 0 (giảm tốc) → chúi về trước; xoay → đầu lắc
    if (u.people) u.people.forEach((p, i) => {
      const k = 0.25, w = s.w || 0;
      p.dy += (clamp((1 - s.gz) * 0.05, -0.06, 0.07) - p.dy) * k;
      p.dx += (clamp(-s.gx * 0.06, -0.1, 0.12) - p.dx) * k;
      p.lean += (clamp(s.gx * 0.18, -0.35, 0.3) - p.lean) * k;
      p.tilt += (clamp(-w * 0.12 + Math.sin(s.t * 9 + i) * Math.min(Math.abs(w), 4) * 0.03, -0.5, 0.5) - p.tilt) * k;
      p.side = (p.side || 0) + (clamp((s.gy || 0) * 0.25 - (s.p || 0) * 0.08, -0.45, 0.45) - (p.side || 0)) * k;
      p.person.position.x = -0.3 + p.dx; p.person.position.y = 0.82 + p.dy; p.person.rotation.x = p.side;
      p.spine.rotation.z = p.lean; p.neck.rotation.z = p.tilt + p.lean * 0.6;
    });
    u.jets.forEach((j) => {
      const f = rescue ? s[j.k] : 0;
      j.plume.visible = f > 0.02;
      const L = 0.5 + 4 * f; j.plume.scale.set(0.7 + 0.8 * f, L, 0.7 + 0.8 * f);
    });
  }
  function setArrow(a, v, origin) {
    if (!v || !opt.vec) { a.visible = false; return; }
    const len = Math.hypot(v[0], v[1]) / 3000;
    if (len < 0.4) { a.visible = false; return; }
    a.visible = true; a.position.copy(origin); a.setDirection(v3(v[0], v[1], 0).normalize()); a.setLength(len, Math.min(1.2, len * 0.3), Math.min(0.7, len * 0.18));
    a.setColor(PAL[a.userData.key]);
  }

  function update(dtReal, dtSim) {
    const F = RUN.F, i = clamp(Math.floor(t / FRAME_DT), 0, F.length - 1), f = F[i], s = f.m;
    poseCar(car, s, opt.ctrl);
    stepSmoke(dtSim, s);

    const o = v3(s.x, s.y, 0);
    ["D", "W", "T"].forEach((k) => setArrow(arrows[k], s.F ? s.F[k] : null, o));

    // quỹ đạo
    planLine.visible = opt.paths && opt.ctrl;
    let first = i; while (first > 0 && F[first - 1].m.phase !== 'DRIVE') first--;
    if (doneLine.userData.k !== first + ":" + i) { setLine(doneLine, F.slice(first, i + 1).map((g) => [g.m.x, g.m.y])); doneLine.userData.k = first + ":" + i; }
    doneLine.visible = opt.paths;
    freeLine.visible = freeRing.visible = opt.paths && opt.ctrl && s.det;
    rail.update(s.x);

    // cảm biến
    const sensOn = opt.sensors && opt.ctrl && (s.phase === 'RIGHTING' || s.phase === 'ARMED' || s.phase === 'BURN');
    const ct = Math.cos(s.th), st = Math.sin(s.th), wp = (lx, ly) => [s.x + lx * ct - ly * st, s.y + lx * st + ly * ct];
    lidarSeg.visible = sensOn && isFinite(s.li);
    if (lidarSeg.visible) {
      const fy = s.upFace ? -0.45 : 0.75, [ax, ay] = wp(-0.45, fy), pts = [];
      for (let k = 0; k <= 6; k++) { const gx = s.x + k * Math.max(4, s.vx * 0.35); const gy = groundY(gx); if (ay - gy > P.sens.lidar.max) continue; pts.push(v3(ax, ay, 0), v3(gx, gy + 0.05, 0)); }
      lidarSeg.geometry.dispose(); lidarSeg.geometry = new T.BufferGeometry().setFromPoints(pts);
    }
    radarCone.visible = sensOn && isFinite(s.ra);
    if (radarCone.visible) { const [ax, ay] = wp(2.2, s.upFace ? -0.2 : 0.4), L = Math.max(0.5, ay - groundY(ax)); radarCone.position.set(ax, ay, 0); radarCone.scale.set(L * Math.tan(7 * D2R), L, L * Math.tan(7 * D2R)); }
    camSeg.visible = sensOn && isFinite(s.ca);
    if (camSeg.visible) {
      const [ax, ay] = wp(0.75, s.upFace ? -0.3 : 0.75), cx = s.x + 5, gy = groundY(cx), w = Math.max(2, (ay - gy) * 0.35);
      const c4 = [[cx - w, -w], [cx + w, -w], [cx + w, w], [cx - w, w]].map(([x, z]) => v3(x, groundY(x) + 0.05, z));
      const pts = []; c4.forEach((p, k) => { pts.push(v3(ax, ay, 0), p, p, c4[(k + 1) % 4]); });
      camSeg.geometry.dispose(); camSeg.geometry = new T.BufferGeometry().setFromPoints(pts);
    }
    // vạch độ cao kích hoạt, song song mặt phẳng tại điểm chạm
    ignLine.visible = opt.ctrl && (s.phase === 'ARMED' || s.phase === 'RIGHTING') && isFinite(s.hIgn) && isFinite(s.hG) && isFinite(s.xTd);
    let ignP = null;
    if (ignLine.visible) {
      const phi = s.slopeTd, nx = -Math.sin(phi), ny = Math.cos(phi), tx = Math.cos(phi), ty = Math.sin(phi);
      const bx = s.xTd + nx * s.hIgn, by = groundY(s.xTd) + ny * s.hIgn; ignP = [bx, by];
      setLine(ignLine, [[bx - tx * 40, by - ty * 40], [bx + tx * 40, by + ty * 40]], true);
    }

    const live = AIR(s.phase) && s.phase !== 'FREEFALL';
    tdRing.visible = opt.ctrl && live && isFinite(s.xTd);
    if (tdRing.visible) { tdRing.position.set(s.xTd, groundY(s.xTd) + 0.1, 0); tdRing.rotation.z = slopeAt(s.xTd); }

    sun.position.set(s.x + 40, s.y + 90, 50); sun.target.position.set(s.x, s.y, 0);

    const k = chapterOf(s);
    if (k !== chap && !userSeek) { chap = k; renderChapters(); renderBody(); markLoop(); if (camMode === "auto") camTo(Object.assign({}, CH[chap].cam)); }
    userSeek = false;
    if (camMode === "drone" && !tween) rig.yaw += dtReal * 0.35;
    stepRig(dtReal, { x: s.x, y: s.y });
    poseFixed(s); prepMain();
    const hud = $("cabinHud"); hud.hidden = !INTERIOR(camMode);
    if (!hud.hidden) {
      const rH = RUN.res && s.t >= RUN.res.t ? RUN.res : null, gT = rH ? rH.gLoad : Math.hypot(s.gx, s.gz) + s.gHead;
      const pv = { DRIVE: "đang chạy", FREEFALL: "RƠI TỰ DO", RIGHTING: "CHỐNG LỘN", ARMED: "chờ kích hoạt", BURN: "JET HÃM", ROLLOUT: "đã đáp", CRASH: "ĐÂM" }[s.phase];
      hud.innerHTML = `<span>${camMode === "pov" ? "mắt người lái" : "trong cabin"} · ${pv}</span><span class="${gT > 3 ? "hot" : ""}">tải ${fmt(gT, 1)} g</span><span>Gz ${fmt(s.gz, 2)} · Gx ${fmt(s.gx, 2)}</span><span class="${Math.abs(s.w * R2D) > 90 ? "hot" : ""}">lộn ${fmt(Math.abs(s.w * R2D), 0)}°/s</span><span class="${Math.abs(s.p * R2D) > 90 ? "hot" : ""}">lật ngang ${fmt(wrap(s.rl) * R2D, 0)}° · ${fmt(Math.abs(s.p * R2D), 0)}°/s</span><span>độ cao ${fmt(Math.max(0, s.h), 0)} m</span>`;
    }

    // nhãn
    const res = RUN.res && s.t >= RUN.res.t ? RUN.res : null;
    const gradeCls = (g) => g === 'SAFE' ? 'ok' : g === 'HARD' ? 'pred' : 'hit';
    const phaseVN = { DRIVE: "đang chạy", FREEFALL: "rơi tự do", RIGHTING: "CHỐNG LỘN", ARMED: "ARMED", BURN: "JET", ROLLOUT: "lăn bánh", CRASH: "đâm" }[s.phase];
    place("car", res ? gradeCls(res.grade) : "plan", `${opt.ctrl ? "cứu hộ" : "không cứu hộ"} · ${res ? res.grade : phaseVN} · ${fmt(s.V, 0)} m/s · người ${fmt(res ? res.gLoad : Math.hypot(s.gx, s.gz) + s.gHead, 1)} g`, s.x, s.y + 2.6);
    if (s.phase === 'DRIVE' && s.x > -45) place("curve", "hit", "cua tay áo · mất phanh", -7.75, cfg.H + 4, -8); else hide("curve");
    if (s.phase === 'RIGHTING' && opt.ctrl) place("spin", "spin", `CHỐNG LỘN · lộn ${fmt(wrap(s.th) * R2D, 0)}° (${fmt(s.w * R2D, 0)}°/s) · ngang ${fmt(wrap(s.rl) * R2D, 0)}° (${fmt(s.p * R2D, 0)}°/s)`, s.x, s.y - 3.4); else hide("spin");
    if (s.phase === 'FREEFALL' && opt.ctrl) place("imu", "det", `IMU ${fmt(s.gs, 2)} g · ${fmt(Math.min(s.tAir, P.ffDetect), 2)}/${fmt(P.ffDetect, 2)} s`, s.x, s.y + 5); else hide("imu");
    const sensLbl = s.phase === 'ARMED' || s.phase === 'RIGHTING';
    if (lidarSeg.visible && sensLbl) place("li", "det", `lidar ${fmt(s.li, 1)} m`, s.x - 0.4, (s.y + groundY(s.x)) / 2, 0); else hide("li");
    if (radarCone.visible && sensLbl) place("ra", "radar", `radar ${fmt(s.ra, 1)} m · ${fmt(s.rv, 1)} m/s`, s.x + 2.2, (s.y + groundY(s.x + 2.2)) / 2 - 3, 0); else hide("ra");
    if (camSeg.visible && sensLbl) place("ca", "cam", `camera ${fmt(s.ca, 1)} m`, s.x + 5, groundY(s.x + 5) + 1.5, 4); else hide("ca");
    if (ignP && isFinite(s.hG)) place("ign", "ign", `độ cao kích hoạt · ${fmt(s.hIgn, 0)} m · còn ${fmt(s.hG - s.hIgn, 0)} m`, ignP[0], ignP[1] + 1); else hide("ign");
    if (tdRing.visible) place("td", "plan", `điểm chạm dự báo · dốc ${fmt(-s.slopeTd * R2D, 0)}°`, s.xTd, groundY(s.xTd) + 2.5, -3); else hide("td");
    const fp = RUN.freePath && RUN.freePath[RUN.freePath.length - 1];
    if (freeLine.visible && fp) place("free", "pred", `rơi tự do · v⊥ ≈ ${fmt(RUN.m.freeVn, 0)} m/s`, fp[0], groundY(fp[0]) + 3); else hide("free");
    if (s.phase === 'BURN' && opt.ctrl) place("burn", "plan", `JET KHÍ LẠNH · ${fmt(s.thr * P.thrTW, 2)}× trọng lượng`, s.x, s.y - 3.4); else hide("burn");

    // toast kết quả
    const tr = $("toast");
    if (res) {
      const key = res.grade + RUN.tMax;
      if (tr.hidden || tr.dataset.k !== key) {
        const lab = { SAFE: "Đáp an toàn", HARD: "Đáp cứng", CRASH: "Đâm / lật" }[res.grade];
        const c = { SAFE: "var(--green)", HARD: "var(--gold)", CRASH: "var(--red)" }[res.grade];
        let why = "";
        if (res.grade !== 'SAFE' && opt.ctrl) {
          const rs = [];
          if (RUN.m.gas <= 0) rs.push("hết khí nén trước khi chạm");
          if (RUN.m.drop0 < 40) rs.push(`chỉ ~${fmt(RUN.m.drop0, 0)} m độ cao, không đủ quãng hãm`);
          if (!rs.length) rs.push("lực đẩy dự phòng không đủ bù sai số cảm biến / gió");
          why = `<div class="why">Nguyên nhân: ${rs.join("; ")}.</div>`;
        }
        tr.style.borderColor = c;
        tr.innerHTML = `<h3 style="color:${c}">${lab}</h3><div class="m"><span>v⊥ ${fmt(res.vn, 2)} m/s</span><span>Δpitch ${fmt(res.pe, 1)}°</span><span>lật ngang ${fmt(res.rollDeg, 1)}°</span><span>người ${fmt(res.gLoad, 1)} g · nguy cơ ${occRisk(res.gLoad)}</span><span>khí còn ${fmt(RUN.m.gas * 100, 0)}%</span></div>
          ${RUN.m.freePath ? `<div class="why" style="color:var(--red)">Dự đoán nếu không cứu hộ: v⊥ ${fmt(RUN.m.freeVn, 0)} m/s · người ~${fmt(RUN.m.freeG, 0)} g (${occRisk(RUN.m.freeG)})</div>` : ""}${why}`;
        tr.dataset.k = key; tr.hidden = false;
      }
    } else tr.hidden = true;

    CH[chap].read.forEach(([, fn], j) => { const el = $("r" + j); if (el) el.textContent = fn(s); });
    updateOccupant(s, res);
    const evs = RUN.events.filter((e) => e.t <= s.t + 1e-6);
    if (evs.length !== logCount && $("log")) { logCount = evs.length; $("log").innerHTML = evs.map((e) => `<div><span class="t">${fmt(e.t, 2)}s</span>${e.msg}</div>`).join("") || '<span class="note">Chưa có sự kiện.</span>'; }
    $("time").value = t; $("tread").textContent = fmt(t, 1) + " s";
  }

  // ============================================================ người trong xe: mô tả trạng thái theo pha + tải hiện tại
  function occText(s, res) {
    const g = Math.hypot(s.gx, s.gz) + s.gHead, wd = Math.abs(s.w * R2D);
    if (res) {
      const r = occRisk(res.gLoad);
      return [res.gLoad < 10 ? "green" : res.gLoad < 20 ? "gold" : "red",
        `Chạm đất: tải đỉnh <b>${fmt(res.gLoad, 1)} g</b> trong tích tắc, tương đương ${res.gLoad < 2 ? "một ổ gà lớn" : res.gLoad < 5 ? "phanh gấp / va chạm nhẹ" : res.gLoad < 20 ? "va chạm giao thông vừa" : "tai nạn rất nặng"}. Nguy cơ chấn thương: <b>${r}</b>${res.gLoad > 4 ? "; túi khí bung" : ""}. Cả quá trình, tải đỉnh ${fmt(s.gPeak, 1)} g.`];
    }
    switch (s.phase) {
      case 'DRIVE': return ["gold", `Ngồi ghế, dây an toàn căng. Xe lao dốc mất phanh, chưa có tải bất thường (≈1 g).`];
      case 'FREEFALL': return ["gold", `<b>Mất trọng lượng</b> (≈${fmt(g, 2)} g): người nhấc khỏi ghế, bị dây an toàn giữ lại; đồ lỏng trong xe trôi nổi. Xe đang lộn ${fmt(wd, 0)}°/s, rất dễ mất định hướng.`];
      case 'RIGHTING': return [s.gHead > 2 ? "red" : "gold", `Đang bị lộn ${fmt(wd, 0)}°/s và lật ngang ${fmt(Math.abs(s.p * R2D), 0)}°/s; người bị dây an toàn giữ nhưng đầu lắc sang hai bên, chịu ly tâm ${fmt(s.gHead, 2)} g, chóng mặt, mất phương hướng. Tải tổng còn thấp. Jet chéo + jet hông đang dập xoay.`];
      case 'ARMED': return ["green", `Xe đã đứng thẳng, gần như không trọng lượng (≈${fmt(g, 2)} g); dây an toàn giữ người áp lưng ghế, chờ jet kích hoạt.`];
      case 'BURN': return [s.gz > 4 ? "gold" : "green", `Jet hãm: người bị <b>ép xuống ghế ${fmt(s.gz, 1)} g</b> dọc cột sống trong ${fmt(s.t - RUN.chapStart[4], 1)} s, giống cú thắng gấp kéo dài; chịu được với người khoẻ có ghế đỡ lưng.`];
      case 'CRASH': return ["red", `Va chạm rất mạnh.`];
      default: return ["green", `Xe đang lăn bánh và giảm tốc.`];
    }
  }
  function updateOccupant(s, res) {
    const st = $("occState"); if (!st) return;
    const [col, txt] = opt.ctrl ? occText(s, res) : ["red", res ? `Không có cứu hộ: tải đỉnh <b>${fmt(res.gLoad, 0)} g</b> → ${occRisk(res.gLoad)}.` : occText(s, null)[1]];
    st.innerHTML = txt; $("occDot").style.background = `var(--${col})`;
    const g = Math.hypot(s.gx, s.gz) + s.gHead;
    $("occGz").textContent = fmt(s.gz, 2) + " g"; $("occGx").textContent = fmt(s.gx, 2) + " g"; $("occGh").textContent = fmt(s.gHead, 2) + " g";
    $("occPk").textContent = fmt(res ? Math.max(s.gPeak, res.gLoad) : s.gPeak, 1) + " g";
    const bar = $("occBar"); bar.style.width = clamp((res ? res.gLoad : g) / 10, 0, 1) * 100 + "%"; bar.style.background = `var(--${col})`;
  }

  // ============================================================ transport
  const setPlay = (on) => { playing = on; $("play").textContent = on ? "❚❚" : "▶"; };
  $("play").onclick = () => { if (t >= RUN.tMax - 1e-3) t = 0; setPlay(!playing); };
  $("restart").onclick = () => { t = 0; userSeek = false; setPlay(true); };
  const SPEEDS = [0.25, 0.5, 1, 2, 4];
  $("speed").onclick = () => { speed = SPEEDS[(SPEEDS.indexOf(speed) + 1) % SPEEDS.length]; $("speed").textContent = String(speed).replace(".", ",") + "×"; };
  $("time").oninput = (e) => { t = +e.target.value; };
  $("camSel").onchange = (e) => setCamMode(e.target.value);
  $("pipBtn").onclick = () => { pipOn = !pipOn; $("pipBtn").setAttribute("aria-pressed", String(pipOn)); layoutPip(); };
  $("themeBtn").onclick = () => {
    const cur = document.documentElement.getAttribute("data-theme") || (window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
    document.documentElement.setAttribute("data-theme", cur === "dark" ? "light" : "dark"); applyPal();
  };
  window.addEventListener("keydown", (e) => {
    if (e.target.closest && e.target.closest("input,select")) return;
    if (e.code === "Space") { e.preventDefault(); $("play").click(); }
    if (e.key === "ArrowRight") goChapter(chap + 1, true);
    if (e.key === "ArrowLeft") goChapter(chap - 1, true);
    if (e.key === "c" || e.key === "C") { const o = [...$("camSel").options].map((x) => x.value); setCamMode(o[(o.indexOf(camMode) + 1) % o.length]); }
  });
  if (window.matchMedia) window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyPal);

  function resize() {
    const r = stage.getBoundingClientRect(); renderer.setSize(r.width, r.height, false);
    prepMain(); layoutPip();
  }
  window.addEventListener("resize", resize);

  let last = performance.now(), plotT = 0;
  function frame(now) {
    const dtReal = clamp((now - last) / 1000, 0, 0.05); last = now;
    let dtSim = 0;
    if (playing) {
      const s = RUN.F[clamp(Math.floor(t / FRAME_DT), 0, RUN.F.length - 1)].m;
      dtSim = dtReal * speed * (opt.slowmo && s.phase === 'BURN' ? 0.35 : 1);
      t += dtSim;
      if (t >= RUN.tMax) { t = RUN.tMax; setPlay(false); }
    }
    update(dtReal, dtSim);
    renderViews();
    if ((plotT += dtReal) > 0.1) { plotT = 0; drawPlot(); }
    requestAnimationFrame(frame);
  }

  window.__demo = { get t() { return t; }, set t(v) { t = v; }, goChapter, opt, rebuild, setPlay, setCamMode };
  resize(); applyPal(); rebuild();
  setPlay(!REDUCED);
  requestAnimationFrame(frame);
})();

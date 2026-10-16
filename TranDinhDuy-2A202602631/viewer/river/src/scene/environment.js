// Existing approach road and T junction, surrounded by a deeper mountain canyon.
export function createEnvironment(T,scene) {
  var palette = { grass: 0x698773, grass2: 0x7f997a, rock: 0x707e76, soil: 0x5c6c60,
    road: 0x343f43, line: 0xe1d9bf, teal: 0x79e3bf, gold: 0xe9b777, red: 0xd96448,
    water: 0x397f88, bark: 0x55604a, leaf: 0x3c6857, roof: 0x587178, wall: 0xb5b4a0 };
  var mats = {};
  Object.keys(palette).forEach(function (key) { mats[key] = new T.MeshStandardMaterial({ color: palette[key], roughness: 0.86 }); });
  mats.water = new T.MeshStandardMaterial({ color: palette.water, roughness: 0.3, metalness: 0.28, transparent: true, opacity: 0.92 });
  var world = new T.Group(); scene.add(world);
  var water, targetRing;
  var boxGeo = new T.BoxGeometry(1, 1, 1);
  function box(parent, material, x, y, z, sx, sy, sz) {
    var mesh = new T.Mesh(boxGeo, material); mesh.position.set(x, y, z); mesh.scale.set(sx, sy, sz);
    mesh.castShadow = true; mesh.receiveShadow = true; parent.add(mesh); return mesh;
  }
  function line(points, color, parent, dashed) {
    var geometry = new T.BufferGeometry().setFromPoints(points.map(function (p) { return new T.Vector3(p[0], p[1], p[2]); }));
    var material = dashed ? new T.LineDashedMaterial({ color: color, dashSize: 1.1, gapSize: 0.9, transparent: true, opacity: 0.65 }) : new T.LineBasicMaterial({ color: color });
    var mesh = new T.Line(geometry, material); if (dashed) mesh.computeLineDistances(); parent.add(mesh); return mesh;
  }
  function label(text, x, y, z, width, parent, bg) {
    var canvas = document.createElement("canvas"); canvas.width = 512; canvas.height = 128;
    var ctx = canvas.getContext("2d"); ctx.fillStyle = bg || "#183a3d"; ctx.fillRect(0, 0, 512, 128);
    ctx.strokeStyle = "#b9d9c0"; ctx.lineWidth = 5; ctx.strokeRect(7, 7, 498, 114);
    ctx.font = "bold 41px Segoe UI, sans-serif"; ctx.fillStyle = "#e7edda";
    ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, 256, 64);
    var texture = new T.CanvasTexture(canvas); texture.encoding = T.sRGBEncoding;
    var material = new T.MeshBasicMaterial({ map: texture, side: T.DoubleSide });
    var mesh = new T.Mesh(new T.PlaneGeometry(width, width / 4), material);
    mesh.position.set(x, y, z); mesh.rotation.y = -Math.PI / 2; parent.add(mesh); return mesh;
  }
  function tree(x, z, scale) {
    var g = new T.Group(); g.position.set(x, 0, z); g.scale.setScalar(scale);
    box(g, mats.bark, 0, 1.7, 0, 0.4, 3.4, 0.4);
    [2.8, 4.2, 5.4].forEach(function (y, i) {
      var mesh = new T.Mesh(new T.ConeGeometry(2.1 - i * 0.4, 3.4, 7), mats.leaf);
      mesh.position.y = y; mesh.castShadow = true; g.add(mesh);
    }); world.add(g);
  }
  function house(x, z, size) {
    var g = new T.Group(); g.position.set(x, 0, z);
    box(g, mats.wall, 0, 2.4, 0, 7 * size, 4.8, 5.5 * size);
    var roof = new T.Mesh(new T.CylinderGeometry(0, 5.2 * size, 3, 4), mats.roof);
    roof.rotation.y = Math.PI / 4; roof.scale.z = 0.8; roof.position.y = 6; roof.castShadow = true; g.add(roof);
    box(g, mats.road, -1, 2.3, 2.78 * size, 1.5, 1.8, 0.03);
    box(g, mats.road, 1.7, 2, 2.78 * size, 1.1, 2.8, 0.03); world.add(g);
  }
  function disposeWorld() {
    var geometries = new Set(), materials = new Set();
    world.traverse(function (node) {
      if (node.geometry && node.geometry !== boxGeo) geometries.add(node.geometry);
      if (node.material && Object.values(mats).indexOf(node.material) < 0) materials.add(node.material);
    });
    geometries.forEach(function (g) { g.dispose(); });
    materials.forEach(function (m) { if (m.map) m.map.dispose(); m.dispose(); });
    world.clear();
  }
  function buildWorld(c) {
    disposeWorld(); var width = c.riverWidth;
    // The approach ends at the cross road: a T junction, with a river immediately behind it.
    box(world, mats.soil, -95, c.waterY/2-.24, 0, 190, -c.waterY, 200);
    box(world, mats.soil, width + 95, c.waterY/2-.24, 0, 190, -c.waterY, 200);
    box(world, mats.grass, -95, -0.12, 0, 190, 0.24, 200);
    box(world, mats.grass2, width + 95, -0.12, 0, 190, 0.24, 200);
    box(world, mats.road, -75, 0.025, 0, 126, 0.05, 11);
    box(world, mats.road, -12, 0.035, 0, 11, 0.06, 126);
    box(world, mats.road, width + 43, 0.025, 0, 76, 0.05, 11);
    for (var x = -132; x < -20; x += 8) box(world, mats.line, x, 0.07, 0, 4, 0.025, 0.13);
    for (var z = -59; z < 62; z += 8) if (Math.abs(z) > 9) box(world, mats.line, -12, 0.08, z, 0.13, 0.025, 4);
    [-5.1, 5.1].forEach(function (z) { box(world, mats.line, -76, 0.07, z, 123, 0.025, 0.1); });
    box(world, mats.line, -19, 0.075, 0, 0.4, 0.03, 9.9);
    // Lane arrow directing traffic left/right rather than toward the river.
    line([[-30, .09, 0], [-26, .09, 0], [-26, .09, -3]], 0xe1d9bf, world);
    line([[-28, .09, -1.5], [-26, .09, -3], [-24, .09, -1.5]], 0xe1d9bf, world);
    var wg = new T.PlaneGeometry(width, 230, 28, 90); wg.rotateX(-Math.PI / 2);
    water = new T.Mesh(wg, mats.water); water.position.set(width / 2, c.waterY, 0); water.receiveShadow = true; world.add(water);
    // Exposed rocky banks, warning posts, and a visible break in the river barrier.
    [0, width].forEach(function (bank, side) {
      for (var z = -94; z <= 95; z += 5.5) {
        var rock = new T.Mesh(new T.DodecahedronGeometry(1.3, 0), mats.rock);
        rock.position.set(bank + (side ? .5 : -.5), -.7, z); rock.scale.set(1, 1.4, 1.6); world.add(rock);
      }
    });
    for (var z = -58; z <= 58; z += 5) if (Math.abs(z) > 6) {
      box(world, mats.rock, -1.6, .65, z, .22, 1.3, .22);
      box(world, mats.gold, -1.6, 1.08, z + 2, .18, .21, 4.2);
    }
    [-6.8, 6.8].forEach(function (z) {
      box(world, mats.red, -2, .65, z, .45, 1.3, .45);
      box(world, mats.line, -2, .8, z, .46, .23, .46);
    });
    box(world, mats.rock, -42, 2.2, -7.8, .16, 4.4, .16);
    label("NGÃ BA  /  GIẢM TỐC", -42, 4.1, -7.8, 8, world);
    box(world, mats.rock, -3.3, 1.8, -8.5, .15, 3.6, .15);
    label("VỰC SÂU / HẾT ĐƯỜNG", -3.3, 3.4, -8.5, 7.8, world, "#884832");
    for (var level = -3; level > c.waterY; level -= 3) {
      box(world, level % 2 ? mats.rock : mats.soil, -.12, level, 0, .28, .35, 200);
      box(world, level % 2 ? mats.rock : mats.soil, width+.12, level, 0, .28, .35, 200);
    }
    var targetX = width + 22, targetZ = c.landingOffset || 0;
    box(world, mats.road, targetX, .055, targetZ, 20, .07, 11);
    targetRing = new T.Mesh(new T.RingGeometry(3.8, 3.94, 64), new T.MeshBasicMaterial({ color: 0x99edc8, side: T.DoubleSide }));
    targetRing.rotation.x = -Math.PI / 2; targetRing.position.set(targetX, .105, targetZ); world.add(targetRing);
    line([[targetX - 9, .11, targetZ-5], [targetX + 9, .11, targetZ-5], [targetX + 9, .11, targetZ+5], [targetX - 9, .11, targetZ+5], [targetX - 9, .11, targetZ-5]], 0x99edc8, world, true);
    box(world, mats.teal, targetX - 1, .12, targetZ, .3, .03, 3);
    box(world, mats.teal, targetX + 1, .12, targetZ, .3, .03, 3);
    box(world, mats.teal, targetX, .12, targetZ, 2, .03, .3);
    // A quiet riverside settlement gives the scene scale and a sense of place.
    [[-92,-27,1],[-119,33,.9],[-59,34,1.1],[width+65,-31,1.1],[width+86,-49,.9],[width+74,32,1]].forEach(function(p){house(p[0],p[1],p[2]);});
    for (var i = 0; i < 55; i++) {
      var side = i % 2, x = side ? width + 8 + ((i * 37) % 125) : -8 - ((i * 31) % 140);
      var z = ((i * 47) % 166) - 83;
      if (Math.abs(z) < 15 || (x > -22 && x < -3)) continue;
      tree(x, z, .7 + (i % 5) * .13);
    }
    for (var i = 0; i < 15; i++) {
      var mountain = new T.Mesh(new T.ConeGeometry(42 + i % 3 * 17, 45 + i % 4 * 18, 5), mats.grass);
      mountain.position.set(-210 + i * 34, 10, -115 - i % 3 * 16); mountain.rotation.y = i; world.add(mountain);
    }
    var boat = new T.Group(); boat.position.set(width * .5, c.waterY + .15, -48);
    box(boat, mats.wall, 0, 0, 0, 2.2, .6, 5.5); box(boat, mats.roof, 0, .45, -1, 1.6, .6, 1.8); world.add(boat);
  }

  return { world, build:buildWorld, dispose:disposeWorld,
    update(time){
      if(!water)return;
      var wave=water.geometry.attributes.position;
      for(var i=0;i<wave.count;i++)wave.setY(i,.06*Math.sin(wave.getX(i)*.65+time*1.5)+.04*Math.sin(wave.getZ(i)*.5+time*2));
      wave.needsUpdate=true;water.geometry.computeVertexNormals();
    }
  };
}

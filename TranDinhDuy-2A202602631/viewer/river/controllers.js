/* Pluggable navigation adapters. The renderer never owns control decisions. */
import * as C from './core.js';
  "use strict";
  function MockController() { this.name = "Điều hướng mock"; }
  MockController.prototype.reset = function () {};
  MockController.prototype.update = C.mockCommand;
  function SocketController(url, onStatus, bindings = {}) {
    this.WebSocket = bindings.WebSocket ?? globalThis.WebSocket;
    this.now = bindings.now ?? (() => performance.now());
    this.name = "WebSocket"; this.url = url; this.onStatus = onStatus || function () {};
    this.socket = null; this.latest = null; this.pending = null; this.runId = null;
    this.lastSent = -Infinity; this.closed = false; this.generation = 0;
  }
  SocketController.prototype.connect = function () {
    var self = this, parsed = new URL(this.url);
    if (parsed.protocol !== "ws:" && parsed.protocol !== "wss:") throw new Error("Địa chỉ phải bắt đầu bằng ws:// hoặc wss://");
    this.closed = false;
    this.socket = new this.WebSocket(this.url);
    this.onStatus("Đang kết nối…");
    this.socket.addEventListener("open", function () { if (!self.closed) self.onStatus("Đã nối · chờ lệnh"); });
    this.socket.addEventListener("close", function () { self.latest = null; if (!self.closed) self.onStatus("Mất kết nối · đã tạm dừng"); });
    this.socket.addEventListener("error", function () { if (!self.closed) self.onStatus("Không nối được bộ điều hướng"); });
    this.socket.addEventListener("message", function (event) {
      if (self.closed) return;
      try {
        var d = JSON.parse(event.data), p = self.pending;
        if (d.schema !== "river-nav/1" || d.type !== "command" || d.runId !== self.runId ||
          !p || d.seq !== p.seq || p.generation !== self.generation) return;
        var controls = C.normalizeCommand(d.controls);
        self.latest = { controls: controls, at: self.now() };
        self.pending = null; self.onStatus("Đã nối · đang nhận lệnh");
      } catch (_) { self.onStatus("Lệnh không hợp lệ · chờ lệnh mới"); }
    });
  };
  SocketController.prototype.reset = function (observation) {
    this.runId = observation.runId; this.latest = null; this.pending = null;
    this.lastSent = -Infinity; this.generation++;
  };
  SocketController.prototype.update = function (observation) {
    var now = this.now(), ws = this.socket;
    if (this.pending && now - this.pending.at > 1000) this.pending = null;
    // One outstanding request, 20 Hz max. Responses must echo the exact request/run.
    if (ws && ws.readyState === 1 && !this.pending && now - this.lastSent >= 50) {
      this.pending = { seq: observation.seq, at: now, generation: this.generation };
      this.lastSent = now; ws.send(JSON.stringify(observation));
    }
    if (!ws || ws.readyState !== 1 || !this.latest || now - this.latest.at > 500) return null;
    return this.latest.controls;
  };
  SocketController.prototype.dispose = function () {
    this.closed = true; this.latest = null; this.pending = null;
    if (this.socket) this.socket.close();
  };
export { MockController, SocketController };

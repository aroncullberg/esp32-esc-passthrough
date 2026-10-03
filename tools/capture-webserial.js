// SPDX-License-Identifier: GPL-3.0-or-later
// Run in Chrome DevTools before connecting ESC Configurator.
// Save with escSerialTrace.download().
// Stop with escSerialTrace.stop().
(() => {
  "use strict";

  if (!globalThis.navigator?.serial || !globalThis.SerialPort) {
    throw new Error("Web Serial is unavailable in this page/context.");
  }
  if (globalThis.escSerialTrace?.active) {
    console.info("ESC serial capture is already active.");
    return;
  }

  const events = [];
  const restores = [];
  const portIds = new WeakMap();
  const readers = new WeakSet();
  const writers = new WeakSet();
  const streams = new WeakSet();
  const startedAt = new Date().toISOString();
  const origin = performance.now();
  const maxEvents = 20000;
  let nextPortId = 1;
  let active = true;
  let droppedEvents = 0;

  function portId(port) {
    if (!portIds.has(port)) portIds.set(port, nextPortId++);
    return portIds.get(port);
  }

  function record(port, kind, details = {}) {
    if (!active) return;
    if (events.length >= maxEvents) {
      droppedEvents++;
      return;
    }
    events.push({
      tMs: Number((performance.now() - origin).toFixed(3)),
      port: portId(port),
      kind,
      ...details,
    });
  }

  function bytes(value) {
    if (ArrayBuffer.isView(value)) {
      return Array.from(new Uint8Array(value.buffer, value.byteOffset, value.byteLength));
    }
    if (value instanceof ArrayBuffer) return Array.from(new Uint8Array(value));
    throw new TypeError("Expected a binary Web Serial chunk.");
  }

  // Capture failures must not prevent the application's serial operation.
  function capture(port, kind, value) {
    try {
      if (active && value !== undefined) record(port, kind, { bytes: bytes(value) });
    } catch (error) {
      record(port, "capture-error", { message: String(error) });
    }
  }

  function patch(object, name, replacement) {
    const own = Object.getOwnPropertyDescriptor(object, name);
    Object.defineProperty(object, name, {
      value: replacement, configurable: true, writable: true,
    });
    restores.push(() => {
      // Leave a subsequent application's replacement intact.
      if (object[name] !== replacement) return;
      if (own) Object.defineProperty(object, name, own);
      else delete object[name];
    });
  }

  function instrument(port) {
    const readable = port.readable;
    if (readable && !streams.has(readable)) {
      streams.add(readable);
      const getReader = readable.getReader;
      patch(readable, "getReader", function (...args) {
        const reader = Reflect.apply(getReader, this, args);
        if (!readers.has(reader)) {
          readers.add(reader);
          const read = reader.read;
          patch(reader, "read", function (...readArgs) {
            return Reflect.apply(read, this, readArgs).then(result => {
              capture(port, "rx", result.value);
              if (result.done) record(port, "read-done");
              return result;
            });
          });
        }
        return reader;
      });
    }
    const writable = port.writable;
    if (writable && !streams.has(writable)) {
      streams.add(writable);
      const getWriter = writable.getWriter;
      patch(writable, "getWriter", function (...args) {
        const writer = Reflect.apply(getWriter, this, args);
        if (!writers.has(writer)) {
          writers.add(writer);
          const write = writer.write;
          patch(writer, "write", function (chunk) {
            // TX (write attempt) != delivery
            capture(port, "tx", chunk);
            return Reflect.apply(write, this, [chunk]);
          });
        }
        return writer;
      });
    }
  }

  const prototype = SerialPort.prototype;
  const open = prototype.open;
  patch(prototype, "open", async function (options) {
    record(this, "open", { info: this.getInfo(), options: { ...options } });
    await Reflect.apply(open, this, [options]);
    if (!active) return;
    try {
      instrument(this);
      record(this, "opened");
    } catch (error) {
      record(this, "capture-error", { message: String(error) });
      console.warn("Could not attach serial capture; reload before retrying.", error);
    }
  });

  const close = prototype.close;
  patch(prototype, "close", function (...args) {
    record(this, "close");
    return Reflect.apply(close, this, args);
  });
  const setSignals = prototype.setSignals;
  if (setSignals) {
    patch(prototype, "setSignals", function (signals) {
      record(this, "signals", { signals: { ...signals } });
      return Reflect.apply(setSignals, this, [signals]);
    });
  }

  globalThis.escSerialTrace = {
    get active() { return active; },
    events,
    clear() { events.length = 0; droppedEvents = 0; },
    json() {
      return JSON.stringify({
        format: "esc-webserial-trace-v1",
        url: location.href,
        userAgent: navigator.userAgent,
        startedAt,
        droppedEvents,
        events,
      }, null, 2);
    },
    download(filename = "esc-serial-trace.json") {
      const blob = new Blob([this.json()], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement("a");
      anchor.href = url;
      anchor.download = filename;
      anchor.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      if (droppedEvents) console.warn("Trace was truncated:", droppedEvents, "dropped events.");
    },
    stop() {
      active = false;
      for (const restore of restores.reverse()) restore();
      console.info("ESC serial capture stopped; recorded events remain available.");
    },
  };

  console.info("ESC serial capture ready, run escSerialTrace.download() when done.");
})();

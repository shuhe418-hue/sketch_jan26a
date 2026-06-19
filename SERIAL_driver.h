#ifndef SERIAL_DRV_H
#define SERIAL_DRV_H

#include <Arduino.h>

// ==========================
// Serial Driver (TX/RX)
// ==========================
//
// - RX: line-based parser (\n or \r\n)
// - TX: send / sendLine helpers
// - User provides callbacks for received line and errors
//

typedef void (*SerialLineCallback)(const char* line, void* user);
typedef void (*SerialErrorCallback)(const char* err, void* user);

struct SerialDrvConfig {
  uint32_t baud = 115200;
  size_t   rx_max_len = 96;      // max line length (excluding '\0')
  bool     echo_rx = false;      // if true, echo received lines back
  bool     trim_space = true;    // trim leading/trailing spaces for callback
  bool     accept_empty = false; // deliver empty line or ignore
};

class SerialDrv {
public:
  SerialDrv();

  // Bind to a specific Stream (Serial / Serial1 / HWCDCSerial / etc.)
  // NOTE: Stream does not have a uniform begin(baud); call Serial.begin(...) in your .ino.
  void begin(Stream& port, const SerialDrvConfig& cfg);

  // Must be called repeatedly in loop()
  void poll();

  // TX helpers
  void send(const char* s);
  void send(const String& s);
  void sendLine(const char* s);
  void sendLine(const String& s);
  void printf(const char* fmt, ...);

  // callbacks
  void onLine(SerialLineCallback cb, void* user = nullptr);
  void onError(SerialErrorCallback cb, void* user = nullptr);

  // state
  bool ready() const { return _port != nullptr; }
  uint32_t baud() const { return _cfg.baud; }

private:
  Stream* _port;           // <-- changed from HardwareSerial* to Stream*
  SerialDrvConfig _cfg;

  // rx buffer
  char*  _rx_buf;
  size_t _rx_len;

  // callbacks
  SerialLineCallback  _cb_line;
  void*               _cb_line_user;
  SerialErrorCallback _cb_err;
  void*               _cb_err_user;

  // helpers
  void _emit_error(const char* msg);
  void _emit_line(char* line);
  void _reset_line();

  // trim in-place
  static char* _trim_inplace(char* s);
};

#endif // SERIAL_DRV_H


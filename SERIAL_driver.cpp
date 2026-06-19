#include "SERIAL_driver.h"
#include <stdarg.h>

SerialDrv::SerialDrv()
: _port(nullptr),
  _rx_buf(nullptr),
  _rx_len(0),
  _cb_line(nullptr),
  _cb_line_user(nullptr),
  _cb_err(nullptr),
  _cb_err_user(nullptr) {}

void SerialDrv::begin(Stream& port, const SerialDrvConfig& cfg) {
  _port = &port;
  _cfg = cfg;

  // (re)alloc buffer
  if (_rx_buf) {
    free(_rx_buf);
    _rx_buf = nullptr;
  }
  _rx_buf = (char*)malloc(_cfg.rx_max_len + 1);
  _rx_len = 0;

  if (!_rx_buf) {
    _emit_error("RX buffer alloc failed");
    return;
  }

  // IMPORTANT:
  // Stream does NOT guarantee begin(baud). Do Serial.begin(cfg.baud) in .ino setup().
  _reset_line();
}

void SerialDrv::onLine(SerialLineCallback cb, void* user) {
  _cb_line = cb;
  _cb_line_user = user;
}

void SerialDrv::onError(SerialErrorCallback cb, void* user) {
  _cb_err = cb;
  _cb_err_user = user;
}

void SerialDrv::poll() {
  if (!_port || !_rx_buf) return;

  while (_port->available() > 0) {
    char c = (char)_port->read();

    // ignore '\r' (supports \r\n)
    if (c == '\r') continue;

    if (c == '\n') {
      _rx_buf[_rx_len] = '\0';

      // deliver
      if (_cfg.trim_space) {
        char* t = _trim_inplace(_rx_buf);
        if (t != _rx_buf) {
          size_t n = strlen(t);
          memmove(_rx_buf, t, n + 1);
        }
      }

      if (_cfg.accept_empty || _rx_buf[0] != '\0') {
        if (_cfg.echo_rx) {
          sendLine(String("[ECHO] ") + _rx_buf);
        }
        _emit_line(_rx_buf);
      }

      _reset_line();
      continue;
    }

    // normal char
    if (_rx_len < _cfg.rx_max_len) {
      _rx_buf[_rx_len++] = c;
    } else {
      // overflow: drop this line until newline
      _emit_error("RX line too long (dropped)");
      _reset_line();
    }
  }
}

void SerialDrv::send(const char* s) {
  if (!_port || !s) return;
  _port->print(s);
}

void SerialDrv::send(const String& s) {
  if (!_port) return;
  _port->print(s);
}

void SerialDrv::sendLine(const char* s) {
  if (!_port) return;
  if (s) _port->print(s);
  _port->print('\n');
}

void SerialDrv::sendLine(const String& s) {
  if (!_port) return;
  _port->print(s);
  _port->print('\n');
}

void SerialDrv::printf(const char* fmt, ...) {
  if (!_port || !fmt) return;

  char buf[160];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n > 0) _port->print(buf);
}

void SerialDrv::_emit_error(const char* msg) {
  if (_cb_err) {
    _cb_err(msg, _cb_err_user);
  } else if (_port) {
    _port->print("ERR ");
    _port->println(msg ? msg : "");
  }
}

void SerialDrv::_emit_line(char* line) {
  if (_cb_line) {
    _cb_line((const char*)line, _cb_line_user);
  }
}

void SerialDrv::_reset_line() {
  _rx_len = 0;
  if (_rx_buf) _rx_buf[0] = '\0';
}

char* SerialDrv::_trim_inplace(char* s) {
  if (!s) return s;

  // leading
  while (*s == ' ' || *s == '\t') s++;

  // trailing
  char* end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t')) {
    end[-1] = '\0';
    end--;
  }
  return s;
}

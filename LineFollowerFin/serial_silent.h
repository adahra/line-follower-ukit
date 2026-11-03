#ifndef SERIAL_SILENT_H
#define SERIAL_SILENT_H

#include <Arduino.h>

#if !ENABLE_SERIAL_OUTPUT

// Override Serial dengan dummy object agar semua panggilan benar-benar dihapus
class SerialDummy {
public:
  void begin(...) {}
  void end() {}
  int available() { return 0; }
  int availableForWrite() { return 0; }
  void flush() {}
  int peek() { return -1; }
  int read() { return -1; }
  size_t readBytes(char *, size_t) { return 0; }
  size_t readBytes(uint8_t *, size_t) { return 0; }
  size_t readBytesUntil(char, char *, size_t) { return 0; }
  size_t readBytesUntil(char, uint8_t *, size_t) { return 0; }
  String readString() { return String(""); }
  String readStringUntil(char) { return String(""); }
  
  size_t print(const __FlashStringHelper *) { return 0; }
  size_t print(const String &) { return 0; }
  size_t print(const char[]) { return 0; }
  size_t print(char) { return 0; }
  size_t print(unsigned char, int = DEC) { return 0; }
  size_t print(int, int = DEC) { return 0; }
  size_t print(unsigned int, int = DEC) { return 0; }
  size_t print(long, int = DEC) { return 0; }
  size_t print(unsigned long, int = DEC) { return 0; }
  size_t print(double, int = 2) { return 0; }
  size_t print(const Printable &) { return 0; }
  
  size_t println(const __FlashStringHelper *) { return 0; }
  size_t println(const String &) { return 0; }
  size_t println(const char[]) { return 0; }
  size_t println(char) { return 0; }
  size_t println(unsigned char, int = DEC) { return 0; }
  size_t println(int, int = DEC) { return 0; }
  size_t println(unsigned int, int = DEC) { return 0; }
  size_t println(long, int = DEC) { return 0; }
  size_t println(unsigned long, int = DEC) { return 0; }
  size_t println(double, int = 2) { return 0; }
  size_t println(const Printable &) { return 0; }
  size_t println(void) { return 0; }
  
  size_t write(uint8_t) { return 1; }
  size_t write(const char *str) { if (str) return strlen(str); return 0; }
  size_t write(const uint8_t *buffer, size_t size) { return size; }
  
  operator bool() { return false; }
};

static SerialDummy SerialDummyInstance;
#define Serial SerialDummyInstance

#endif // !ENABLE_SERIAL_OUTPUT

#endif // SERIAL_SILENT_H

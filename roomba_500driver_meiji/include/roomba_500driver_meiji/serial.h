// Copyright 2023 amsl
//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       serial.h
 *
 *
 * Environment  :       g++
 * Latest Update:       2008/07/07
 *
 * Designer(s)  :       m.sanpei (AMSL)
 * Author(s)    :       m.sanpei (AMSL)
 *
 * CopyRight    :       2007, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2007/08/23
 *
 */
//-----------------------------------------------------------------------------

#ifndef ROOMBA_500DRIVER_MEIJI__SERIAL_H_
#define ROOMBA_500DRIVER_MEIJI__SERIAL_H_

#include <termios.h>

#define MODEMDEVICE1 "/dev/ttyS0"
#define MODEMDEVICE_USB0 "/dev/ttyUSB0"
#define MODEMDEVICE_USB1 "/dev/ttyUSB1"
#define _POSIX_SOURCE 1 /* POSIX 準拠のソース */
#define FALSE 0
#define TRUE 1

// #define DEBUG

class Serial
{
private:
  int fd_;  //, c_, res_;
  struct termios oldtio_, newtio_;

public:
  Serial(int baudrate, const char * modemdevice, int vmin = 0, int lflag = 0);
  ~Serial();

  int read(unsigned char * p, int len);
  int write(const unsigned char * p, int len);
  void setVmin(int vmin);  // non canonical 時のreadで待つ最低限の文字数
  void setRts(int);
};

#endif  // ROOMBA_500DRIVER_MEIJI__SERIAL_H_

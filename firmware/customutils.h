#ifndef CUSTOMUTILS_H
#define CUSTOMUTILS_H

inline int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == nullptr ? (int) &__heap_start : (int) __brkval);
}

#endif

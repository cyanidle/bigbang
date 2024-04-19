#ifndef GLOBAL_H
#define GLOBAL_H

#include <ros.h>
#include "customutils.h"

#define SUB_BUFF 1500
#define PUB_BUFF 1500
#define SUB_TOPICS 25
#define PUB_TOPICS 25

extern ros::NodeHandle_<ArduinoHardware, SUB_TOPICS, PUB_TOPICS, SUB_BUFF, PUB_BUFF> nh; // recieve/publish

inline void logInt(const char* msg, int value) {
  char buf[40];
  sprintf(buf, msg, value);
  nh.loginfo(buf);
}
inline void logIntErr(const char* msg, int value) {
  char buf[40];
  sprintf(buf, msg, value);
  nh.logerror(buf);
}
inline void logRam() {
  logInt("Arduino: Free Ram: %d", freeRam());
}
#endif

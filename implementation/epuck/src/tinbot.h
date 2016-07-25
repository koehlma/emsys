#ifndef TINBOT_H
#define TINBOT_H

#include "controller.h"
#include "sensors.h"
#include "moderator.h"
#include "t2t-data.h"

typedef struct TinBot {
    Sensors sens;
    Controller controller;
    T2TData rx_buffer;
    ModState moderator;
    unsigned int mode;
} TinBot;

typedef void (*Setup)(TinBot* tinbot);
typedef void (*Loop)(TinBot* tinbot);

typedef struct TinMode {
    Setup setup;
    Loop loop;
} TinMode;

void setup(TinBot* tinbot);
void loop(TinBot* tinbot);

void set_mode(TinBot* tinbot, unsigned int mode);

void update_proximity(TinBot* tinbot, double proximity[8]);
void update_ir(TinBot* tinbot, int ir[6]);
void update_lps(TinBot* tinbot, double x, double y, double phi);

void update_victim_pickup(TinBot* tinbot, unsigned int grabbed);

void set_victim_attached(TinBot* tinbot, unsigned int value);

#endif

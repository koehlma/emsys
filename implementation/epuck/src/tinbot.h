#ifndef TINBOT_H
#define TINBOT_H

typedef struct TinBot {

} TinBot;


void setup(TinBot* tinbot);
void loop(TinBot* tinbot);

void update_proximity(TinBot* tinbot, double proximity[8]);
void update_ir(TinBot* tinbot, int ir[6]);
void update_lps(TinBot* tinbot, double x, double y, double direction);

void set_victim_attached(TinBot* tinbot, unsigned int value);

#endif


#ifndef EPUCK_PHANDLER_H
#define EPUCK_PHANDLER_H

typedef struct {
    unsigned int party_soft;
    unsigned int party_hard;
} PHandlerInput;

typedef struct {
    unsigned int soft_state;
    unsigned int hard_state;
    hal_time entry;
} PHandlerState;

void phandler_reset(PHandlerState* state);
void phandler_step(PHandlerInput* input, PHandlerState* state);

#endif /* EPUCK_PHANDLER_H */

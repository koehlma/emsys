
#ifndef EPUCK_PHANDLER_H
#define EPUCK_PHANDLER_H

typedef struct {
    int party_soft;
    int party_hard;
} PHandlerInput;

typedef struct {
    int soft_state;
    int hard_state;
} PHandlerState;

void phandler_reset(PHandlerState* state);
void phandler_step(PHandlerInput* input, PHandlerState* state);

#endif /* EPUCK_PHANDLER_H */

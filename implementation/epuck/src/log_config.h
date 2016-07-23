#ifndef EPUCK_LOG_CONFIG_H
#define EPUCK_LOG_CONFIG_H

/* If we end up having too many debug messages (either because
 * Bluetooth is too busy or we're just annoyed by the visuals), just turn
 * off any debug outputs here: */

#define LOG_BELLMAN_FORD
#define LOG_TRANSITIONS_BLIND_COP
#define LOG_TRANSITIONS_COP_EYES
#define LOG_TRANSITIONS_MOD
#define LOG_TRANSITIONS_PATH_EXEC
#define LOG_TRANSITIONS_VICDIR

/* "Expensive" as in "you're going to need a lot of space for this,
 * and building for the PIC might fail." */
/* #define LOG_EXPENSIVE_PATH_FINDER */

#endif

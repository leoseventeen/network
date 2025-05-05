/* sr.h */
#ifndef SR_H
#define SR_H

#include "emulator.h"

/* Sender-side API */
void A_init(void);
void A_output(struct msg message);
void A_input(struct pkt packet);
void A_timerinterrupt(void);

/* Receiver-side API */
void B_init(void);
void B_input(struct pkt packet);
void B_output(struct msg message);
void B_timerinterrupt(void);

#endif /* SR_H */

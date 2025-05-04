#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  for (int i = 0; i < 20; i++) checksum += (int)packet.payload[i];
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}


/********* Sender (A) variables and functions ************/

/* Sender (A) state for Selective Repeat */
static struct pkt  A_window[WINDOWSIZE];    /* buffered packets */
static bool        A_acked[WINDOWSIZE];     /* ack status */
static bool        A_timer_running[WINDOWSIZE];
static int         A_base;
static int         A_nextseq;

/* Receiver (B) state for Selective Repeat */
static struct pkt  B_buffer[WINDOWSIZE];
static bool        B_received[WINDOWSIZE];
static int         B_base;
static int         B_nextseq;


/* called from layer 5 (application layer), passed the message to be sent to other side */
/* Called from layer 5: send message if window not full */
void A_output(struct msg message) {
  if (A_nextseq < A_base + WINDOWSIZE) {
      int seq = A_nextseq % WINDOWSIZE;
      struct pkt p;
      p.seqnum = A_nextseq;
      p.acknum = NOTINUSE;
      memcpy(p.payload, message.data, 20);
      p.checksum = ComputeChecksum(p);

      A_window[seq] = p;
      A_acked[seq] = false;
      tolayer3(A, p);

      if (!A_timer_running[seq]) {
          starttimer(A, RTT);
          A_timer_running[seq] = true;
      }
      A_nextseq++;
  } else {
      /* window full */
      if (TRACE > 0) printf("----A: window full, dropping message\n");
      window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet) {
  if (!IsCorrupted(packet)
      && packet.acknum >= A_base
      && packet.acknum < A_nextseq) {
      int idx = packet.acknum % WINDOWSIZE;
      if (!A_acked[idx]) {
          A_acked[idx] = true;
          total_ACKs_received++;
          /* stop timer for this packet */
          if (A_timer_running[idx]) {
              stoptimer(A);
              A_timer_running[idx] = false;
          }
          /* slide window base forward */
          while (A_acked[A_base % WINDOWSIZE]) {
              A_acked[A_base % WINDOWSIZE] = false;
              A_base++;
          }
      }
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void) {
  if (TRACE > 0) printf("----A: timeout, resending unacked packets\n");
  for (int seqnum = A_base; seqnum < A_nextseq; seqnum++) {
      int idx = seqnum % WINDOWSIZE;
      if (!A_acked[idx]) {
          tolayer3(A, A_window[idx]);
          packets_resent++;
          /* restart timer for this packet */
          if (A_timer_running[idx]) stoptimer(A);
          starttimer(A, RTT);
          A_timer_running[idx] = true;
      }
  }
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void) {
  A_base     = 0;
  A_nextseq  = 0;
  for (int i = 0; i < WINDOWSIZE; i++) {
    A_acked[i]         = false;
    A_timer_running[i] = false;
  }
}


/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
  struct pkt ackpkt;
  /* prepare ACK packet */
  ackpkt.seqnum = NOTINUSE;
  ackpkt.acknum = packet.seqnum;
  memset(ackpkt.payload, 0, 20);
  ackpkt.checksum = ComputeChecksum(ackpkt);

  if (!IsCorrupted(packet)
      && packet.seqnum >= B_base
      && packet.seqnum < B_base + WINDOWSIZE) {
      int idx = packet.seqnum % WINDOWSIZE;
      if (!B_received[idx]) {
          B_buffer[idx] = packet;
          B_received[idx] = true;
      }
      /* send ACK */
      tolayer3(B, ackpkt);
  } else {
      /* out-of-window or corrupted: resend last in-order ACK */
      int lastack = (B_base == 0) ? SEQSPACE-1 : B_base-1;
      ackpkt.acknum = lastack;
      ackpkt.checksum = ComputeChecksum(ackpkt);
      tolayer3(B, ackpkt);
  }

  /* deliver all in-order buffered packets */
  while (B_received[B_base % WINDOWSIZE]) {
      int idx = B_base % WINDOWSIZE;
      tolayer5(B, B_buffer[idx].payload);
      B_received[idx] = false;
      B_base++;
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void) {
  B_base = 0;
  for (int i = 0; i < WINDOWSIZE; i++) {
      B_received[i] = false;
  }
}
/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

/* sr.c */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "sr.h"

#define RTT        16.0
#define WINDOWSIZE 6
#define SEQSPACE   7
#define NOTINUSE  (-1)

/* Sender (A) state */
static struct pkt A_window[WINDOWSIZE];
static char       A_acked[WINDOWSIZE];
static char       A_timer_running[WINDOWSIZE];
static int        A_base;
static int        A_nextseq;

/* Receiver (B) state */
static struct pkt B_buffer[WINDOWSIZE];
static char       B_received[WINDOWSIZE];
static int        B_base;

/*---------------------------------------------------------------------------*/
/* Compute checksum over seqnum, acknum, and payload */
static int ComputeChecksum(struct pkt p)
{
    int sum = p.seqnum + p.acknum;
    int i;
    for (i = 0; i < 20; i++) {
        sum += (unsigned char)p.payload[i];
    }
    return sum;
}

static int IsCorrupted(struct pkt p)
{
    if (p.checksum != ComputeChecksum(p)) {
        return 1;
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Sender initialization */
void A_init(void)
{
    int i;
    A_base     = 0;
    A_nextseq  = 0;
    for (i = 0; i < WINDOWSIZE; i++) {
        A_acked[i]         = 0;
        A_timer_running[i] = 0;
    }
}

/* Called from layer 5 */
void A_output(struct msg message)
{
    int seq;
    struct pkt p;

    if (A_nextseq < A_base + WINDOWSIZE) {
        seq = A_nextseq % SEQSPACE;
        p.seqnum   = seq;
        p.acknum   = NOTINUSE;
        memcpy(p.payload, message.data, 20);
        p.checksum = ComputeChecksum(p);

        A_window[seq]         = p;
        A_acked[seq]          = 0;
        tolayer3(A, p);

        if (!A_timer_running[seq]) {
            starttimer(A, RTT);
            A_timer_running[seq] = 1;
        }
        A_nextseq++;
    }
    else {
        if (TRACE > 0) printf("----A: window full, drop msg\n");
        window_full++;
    }
}

/* Called from layer 3 on ACK arrival */
void A_input(struct pkt packet)
{
    int ack;
    int i;
    int idx;

    if (IsCorrupted(packet)) {
        return;
    }

    ack = packet.acknum;
    for (i = A_base; i < A_nextseq; i++) {
        if (ack == (i % SEQSPACE)) {
            idx = ack;
            if (!A_acked[idx]) {
                A_acked[idx] = 1;
                total_ACKs_received++;
                if (A_timer_running[idx]) {
                    stoptimer(A);
                    A_timer_running[idx] = 0;
                }
                /* Slide base for contiguous ACKs */
                while (A_acked[A_base % SEQSPACE]) {
                    A_acked[A_base % SEQSPACE] = 0;
                    A_base++;
                }
            }
            break;
        }
    }
}

/* Called when A's timer expires */
void A_timerinterrupt(void)
{
    int i;
    int idx;

    if (TRACE > 0) printf("----A: timeout, resending unacked pkts\n");

    for (i = A_base; i < A_nextseq; i++) {
        idx = i % SEQSPACE;
        if (!A_acked[idx]) {
            tolayer3(A, A_window[idx]);
            packets_resent++;
            if (A_timer_running[idx]) {
                stoptimer(A);
            }
            starttimer(A, RTT);
            A_timer_running[idx] = 1;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Receiver initialization */
void B_init(void)
{
    int i;
    B_base = 0;
    for (i = 0; i < WINDOWSIZE; i++) {
        B_received[i] = 0;
    }
}

/* Called from layer 3 on data arrival */
void B_input(struct pkt packet)
{
    struct pkt ackpkt;
    int inWindow;
    int i;
    int idx;
    int lastack;

    /* Prepare ACK packet */
    ackpkt.seqnum   = NOTINUSE;
    ackpkt.acknum   = packet.seqnum;
    memset(ackpkt.payload, 0, 20);
    ackpkt.checksum = ComputeChecksum(ackpkt);

    /* Check if packet.seqnum ∈ [B_base, B_base+WINDOWSIZE) mod SEQSPACE */
    inWindow = 0;
    for (i = B_base; i < B_base + WINDOWSIZE; i++) {
        if (packet.seqnum == (i % SEQSPACE)) {
            inWindow = 1;
            break;
        }
    }

    if (!IsCorrupted(packet) && inWindow) {
        idx = packet.seqnum;
        if (!B_received[idx]) {
            B_buffer[idx]   = packet;
            B_received[idx] = 1;
        }
        /* Send ACK */
        tolayer3(B, ackpkt);
    }
    else {
        /* Out-of-window or corrupted: resend last in-order ACK */
        if (B_base == 0) {
            lastack = SEQSPACE - 1;
        }
        else {
            lastack = (B_base - 1) % SEQSPACE;
        }
        ackpkt.acknum   = lastack;
        ackpkt.checksum = ComputeChecksum(ackpkt);
        tolayer3(B, ackpkt);
    }

    /* Deliver all in-order buffered packets */
    while (B_received[B_base % SEQSPACE]) {
        idx = B_base % SEQSPACE;
        tolayer5(B, B_buffer[idx].payload);
        B_received[idx] = 0;
        B_base++;
    }
}

/* Unused in simplex scenario */
void B_output(struct msg message) { }
void B_timerinterrupt(void)  { }

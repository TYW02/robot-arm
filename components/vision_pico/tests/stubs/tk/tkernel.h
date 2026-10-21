#ifndef VISION_TEST_TKERNEL_H
#define VISION_TEST_TKERNEL_H
#include <stddef.h>
#include <stdint.h>
typedef int32_t INT, ID, ER, SZ, TMO;
typedef uint32_t UW, ATR;
typedef unsigned char UB;
typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define EXPORT
#define TK_SUPPORT_SMP 1
#define TA_TFIFO 0
#define TA_HLNG 1
#define TA_RNG3 2
#define TA_ASSPRC 4
#define TP_PRC1 1
#define TP_PRC2 2
#define TMO_POL 0
#define TMO_FEVR (-1)
#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
typedef struct { ATR mbfatr; SZ bufsz, maxmsz; void *bufptr; } T_CMBF;
typedef struct {
    INT itskpri, stksz;
    void (*task)(INT, void *);
    ATR tskatr;
    UW assprc;
} T_CTSK;
ID tk_cre_mbf(const T_CMBF *);
ER tk_del_mbf(ID);
ER tk_snd_mbf(ID, const void *, INT, TMO);
INT tk_rcv_mbf(ID, void *, TMO);
ID tk_cre_tsk(const T_CTSK *);
ER tk_sta_tsk(ID, INT);
ER tk_del_tsk(ID);
ER tk_dly_tsk(INT);
ER tk_slp_tsk(TMO);
#endif

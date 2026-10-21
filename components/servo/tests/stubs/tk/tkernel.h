#ifndef MOTION_TEST_TKERNEL_H
#define MOTION_TEST_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int INT;
typedef int ID;
typedef int ER;
typedef int SZ;
typedef int BOOL;
typedef uint8_t UB;
typedef uint16_t UH;
typedef uint32_t UW;
typedef struct {
    unsigned int sysstat;
    ID runtskid;
    ID schedtskid;
} T_RSYS;

#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_IO (-57)
#define E_PAR (-17)
#define TMO_FEVR (-1)
#define TSS_DDSP 1U
#define TSS_DINT 2U
#define TSS_INDP 4U

ID tk_get_tid(void);
ER tk_ref_sys(T_RSYS *state);
ER tk_loc_mtx(ID mutex, INT timeout);
ER tk_unl_mtx(ID mutex);
ER tk_swri_dev(ID device, INT address, const void *data, SZ size, SZ *actual);
ER tk_dly_tsk(INT delay);
INT tk_rcv_mbf(ID buffer, void *data, INT timeout);
ER tk_snd_mbf(ID buffer, const void *data, INT size, INT timeout);

#endif

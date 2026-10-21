#include <tk/tkernel.h>
#define TD_UPDATE 3
#define TD_EXCL 4
ID tk_opn_dev(const UB *, INT);
ER tk_cls_dev(ID, INT);
ER tk_swri_dev(ID, INT, const void *, SZ, SZ *);
ER tk_srea_dev(ID, INT, void *, SZ, SZ *);

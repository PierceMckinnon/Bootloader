#ifndef _H_REG_OP_H_
#define _H_REG_OP_H_

#define _write_reg(_REG_, _VAL_) ((_REG_) = (_VAL_))
#define _write_reg_f(_REG_, _VAL_, _POS_, _MASK_)               \
    _write_reg(_REG_,                                           \
               (((_read_reg(_REG_)) & (~((_MASK_) << (_POS_)))) \
                | (((_VAL_) & (_MASK_)) << (_POS_))))
#define _read_reg(_REG_)                  ((_REG_))
#define _read_reg_f(_REG_, _POS_, _MASK_) (((_REG_) >> (_POS_)) & (_MASK_))

// SPECIAL REGISTER OPERATIONS

#define _write_reg_scb_aircr(_REG_, _VAL_) \
    ((_REG_) = (((_VAL_) & ~0xFFFF0000) | (0x5FA << 0x10)))
#define _write_reg_scb_aircr_f(_REG_, _VAL_, _POS_, _MASK_)               \
    _write_reg_scb_aircr(_REG_,                                           \
                         (((_read_reg(_REG_)) & (~((_MASK_) << (_POS_)))) \
                          | (((_VAL_) & (_MASK_)) << (_POS_))))

#endif
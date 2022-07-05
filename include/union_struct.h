#ifndef _UNION_STRUCT_H_
#define _UNION_STRUCT_H_
#include <iostream>

typedef union USHORT_UNION_{
    uint16_t ushort_;
    char bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    char bytes_[4];
} UINT_UNION;

typedef union FLOAT_UNION_{
    float float_;
    char bytes_[4];
} FLOAT_UNION;

#endif
/*
 * datatype.h
 *
 *  Created on: 2015-07-24
 *      Author: Dylan.Gao
 */

#ifndef _DM_DATATYPE_H_
#define _DM_DATATYPE_H_

namespace dm{

#ifdef UNIX
#define PACKED_SUF __attribute__ ((packed))
#else
#define PACKED_SUF
#endif

typedef unsigned int uint;

typedef unsigned char uint8;
typedef char int8;

typedef unsigned short uint16;
typedef short int16;

#ifdef __TMS320C28XX__

typedef unsigned long uint32;
typedef long int32;

typedef unsigned long long uint64;
typedef long long int64;

typedef float float32;
typedef long double float64;

#else

typedef unsigned int uint32;
typedef int int32;

typedef unsigned long long uint64;
typedef long long int64;

typedef float float32;
typedef double float64;

#endif

/**
 * 近似判断
 * @param v1
 * @param v2
 * @param d
 * @return
 */
template<typename T>
inline bool approximate( T v1,T v2, T d ){
	return v1>=v2?(v1-v2)<d:(v2-v1)<d;
}

typedef unsigned long long ptr_t;

}

#endif /* COMMON_DATATYPES_DATATYPE_H_ */

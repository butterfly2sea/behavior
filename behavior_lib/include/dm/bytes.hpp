/*
 * bytes.h
 *
 *  Created on: 2015��8��10��
 *      Author: work
 */

#ifndef DM_BYTES_H_
#define DM_BYTES_H_

#include <dm/types.hpp>

namespace dm{

inline uint8 toByte( const int8& v ){
	return *((const uint8*)&v);
}

inline uint8 lowByte_i16( const int16& v ){
	return ((const uint8*)&v)[0];
}

inline uint8 higByte_i16( const int16& v ){
	return ((const uint8*)&v)[1];
}

inline uint8 lowByte_u16( const uint16& v ){
	return ((const uint8*)&v)[0];
}

inline uint8 higByte_u16( const uint16& v ){
	return ((const uint8*)&v)[1];
}

inline uint8 lowByte_i32( const int32& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_i32( const int32& v,const int& b=0 ){
	return ((const uint8*)&v)[3-b];
}

inline uint8 lowByte_u32( const uint32& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_u32( const uint32& v,const int& b=0 ){
	return ((const uint8*)&v)[3-b];
}

inline uint8 lowByte_f32( const float32& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_f32( const float32& v,const int& b=0 ){
	return ((const uint8*)&v)[3-b];
}

inline uint8 lowByte_u64( const uint64& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_u64( const uint64& v,const int& b=0 ){
	return ((const uint8*)&v)[7-b];
}

inline uint8 lowByte_i64( const int64& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_i64( const int64& v,const int& b=0 ){
	return ((const uint8*)&v)[7-b];
}

inline uint8 lowByte_f64( const float64& v,const int& b=0 ){
	return ((const uint8*)&v)[b];
}

inline uint8 higByte_f64( const float64& v,const int& b=0 ){
	return ((const uint8*)&v)[3-b];
}

inline int8 byte2int8( const uint8& c ){
	return *((const int8*)&c);
}

inline int16 bytes2int16( const uint8& l,const uint8& h ){
	int16 r;
	uint8* p = (uint8*)(&r);
	p[0] = l;
	p[1] = h;
	return r;
}

inline uint16 bytes2uint16( const uint8& l,const uint8& h ){
	uint16 r;
	uint8* p = (uint8*)(&r);
	p[0] = l;
	p[1] = h;
	return r;
}

inline int32 bytes2int32( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3 ){
	int32 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	return r;
}

inline uint32 bytes2uint32( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3 ){
	uint32 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	return r;
}

inline float32 bytes2float32( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3 ){
	float32 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	return r;
}

inline int64 bytes2int64( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3,
		const uint8& l4,const uint8& l5,const uint8& l6,const uint8& l7 ){
	int64 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	p[4] = l4;
	p[5] = l5;
	p[6] = l6;
	p[7] = l7;
	return r;
}

inline uint64 bytes2uint64( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3,
		const uint8& l4,const uint8& l5,const uint8& l6,const uint8& l7 ){
	uint64 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	p[4] = l4;
	p[5] = l5;
	p[6] = l6;
	p[7] = l7;
	return r;
}

inline float64 bytes2float64( const uint8& l0,const uint8& l1,const uint8& l2,const uint8& l3,const uint8& l4,const uint8& l5,const uint8& l6,const uint8& l7 ){
	float64 r;
	uint8* p = (uint8*)(&r);
	p[0] = l0;
	p[1] = l1;
	p[2] = l2;
	p[3] = l3;
	p[4] = l4;
	p[5] = l5;
	p[6] = l6;
	p[7] = l7;
	return r;
}

}

#endif /* INCLUDE_BYTES_H_ */

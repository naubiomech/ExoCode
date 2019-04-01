#include "ByteTranscriber.hpp"
#include <string.h>

void TeensyByteTranscriber::decodeJointSelect(int* selects, double encoded_double_select){

	int encoded_select = *((int*) &encoded_double_select);
	selects[0] = (encoded_select >> 6) & 3;
	selects[1] = (encoded_select >> 3) & 7;
	selects[2] = (encoded_select >> 0) & 7;
}

void TeensyByteTranscriber::decodeDoubles(double* doubles, char* bytes, int amount) {
	memcpy(doubles, bytes, sizeof(double) * amount);
}

void TeensyByteTranscriber::encodeFloat(char* bytes, float* vals, int amount){
	memcpy(vals, bytes, sizeof(double) * amount);
}

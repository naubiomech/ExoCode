#include "ByteTranscriber.hpp"
#include <string.h>

void TeensyByteTranscriber::decodeJointSelect(int* selects, double encoded_double_select){
	int encoded_select = 0;
	memcpy(&encoded_select, &encoded_double_select, sizeof(int));
	selects[0] = (encoded_select >> 6) & 3;
	selects[1] = (encoded_select >> 3) & 7;
	selects[2] = (encoded_select >> 0) & 7;
}

float TeensyByteTranscriber::encodeJointSelect(int* selects){
	int encoded_select = (selects[0] << 6) & 3 ||
		(selects[1] << 3) & 7 ||
		(selects[2] << 0) & 7;
	float value = 0;
	memcpy(&value, &encoded_select, sizeof(float));
	return value;
}

void TeensyByteTranscriber::decodeDoubles(double* doubles, char* bytes, int amount) {
	memcpy(doubles, bytes, sizeof(double) * amount);
}

void TeensyByteTranscriber::encodeFloat(char* bytes, float* vals, int amount){
	memcpy(bytes, vals, sizeof(float) * amount);
}

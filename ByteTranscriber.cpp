#include "ByteTranscriber.hpp"
#include <string.h>

void TeensyByteTranscriber::decodeJointSelect(int* selects, double encoded_double_select){
	int encoded_select = 0;
	memcpy(&encoded_select, &encoded_double_select, sizeof(int));
	selects[0] = (encoded_select >> 16) & 255;
	selects[1] = (encoded_select >> 8) & 255;
	selects[2] = (encoded_select >> 0) & 255;
}

float TeensyByteTranscriber::encodeJointSelect(int* selects){
	int encoded_select = (selects[0] << 16) & 255 ||
		(selects[1] << 8) & 255 ||
		(selects[2] << 0) & 255;
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

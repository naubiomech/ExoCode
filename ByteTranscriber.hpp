#ifndef BYTE_TRANSCRIBER_HEADER
#define BYTE_TRANSCRIBER_HEADER

class ByteTranscriber{
public:
	virtual void decodeJointSelect(int* selects, double encoded_double_select) = 0;
	virtual void decodeDoubles(double* doubles, char* bytes, int amount) = 0;
	virtual void encodeFloat(char* bytes, float* vals, int amount) = 0;
};

class TeensyByteTranscriber : public ByteTranscriber{
public:
	virtual void decodeJointSelect(int* selects, double encoded_double_select);
	virtual void decodeDoubles(double* doubles, char* bytes, int amount);
	virtual void encodeFloat(char* bytes, float* vals, int amount);
};

#endif

#ifndef ENCODER_INTERFACE
#define ENCODER_INTERFACE

#include <Arduino.h>

class EncoderInterface
{
public:
    EncoderInterface() = default;

    virtual inline int32_t read() = 0;
	virtual inline int32_t readAndReset() = 0;
	virtual inline void write(int32_t p) = 0;
	virtual float getRPM() = 0;
};

class EncoderNull: public EncoderInterface
{
public:    
    EncoderNull() = default;
    inline int32_t read() { return 0; }
	inline int32_t readAndReset() { return 0; }
	inline void write(int32_t p) {}
    float getRPM() { return 0.0f; }
};

#endif // #define ENCODER_INTERFACE

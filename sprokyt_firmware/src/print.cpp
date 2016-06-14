// print.c
// A helper file for printing output to Console


#include "print.h"
#include <mbed.h>

Serial pc(USBTX, USBRX);
PwmOut led(LED2);

void ConsoleBlink(unsigned char value)
{
	led = (1.0f / 255.0f) * (float)value;
}

void ConsolePrint(char* format, ...)
{
	va_list args;

	va_start(args, format);
	pc.printf(format, args);
	va_end(args);
}
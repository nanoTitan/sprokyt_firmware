/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRINT_H_
#define __PRINT_H_

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
 extern "C" {
#endif
	
void ConsoleBlink(unsigned char value);
	 
#ifdef DEBUG
void ConsolePrint(char* format, ...);
#endif
	 
#ifdef __cplusplus
}
#endif

#ifdef DEBUG
#define ConPrintF(...) ConsolePrint(__VA_ARGS__)
#else
#define ConPrintF(...)
#endif

#endif // __PRINT_H_
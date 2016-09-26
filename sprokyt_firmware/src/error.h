// error.h

#ifndef _ERROR_H_
#define _ERROR_H_

#include "debug.h"

static void CError_Handler(void)
{
	PRINTF("Error thrown in CError_Handler\r\n");
	while (1)
	{
	}
}

#endif // _ERROR_H_
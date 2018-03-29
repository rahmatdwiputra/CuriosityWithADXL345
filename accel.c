#include "accel.h"
#include "pin_manager.h"

int16_t AccelConversion(char dataMSB, char dataLSB)
{
    char TempData;
    signed int Result;
    
    TempData = dataMSB & 0b01111111;
    Result = ((TempData << 8) + dataLSB) >> 2;
    if (dataMSB & 0b10000000)
    {
        Result = -8192 + Result;
    }
    return Result;
}


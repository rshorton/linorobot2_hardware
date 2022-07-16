#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

inline int sgn(int x)
{
    return (x > 0) - (x < 0);
}

#endif // UTIL_H
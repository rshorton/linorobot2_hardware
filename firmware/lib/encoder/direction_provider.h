#ifndef DIRECTION_PROVIDER_H
#define DIRECTION_PROVIDER_H

#include "Arduino.h"

class DirectionProvider
{
public:
    DirectionProvider() = default;
    virtual bool is_dir_fwd() { return true; }
};

#endif // #define DIRECTION_PROVIDER_H


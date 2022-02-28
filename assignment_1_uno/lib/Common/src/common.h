#pragma once

#define CARRIAGE_RETURN 0x0D

namespace hovercraft
{

    const char whitespace[5] = {0x20,0x20,0x20, 0x20,0x20};
    

    long map(long x, long min, long max, long out_min, long out_max)
    {
        if (x <= min)
        {
            return out_min;
        }
        else if (x >= max)
        {
            return out_min;
        }

        return (x - min) * (out_max - out_min) / (max - min) + out_min;
    }
}
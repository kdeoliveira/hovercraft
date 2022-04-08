#pragma once

namespace hc
{

    class Util
    {
        public:
            Util() = default;

            ~Util()
            {
            }

            static long map(long x, long min, long max, long out_min, long out_max)
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
        };
}
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "Recast.h"

int main()
{
    int iterations = 50000; //change the number of iterations here to observe different iteration performances
    
    float verts[] = {
        1.0f, 2.0f, 3.0f,
        0.0f, 2.0f, 5.0f,
        4.0f, 1.0f, 6.0f,
        2.0f, 8.0f, 1.0f
    };

    float bmin[3];
    float bmax[3];

    // Warm-up call - calls once before timing starts to minimize distortion of timing
    rcCalcBounds(verts, 4, bmin, bmax);

    volatile float sink = 0.0f;

    const auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < iterations; ++i)
    {
        rcCalcBounds(verts, 4, bmin, bmax);

        // Prevent the compiler from treating the calls as useless
        sink += bmin[0] + bmin[1] + bmin[2];
        sink += bmax[0] + bmax[1] + bmax[2];
    }

    const auto end = std::chrono::steady_clock::now();

    const auto total_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    const double ns_per_call = static_cast<double>(total_ns) / iterations;

    std::cout << "iterations=" << iterations << '\n';
    std::cout << "total_ns=" << total_ns << '\n';
    std::cout << "ns_per_call=" << ns_per_call << '\n';
    std::cout << "sink=" << sink << '\n';

    return 0;
}

#include <vector>
#include <utility>
#include "speed_pressure_data.h"

// Define the data array
std::vector<std::pair<float, int>> data = {
        {0.0, 0},
        {12.2, 1},
        {13.1, 2},
        {13.8, 3},
        {14.1, 4},
        {15.7, 5},
        {16.2, 6},
        {17.5, 7},
        {18.9, 8},
        {20.0, 9},
        {22.2, 10},
        {24.1, 11},
        {26.1, 12},
        {28.1, 13},
        {29.0, 14},
        {29.8, 15},
        {31.0, 16},
        {31.6, 17},
        {32.7, 18},
        {33.7, 19},
        {34.5, 20},
        {36.6, 21},
        {37.4, 22},
        {38.2, 23},
        {38.7, 24},
        {39.1, 25},
        {40.3, 26},
        {40.6, 27},
        {41.2, 28},
        {41.8, 29},
        {42.1, 30},
        {42.8, 31},
        {43.1, 32},
        {45.0, 33},
        {45.4, 34},
        {45.8, 35},
        {46.2, 36},
        {46.6, 37},
        {47.0, 38},
        {47.3, 39},
        {47.8, 40},
        {48.3, 41},
        {48.6, 42},
        {49.4, 43},
        {50.2, 44},
        {50.6, 45},
        {51.0, 46},
        {51.8, 47},
        {52.1, 48},
        {52.6, 49},
        {53.0, 50},
        {53.8, 51},
        {53.7, 52},
        {54.2, 53},
        {54.9, 54},
        {55.4, 55},
        {55.8, 56},
        {56.3, 57},
        {56.9, 58},
        {57.1, 59},
        {57.8, 60},
        {58.2, 61},
        {58.6, 62},
        {58.8, 63},
        {60.0, 64},
        {60.2, 65},
        {60.4, 66},
        {60.6, 67},
        {60.8, 68},
        {62.2, 69},
        {62.4, 70},
        {62.9, 71},
        {63.5, 72},
        {64.2, 73},
        {65.1, 74},
        {65.3, 75},
        {65.5, 76},
        {65.7, 77},
        {66.2, 78},
        {66.6, 79},
        {66.8, 80},
        {67.8, 81},
        {68.1, 82},
        {68.4, 83},
        {68.6, 84},
        {68.8, 85},
        {69.3, 86},
        {70.0, 87},
        {70.4, 88},
        {70.8, 89},
        {72.1, 90},
        {72.5, 91},
        {72.8, 92},
        {73.1, 93},
        {73.4, 94},
        {73.7, 95},
        {74.0, 96},
        {74.3, 97},
        {74.6, 98},
        {75.0, 99},
        {76.0, 100},
        {76.9, 101},
        {75.9, 102},
        {76.6, 103},
        {77.1, 104},
        {77.6, 105},
        {78.2, 106},
        {78.8, 107},
        {79.4, 108},
        {79.7, 109},
        {80.0, 110},
        {81.0, 111},
        {82.5, 112},
        {83.0, 113},
        {83.4, 114},
        {83.7, 115},
        {85.1, 116},
        {85.6, 117},
        {87.6, 118},
        {88.0, 119},
        {91.5, 120}
};

float getSpeed(const int pressure_) {
    for (const auto& pair : data) {
        if (pair.second == pressure_) {
            return pair.first;
        }
    }
    return -1.0; // Invalid pressure value
}
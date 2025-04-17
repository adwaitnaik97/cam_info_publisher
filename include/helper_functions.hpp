#include <array>
#include <iostream>
#include <vector>

using namespace std;

template <std::size_t N>

std::array<double, N> VectorToArray(const std::vector<double>& vec)
{
    std::array<double, N> arr;
    std::copy_n(vec.begin(), std::min(vec.size(), N), arr.begin());
    return arr;
}


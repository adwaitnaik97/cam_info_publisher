#include <array>
#include <iostream>
#include <vector>

using namespace std;

template <std::size_t N>

std::array<double, N> VectorToArray(const std::vector<double> &vec)
{
    std::array<double, N> arr;
    std::copy_n(vec.begin(), std::min(vec.size(), N), arr.begin());
    return arr;
}

std::string VectorToString(const std::vector<double> &vec)
{
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i)
    {
        oss << vec[i];
        if (i < vec.size() - 1)
            oss << ", ";
    }
    return oss.str();
}

std::vector<double> FlattenMatrix(const cv::Mat &mat)
{
    std::vector<double> flat;
    for (int r = 0; r < mat.rows; ++r)
        for (int c = 0; c < mat.cols; ++c)
            flat.push_back(mat.at<double>(r, c));
    return flat;
}

std::vector<double> FlattenMatrixToVec(const cv::Mat &mat)
{
    return std::vector<double>((double *)mat.datastart, (double *)mat.dataend);
}

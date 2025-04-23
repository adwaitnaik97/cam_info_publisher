/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2025 AIOS @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * author = 'Adwait Naik'
 * email  = 'adwait@aidrivers.ai'
 *******************************************************/

#include <array>
#include <iostream>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std;

/**
 * @brief Converts a std::vector<double> to a std::array<double, N>.
 *
 * If the input vector is shorter than N, the remaining elements will be default-initialized.
 *
 * @tparam N The size of the target std::array
 * @param vec The input vector
 * @return std::array<double, N> containing the first N elements of the input vector
 */
template <std::size_t N>
std::array<double, N> VectorToArray(const std::vector<double> &vec)
{
    std::array<double, N> arr;
    std::copy_n(vec.begin(), std::min(vec.size(), N), arr.begin());
    return arr;
}

/**
 * @brief Converts a std::vector<double> to a comma-separated string.
 *
 * Useful for logging or debugging purposes.
 *
 * @param vec The input vector
 * @return A string representing the vector values separated by commas
 */
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

/**
 * @brief Flattens a 2D OpenCV matrix (cv::Mat) into a 1D vector in row-major order.
 *
 * Each matrix element is accessed individually using `cv::Mat::at<double>`.
 *
 * @param mat The input matrix
 * @return A std::vector<double> containing the matrix elements in row-major order
 */
std::vector<double> FlattenMatrix(const cv::Mat &mat)
{
    std::vector<double> flat;
    for (int r = 0; r < mat.rows; ++r)
        for (int c = 0; c < mat.cols; ++c)
            flat.push_back(mat.at<double>(r, c));
    return flat;
}

/**
 * @brief Quickly flattens a cv::Mat into a vector using memory access.
 *
 * This method assumes that the matrix is of type CV_64F and is stored contiguously.
 * Use only if you're confident about the underlying data layout.
 *
 * @param mat The input matrix
 * @return A std::vector<double> referencing the matrix's memory directly
 */
std::vector<double> FlattenMatrixToVec(const cv::Mat &mat)
{
    return std::vector<double>((double *)mat.datastart, (double *)mat.dataend);
}

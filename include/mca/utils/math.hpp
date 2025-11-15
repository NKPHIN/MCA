//
// Created by ph431 on 2025/3/26.
//

#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>
#include "mca/common/cv/cv2.hpp"

namespace mca::utils {
    // output: [-180, 180]
    inline double ang(const cv::PointF p1, const cv::PointF p2)
    {
        const float delta_x = p2.getX() - p1.getX();
        const float delta_y = p1.getY() - p2.getY();
        const double angle_radians = std::atan2(delta_y, delta_x);
        const double angle_degrees = angle_radians * (180.0 / M_PI);

        return angle_degrees;
    }

    inline double dis(const cv::PointF p1, const cv::PointF p2)
    {
        const float delta_x = p1.getX() - p2.getX();
        const float delta_y = p1.getY() - p2.getY();
        return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
    }

    inline double psnr(cv::Mat& a, cv::Mat& b)
    {
        constexpr double MAX_N = 255;

        double mse = 0;
        const int width = a.getCols();
        const int height = a.getRows();

        const unsigned char* a_data = a.getData();
        const unsigned char* b_data = b.getData();

        for (int i = 0; i < width * height; i++)
            mse += std::pow((a_data[i] - b_data[i]), 2);
        mse /= (width * height);

        const double res = 10 * std::log10(MAX_N * MAX_N / mse);
        return res;
    }

    inline double y_psnr(const std::string& a, const std::string& b, const int width, const int height, const int frames)
    {
        double total = 0;

        std::ifstream ifs_a(a, std::ifstream::binary);
        std::ifstream ifs_b(b, std::ifstream::binary);
        for (int i = 0; i < frames; i++)
        {
            cv::Mat_C3 image_a = cv::read(ifs_a, width, height);
            cv::Mat_C3 image_b = cv::read(ifs_b, width, height);

            total += psnr(image_a[0], image_b[0]);
        }

        ifs_a.close();
        ifs_b.close();
        return total / frames;
    }
}

#endif //MATH_HPP

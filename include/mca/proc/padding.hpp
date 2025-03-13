//
// Created by ph431 on 2025/3/2.
//

#ifndef PADDING_HPP
#define PADDING_HPP

#include <cmath>
#include <queue>

#include "mca/common/cv/cv2.hpp"

namespace mca::proc {
    inline double Angle(cv::PointF p1, cv::PointF p2)
    {
        const float delta_x = p2.getX() - p1.getX();
        const float delta_y = p1.getY() - p2.getY();
        const double angle_radians = std::atan2(delta_y, delta_x);
        const double angle_degrees = angle_radians * (180.0 / M_PI);

        return angle_degrees;
    }

    inline void default_padding(cv::Mat& src)
    {
        const std::vector<std::vector<int>> vecs = {{0, -90}, {0, 90}, {-76, -46},
            {-76, 44}, {76, -44}, {76, 46}};

        const int rows = src.getRows();
        const int cols = src.getCols();

        for (int x = 0; x < cols; x++)
        {
            for (int y = 0; y < rows; y++)
            {
                unsigned char ch = src.at(y, x);
                if (ch != 0) continue;

                int sum = 0, cnt = 0;
                for (auto vec : vecs)
                {
                    const int ux = x + vec[0];
                    const int uy = y + vec[1];

                    if (ux < 0 || ux >= cols || uy < 0 || uy >= rows || src.at(uy, ux) == 0) continue;

                    cnt++;
                    sum += src.at(uy, ux);
                }
                if (cnt != 0) sum /= cnt;
                src.set(y, x, static_cast<unsigned char>(sum));
            }
        }
    }

    inline void angle_padding(cv::Mat& src, const mca::MI::layout_ptr& layout, int channel)
    {
        const std::vector<std::vector<int>> vecs = {{-76, -46}, {-76, 44}, {0, 90}, {76, 44}, {76, -44}, {0, -90}};

        const int rows = src.getRows();
        const int cols = src.getCols();

        for (int x = 0; x < cols; x++)
        {
            for (int y = 0; y < rows; y++)
            {
                unsigned char ch = src.at(y, x);
                if (ch != 0) continue;

                const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                const double angle = Angle(closest_center, cv::PointF(x, y));

                int offset = 0;
                if (-30 <= angle && angle < 30) offset = 0;
                else if (30 <= angle && angle < 90) offset = 1;
                else if (90 <= angle && angle < 150) offset = 2;
                else if (150 <= angle && angle <= 180 || -180 <= angle  && angle < -150) offset = 3;
                else if (-150 <= angle && angle < -90) offset = 4;
                else if (-90 <= angle && angle < -30) offset = 5;

                int sum = 0, cnt = 0;
                for (int i=0; i<2; i++)
                {
                    const auto& vec = vecs.at((offset + i) % 6);
                    const int ux = x + vec[0];
                    const int uy = y + vec[1];

                    if (ux < 0 || ux >= cols || uy < 0 || uy >= rows || src.at(uy, ux) == 0) continue;

                    cnt++;
                    sum += src.at(uy, ux);
                }
                if (cnt != 0) sum /= cnt;

                if (channel == 0 && sum != 0) sum = std::min(sum + 10, 255);
                src.set(y, x, static_cast<unsigned char>(sum));
            }
        }
    }
};

#endif //PADDING_HPP

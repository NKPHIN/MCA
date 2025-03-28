//
// Created by ph431 on 2025/3/2.
//

#ifndef PADDING_HPP
#define PADDING_HPP

#include <cmath>
#include <queue>

#include "mca/common/cv/cv2.hpp"
#include "mca/utils/math.hpp"

namespace mca::proc {
    inline void default_padding(cv::Mat& src, const std::vector<std::vector<int>>& vecs)
    {
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

    inline void angle_padding(cv::Mat& src, const mca::MI::layout_ptr& layout, const std::vector<std::vector<int>>& vecs, const double ratio)
    {
        const int rows = src.getRows();
        const int cols = src.getCols();

        for (int x = 0; x < cols; x++)
        {
            for (int y = 0; y < rows; y++)
            {
                unsigned char ch = src.at(y, x);
                if (ch != 0) continue;

                const auto fx = static_cast<float>(x);
                const auto fy = static_cast<float>(y);

                const cv::PointF closest_center = layout->closestMICenter(cv::PointF(fx, fy));
                const double angle = ang(closest_center, cv::PointF(fx, fy));

                double distance = dis(closest_center, cv::PointF(fx, fy));
                if (distance > layout->getDiameter() * ratio) continue;

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
                src.set(y, x, static_cast<unsigned char>(sum));
            }
        }
    }

    inline void edge_blurring(cv::Mat_C3& pre, cv::Mat_C3& label, const mca::MI::layout_ptr& layout, const std::vector<double> &theta)
    {
        const int width = pre[0].getCols();
        const int height = pre[0].getRows();

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                unsigned char ch = label[0].at(y, x);
                if (ch != 0) continue;

                const auto fx = static_cast<float>(x);
                const auto fy = static_cast<float>(y);

                const cv::PointF closest_center = layout->closestMICenter(cv::PointF(fx, fy));
                const int distance = static_cast<int>(dis(closest_center, cv::PointF(fx, fy)));

                int pixel = pre[0].at(y, x);
                pixel = std::min(static_cast<int>(pixel * theta[distance]), 255);

                pre[0].set(y, x, static_cast<unsigned char>(pixel));
            }
        }
    }

    inline void padding(cv::Mat_C3& src, const mca::MI::layout_ptr& layout, const std::vector<std::vector<int>>& vecs)
    {
        cv::Mat_C3 label = src;
        for (int c = 1; c < 3; c++)
        {
            // boys2: 0.43
            mca::proc::angle_padding(src[c], layout, vecs, 0.5);
            mca::proc::default_padding(src[c], vecs);
            mca::proc::default_padding(src[c], vecs);
        }
        // mca::proc::edge_blurring(src, label, layout, theta);
    }
};

#endif //PADDING_HPP

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

    inline double dis(cv::PointF p1, cv::PointF p2)
    {
        const float delta_x = p1.getX() - p2.getX();
        const float delta_y = p1.getY() - p2.getY();
        return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
    }

    inline void default_padding(cv::Mat& src)
    {
        // const std::vector<std::vector<int>> vecs = {{0, -90}, {0, 90}, {-76, -46}, {-76, 44}, {76, -44}, {76, 46}};
        // const std::vector<std::vector<int>> vecs = {{14, 8}, {14, -8}, {0, -16}, {-14, -8}, {-14, 8}, {0, 16}}; // fujita
        const std::vector<std::vector<int>> vecs = {{23, 13}, {23, -13}, {0, -26}, {-23, -13}, {-23, 13}, {0, 26}};

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

    inline void angle_padding(cv::Mat& src, const mca::MI::layout_ptr& layout)
    {
        // const std::vector<std::vector<int>> vecs = {{-76, -46}, {-76, 44}, {0, 90}, {76, 44}, {76, -44}, {0, -90}};    // Boys2
        // const std::vector<std::vector<int>> vecs = {{-76, -42}, {-76, 44}, {0, 86}, {76, 44}, {76, -42}, {0, -86}};    // MiniGarden2 TSPC
        // const std::vector<std::vector<int>> vecs = {{14, 8}, {14, -8}, {0, -16}, {-14, -8}, {-14, 8}, {0, 16}};        // fujita、origami
        const std::vector<std::vector<int>> vecs = {{23, 13}, {23, -13}, {0, -26}, {-23, -13}, {-23, 13}, {0, 26}};   // R32、Boxer2

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

                float distance = dis(closest_center, cv::PointF(x, y));
                if (distance > layout->getDiameter() * 0.5) continue;

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


    inline void edge_blurring(cv::Mat_C3& pre, cv::Mat_C3& label, const mca::MI::layout_ptr& layout)
    {
        // std::string ori_path = R"(C:\WorkSpace\MPEG\MCA\Sequence\MiniGarden2_4036x3064_1frames_8bit_yuv420.yuv)";
        // std::string ori_path = R"(C:\WorkSpace\MPEG\MCA\Sequence\Boys2_3976x2956_10frames_8bit_yuv420.yuv)";
        // std::string ori_path = R"(C:\WorkSpace\MPEG\MCA\Sequence\Origami_2048x2048_1frames_8bit_yuv420.yuv)";
        std::string ori_path = R"(C:\WorkSpace\MPEG\MCA\Sequence\Boxer-IrishMan-Gladiator2_3840x2160_1frames_8bit_yuv420.yuv)";
        std::ifstream ifs(ori_path, std::ios::binary);

        // attention the width * height !!
        cv::Mat_C3 ori = mca::cv::read(ifs, 3840, 2160);
        ori = cv::Transpose(ori);

        const int width = pre[0].getCols();
        const int height = pre[0].getRows();

        for (int c = 0; c < 3; c++)
        {
            std::vector<std::vector<double>> zi, mu, theta;
            int rows = layout->getRows();
            int cols = layout->getCols();

            zi.resize(rows * cols + 1);
            mu.resize(rows * cols + 1);
            theta.resize(rows * cols + 1);

            for (int i = 0; i < rows * cols; i++)
            {
                zi[i].resize(100, 0);
                mu[i].resize(100, 0);
                theta[i].resize(100, 1.0);
            }

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    char ch = label[c].at(y, x);
                    if (ch != 0) continue;

                    const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                    int distance = static_cast<int>(dis(closest_center, cv::PointF(x, y)));

                    int ori_pixel = ori[c].at(y, x);
                    int pre_pixel = pre[c].at(y, x);

                    int row_id = layout->getMIRowColIndex(closest_center).first;
                    int col_id = layout->getMIRowColIndex(closest_center).second;

                    int id = row_id * cols + col_id;

                    zi[id][distance] += static_cast<double>(pre_pixel * ori_pixel);
                    mu[id][distance] += static_cast<double>(pre_pixel * pre_pixel);
                }
            }

            for (int r = 0; r < rows; r++)
            {
                for (int _c = 0; _c < cols; _c++)
                {
                    int id = r * cols + _c;
                    for (int i = 0; i < 100; i++)
                    {
                        if (zi[id][i] != 0)
                            theta[id][i] = zi[id][i] / mu[id][i];
                    }
                }
            }

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    char ch = label[c].at(y, x);
                    if (ch != 0) continue;

                    const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                    int distance = static_cast<int>(dis(closest_center, cv::PointF(x, y)));

                    int row_id = layout->getMIRowColIndex(closest_center).first;
                    int col_id = layout->getMIRowColIndex(closest_center).second;
                    int id = row_id * cols + col_id;

                    int pixel = pre[c].at(y, x);
                    pixel = std::min(static_cast<int>(pixel * theta[id][distance]), 255);

                    pre[c].set(y, x, static_cast<unsigned char>(pixel));
                }
            }
        }
    }
};

#endif //PADDING_HPP

//
// Created by ph431 on 2025/8/5.
//

#pragma once

#include "mca/utils/math.hpp"

namespace mca::module::decoder {
    class OptimizeModule final : public Module<cv::Mat_C3, cv::Mat_C3, cv::Mat_C3, std::vector<double>, MI::layout_ptr> {
    public:
        cv::Mat_C3 exec(cv::Mat_C3 recon_frame, cv::Mat_C3 reloc_frame, std::vector<double> theta, MI::layout_ptr layout) override
        {
            const auto width = layout->getWidth();
            const auto height = layout->getHeight();

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    unsigned char ch = reloc_frame[0].at(y, x);
                    if (ch != 0) continue;

                    const auto fx = static_cast<float>(x);
                    const auto fy = static_cast<float>(y);

                    const cv::PointF closest_center = layout->closestMICenter(cv::PointF(fx, fy));
                    const int distance = static_cast<int>(utils::dis(closest_center, cv::PointF(fx, fy)));

                    int pixel = recon_frame[0].at(y, x);
                    pixel = std::min(static_cast<int>(pixel * theta[distance]), 255);

                    recon_frame[0].set(y, x, static_cast<unsigned char>(pixel));
                }
            }

            return std::move(recon_frame);
        }
    };
}

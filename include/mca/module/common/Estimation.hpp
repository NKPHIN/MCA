//
// Created by ph431 on 2025/7/31.
//

#ifndef ESTIMATION_HPP
#define ESTIMATION_HPP

#include "../../utils/math.hpp"

namespace mca::module::common {
    class EstimationModule final : public Module<cv::Mat_C3, cv::Mat_C3, MI::layout_ptr, Dict> {
    public:
        cv::Mat_C3 exec(const cv::Mat_C3 reloc_frame, const MI::layout_ptr layout, Dict config) override {

            const auto opt = std::any_cast<int>(config["optimize"]);
            const auto vecs = std::any_cast<std::vector<std::vector<int>>>(config["vectors"]);

            cv::Mat_C3 recon_frame = estimate(reloc_frame, layout, vecs, opt);

            return std::move(recon_frame);
        }

    private:
        static void default_padding(cv::Mat& src, const std::vector<std::vector<int>>& vecs)
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

        static void angle_padding(cv::Mat& src, const mca::MI::layout_ptr& layout, const std::vector<std::vector<int>>& vecs)
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
                    const double angle = utils::ang(closest_center, cv::PointF(fx, fy));

                    double distance = utils::dis(closest_center, cv::PointF(fx, fy));
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

        static cv::Mat_C3 estimate(cv::Mat_C3 src, const mca::MI::layout_ptr& layout, const std::vector<std::vector<int>>& vecs, const int mode)
        {
            for (int c = 0; c < 3; c++)
            {
                if (mode == 1)
                    angle_padding(src[c], layout, vecs);
                else if (mode == 0)
                    default_padding(src[c], vecs);
                default_padding(src[c], vecs);
                default_padding(src[c], vecs);
            }
            return src;
        }
    };
}

#endif //ESTIMATION_HPP

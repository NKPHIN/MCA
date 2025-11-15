//
// Created by ph431 on 2025/7/31.
//

#ifndef FITTING_HPP
#define FITTING_HPP

namespace mca::module::encoder {
    class FittingModule final : public Module<std::pair<std::vector<double>, std::vector<double>>, cv::Mat_C3, cv::Mat_C3, cv::Mat_C3, MI::layout_ptr> {
    public:
        std::pair<std::vector<double>, std::vector<double>>
        exec(cv::Mat_C3 raw_frame, cv::Mat_C3 reloc_frame, cv::Mat_C3 recon_frame, const MI::layout_ptr layout) override {

            auto [a, b] = fitting(raw_frame, reloc_frame, recon_frame, layout);

            return {a, b};
        }

    private:
        static std::pair<std::vector<double>, std::vector<double>> fitting(cv::Mat_C3& src, cv::Mat_C3& reloc, cv::Mat_C3& recon, const MI::layout_ptr& layout)
        {
            // Only for Y channel
            const int width = src[0].getCols();
            const int height = src[0].getRows();

            std::vector<double> zi, mu, a, b;
            zi.resize(100, 0);
            mu.resize(100, 0);
            a.resize(100, 1.0);
            b.resize(100, 1.0);

            std::vector<double> src_pixel_avg, pre_pixel_avg, pixel_cnt;
            src_pixel_avg.resize(100, 0);
            pre_pixel_avg.resize(100, 0);
            pixel_cnt.resize(100, 0);

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    unsigned char ch = reloc[0].at(y, x);
                    if (ch != 0) continue;

                    const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                    const int distance = static_cast<int>(utils::dis(closest_center, cv::PointF(x, y)));

                    const int src_pixel = src[0].at(y, x);
                    const int pre_pixel = recon[0].at(y, x);

                    src_pixel_avg[distance] += src_pixel;
                    pre_pixel_avg[distance] += pre_pixel;
                    pixel_cnt[distance] += 1;
                }
            }

            for (int i=0; i<100; i++)
            {
                if (pixel_cnt[i] > 0)
                {
                    src_pixel_avg[i] /= pixel_cnt[i];
                    pre_pixel_avg[i] /= pixel_cnt[i];
                }
            }

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    unsigned char ch = reloc[0].at(y, x);
                    if (ch != 0) continue;

                    const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                    const int distance = static_cast<int>(utils::dis(closest_center, cv::PointF(x, y)));

                    const int src_pixel = src[0].at(y, x);
                    const int pre_pixel = recon[0].at(y, x);

                    zi[distance] += (pre_pixel - pre_pixel_avg[distance]) * (src_pixel - src_pixel_avg[distance]);
                    mu[distance] += (pre_pixel - pre_pixel_avg[distance]) * (pre_pixel - pre_pixel_avg[distance]);
                }
            }

            for (int i = 0; i < 100; i++)
            {
                if (zi[i] != 0)
                {
                    a[i] = zi[i] / mu[i];
                    b[i] = src_pixel_avg[i] - a[i] * pre_pixel_avg[i];
                }
                else a[i] = b[i] = -1000;
            }

            return {a, b};
        }
    };
}

#endif //FITTING_HPP

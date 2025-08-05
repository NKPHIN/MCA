//
// Created by ph431 on 2025/7/31.
//

#ifndef FITTING_HPP
#define FITTING_HPP

namespace mca::module::encoder {
    class FittingModule final : public Module<std::vector<double>, cv::Mat_C3, cv::Mat_C3, cv::Mat_C3, MI::layout_ptr> {
    public:
        std::vector<double>
        exec(cv::Mat_C3 raw_frame, cv::Mat_C3 reloc_frame, cv::Mat_C3 recon_frame, const MI::layout_ptr layout) override {

            std::vector<double> theta = fitting(raw_frame, reloc_frame, recon_frame, layout);

            return std::move(theta);
        }

    private:
        static std::vector<double> fitting(cv::Mat_C3& src, cv::Mat_C3& reloc, cv::Mat_C3& recon, const MI::layout_ptr& layout)
        {
            // Only for Y channel
            const int width = src[0].getCols();
            const int height = src[0].getRows();

            std::vector<double> zi, mu, theta;
            zi.resize(100, 0);
            mu.resize(100, 0);
            theta.resize(100, 1.0);

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

                    zi[distance] += static_cast<double>(pre_pixel * src_pixel);
                    mu[distance] += static_cast<double>(pre_pixel * pre_pixel);
                }
            }

            for (int i = 0; i < 100; i++)
            {
                if (zi[i] != 0) theta[i] = zi[i] / mu[i];
                else theta[i] = -1.0;
            }

            return theta;
        }
    };
}

#endif //FITTING_HPP

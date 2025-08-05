//
// Created by ph431 on 2025/8/5.
//

#ifndef YUV420WRITER_HPP
#define YUV420WRITER_HPP

#include <any>
#include <vector>
#include <string>
#include <fstream>

#include "../../common/cv/Mat.hpp"
#include "../../common/cv/yuv420.hpp"
#include "../../common/layout/TSPCLayout.hpp"
#include "../../common/pipeline/module.hpp"


namespace mca::module::common {
    class YUV420Writer final : public Module<void, std::vector<cv::Mat_C3>, MI::layout_ptr, Dict> {
    public:
        void exec(std::vector<cv::Mat_C3> video, MI::layout_ptr layout, Dict config) override
        {
            const auto frames = std::any_cast<int>(config["frames"]);
            const auto rotation = std::any_cast<float>(config["rotation"]);

            const auto output_path = std::any_cast<std::string>(config["output"]);
            std::ofstream ofs(output_path, std::ios::binary);
            if (!ofs.is_open())
                throw std::ios_base::failure("Failed to open file: " + output_path);

            for (int i = 0; i < frames; i++)
            {
                if (rotation < std::numbers::pi / 4)
                    video[i] = cv::Transpose(video[i]);

                cv::write(ofs, video[i]);
            }
        }
    };
}

#endif //YUV420WRITER_HPP

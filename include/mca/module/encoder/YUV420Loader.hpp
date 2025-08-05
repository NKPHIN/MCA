//
// Created by ph431 on 2025/8/5.
//

#ifndef YUV420LOADER_HPP
#define YUV420LOADER_HPP

#include <any>
#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>

#include "mca/common/cv/Mat.hpp"
#include "mca/common/cv/yuv420.hpp"
#include "mca/common/pipeline/module.hpp"


namespace mca::module::encoder {
    class YUV420Loader final : public Module<std::vector<cv::Mat_C3>, common::Dict> {
    public:
        std::vector<cv::Mat_C3> exec(common::Dict config) override
        {
            const auto path = std::any_cast<std::string>(config["input"]);

            const auto frames = std::any_cast<int>(config["frames"]);
            const auto width = std::any_cast<int>(config["width"]);
            const auto height = std::any_cast<int>(config["height"]);
            const auto rotation = std::any_cast<float>(config["rotation"]);

            std::vector<cv::Mat_C3> video;

            std::ifstream ifs(path, std::ios::binary);
            if (!ifs.is_open())
                throw std::ios_base::failure("Failed to open file: " + path);

            for (int i = 0; i < frames; i++)
            {
                cv::Mat_C3 YUV = cv::read(ifs, width, height);
                if (rotation < std::numbers::pi / 4)
                    YUV = cv::Transpose(YUV);

                video.push_back(YUV);
            }

            return std::move(video);
        }
    };
}

#endif //YUV420LOADER_HPP

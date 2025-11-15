//
// Created by ph431 on 2025/8/5.
//

#pragma once


namespace mca::module::decoder {
    class YUV420Loader final : public Module<std::vector<cv::Mat_C3>, MI::layout_ptr, common::Dict> {
    public:
        std::vector<cv::Mat_C3> exec(const MI::layout_ptr layout, common::Dict config) override
        {
            const auto path = std::any_cast<std::string>(config["input"]);

            const auto frames = std::any_cast<int>(config["frames"]);
            const auto rotation = std::any_cast<float>(config["rotation"]);

            const auto patch = std::any_cast<int>(config["patch"]);
            const auto width = layout->getMCAWidth(patch);
            const auto height = layout->getMCAHeight(patch);

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

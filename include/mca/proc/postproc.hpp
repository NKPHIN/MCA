//
// Created by ph431 on 2025/2/25.
//

#ifndef POSTPROC_HPP
#define POSTPROC_HPP

#include "mca/common/cv/cv2.hpp"
#include "mca/io/parser/parser.hpp"

namespace mca::proc {
    inline void postproc(parser::ArgParser arg_parser,
        parser::ConfigParser& config_parser)
    {
        const std::string input_path = arg_parser.get("-i");
        const std::string output_dir = arg_parser.get("-o");

        const int frames = std::stoi(config_parser.get("Frames"));
        const int dstWidth = std::stoi(config_parser.get("Width"));
        const int dstHeight = std::stoi(config_parser.get("Height"));
        const float cropRatio = std::stof(config_parser.get("Ratio"));

        const std::string output_path = get_ouput_path(dstWidth, dstHeight, frames,
            input_path, output_dir, "post");

        std::ifstream ifs(input_path, std::ios::binary);
        std::ofstream ofs(output_path, std::ios::binary);

        // 把 rotation 作为 TSPC 和 Raytrix 的公有属性提取出来
        float rotation = 1;
        MI::layout_ptr layout;

        if (config_parser.get("Type") == "RayCalibData")
        {
            layout = std::make_shared<MI::RaytrixLayout>(dstWidth, dstHeight, config_parser);
            rotation = std::stof(config_parser.get("rotation"));
        }
        else if (config_parser.get("Type") == "TSPCCalibData")
            layout = std::make_shared<MI::TSPCLayout>(dstWidth, dstHeight, config_parser);

        int width = layout->getMCAWidth(cropRatio);
        int height = layout->getMCAHeight(cropRatio);

        for (int i = 0; i < frames; i++)
        {
            cv::Mat_C3 YUV = cv::read(ifs, width, height);
            if (rotation < std::numbers::pi / 4)
                YUV = cv::Transpose(YUV);

            cv::Mat_C3 MCA_YUV = proc::single_frame(YUV, layout, cropRatio, proc::POST);
            if (rotation < std::numbers::pi / 4)
                MCA_YUV = cv::Transpose(MCA_YUV);

            cv::write(ofs, MCA_YUV);
            std::cout << (i+1) << std::endl;
        }

        ifs.close();
        ofs.close();
    }
}


#endif //POSTPROC_HPP

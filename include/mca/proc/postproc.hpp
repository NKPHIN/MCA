//
// Created by ph431 on 2025/2/25.
//

#ifndef POSTPROC_HPP
#define POSTPROC_HPP

#include "mca/common/cv/cv2.hpp"
#include "mca/io/parser/parser.hpp"
#include "tspc.hpp"

namespace mca::proc {
    inline void postproc(parser::ArgParser arg_parser,
        parser::ConfigParser& config_parser)
    {
        const std::string input_path = arg_parser.get("-i");
        const std::string output_path = arg_parser.get("-o");

        const int frames = std::stoi(config_parser.get("Frames"));
        const int dstWidth = std::stoi(config_parser.get("Width"));
        const int dstHeight = std::stoi(config_parser.get("Height"));
        const float cropRatio = std::stof(config_parser.get("Ratio"));

        std::ifstream ifs(input_path, std::ios::binary);
        std::ofstream ofs(output_path, std::ios::binary);


        for (int i = 0; i < frames; i++)
        {
            if (config_parser.get("Type") == "TSPCCalibData")
            {
                mca::MI::TSPCLayout layout(dstWidth, dstHeight, config_parser);

                const int width = layout.getMCAWidth(cropRatio);
                const int height = layout.getMCAHeight(cropRatio);

                cv::Mat_C3 YUV = cv::read(ifs, width, height);
                cv::Mat_C3 MCA_YUV = mca::proc::TSPC_POST(YUV, layout, cropRatio);
                cv::write(ofs, MCA_YUV);
            }
            else if (config_parser.get("Type") == "RayCalibData")
            {
                mca::MI::RaytrixLayout layout(dstWidth, dstHeight, config_parser);

                int width = layout.getMCAWidth(cropRatio);
                int height = layout.getMCAHeight(cropRatio);

                const float rotation = std::stof(config_parser.get("rotation"));
                if (rotation < std::numbers::pi / 4)
                    std::swap(width, height);

                cv::Mat_C3 YUV = cv::read(ifs, width, height);
                if (rotation < std::numbers::pi / 4)
                    YUV = cv::Transpose(YUV);
                cv::Mat_C3 MCA_YUV = mca::proc::Raytrix_Post(YUV, layout, cropRatio);
                if (rotation < std::numbers::pi / 4)
                    MCA_YUV = cv::Transpose(MCA_YUV);
                cv::write(ofs, MCA_YUV);
            }
        }

        ifs.close();
        ofs.close();
    }
}


#endif //POSTPROC_HPP

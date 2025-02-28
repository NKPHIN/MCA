//
// Created by ph431 on 2025/2/25.
//

#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <complex>

#include "raytrix.hpp"
#include "mca/common/cv/cv2.hpp"
#include "mca/io/parser/parser.hpp"
#include "mca/common/layout/layout.hpp"
#include "tspc.hpp"

namespace mca::proc {

    inline void log(parser::ArgParser& arg_parser,
        parser::ConfigParser& config_parser,
        parser::Calibration::CalibParser& calib_parser)
    {
        const std::string log_path = arg_parser.get("-log");
        auto ofs = std::ofstream(log_path);

        if (calib_parser.type() == "TSPCCalibData")
        {
            ofs << "Type:" << calib_parser.type() << std::endl;

            ofs << "Frames:" << config_parser.get("FramesToBeEncoded") << std::endl;
            ofs << "Width:" << config_parser.get("SourceWidth") << std::endl;
            ofs << "Height:" << config_parser.get("SourceHeight") << std::endl;
            ofs << "Ratio:" << arg_parser.get("-ratio") << std::endl;

            ofs << "diameter:" << calib_parser.search("diameter") << std::endl;
            ofs << "ltopx:" << calib_parser.search("ltop", "x") << std::endl;
            ofs << "ltopy:" << calib_parser.search("ltop", "y") << std::endl;
            ofs << "rtopx:" << calib_parser.search("rtop", "x") << std::endl;
            ofs << "rtopy:" << calib_parser.search("rtop", "y") << std::endl;
            ofs << "lbotx:" << calib_parser.search("lbot", "x") << std::endl;
            ofs << "lboty:" << calib_parser.search("lbot", "y") << std::endl;
            ofs << "rbotx:" << calib_parser.search("rbot", "x") << std::endl;
            ofs << "rboty:" << calib_parser.search("rbot", "y") << std::endl;

            ofs.close();
        }
        else if (calib_parser.type() == "RayCalibData")
        {
            ofs << "Type:" << calib_parser.type() << std::endl;

            ofs << "Frames:" << config_parser.get("FramesToBeEncoded") << std::endl;
            ofs << "Width:" << config_parser.get("SourceWidth") << std::endl;
            ofs << "Height:" << config_parser.get("SourceHeight") << std::endl;
            ofs << "Ratio:" << arg_parser.get("-ratio") << std::endl;

            ofs << "diameter:" << calib_parser.search("diameter") << std::endl;
            ofs << "rotation:" << calib_parser.search("rotation") << std::endl;
            ofs << "offsetx:" << calib_parser.search("offset", "x") << std::endl;
            ofs << "offsety:" << calib_parser.search("offset", "y") << std::endl;
        }
    }

    inline void preproc(parser::ArgParser arg_parser,
        parser::ConfigParser& config_parser,
        parser::Calibration::CalibParser& calib_parser)
    {
        const std::string input_path = arg_parser.get("-i");
        const std::string output_path = arg_parser.get("-o");

        const int frames = std::stoi(config_parser.get("FramesToBeEncoded"));
        int width = std::stoi(config_parser.get("SourceWidth"));
        int height = std::stoi(config_parser.get("SourceHeight"));

        const float cropRatio = std::stof(arg_parser.get("-ratio"));

        std::ifstream ifs(input_path, std::ios::binary);
        std::ofstream ofs(output_path, std::ios::binary);

        for (int i = 0; i < frames; i++)
        {
            cv::Mat_C3 YUV = cv::read(ifs, width, height);
            if (calib_parser.type() == "TSPCCalibData")
            {
                mca::MI::TSPCLayout layout(width, height, calib_parser);
                cv::Mat_C3 MCA_YUV = mca::proc::TSPC_PRE(YUV, layout, cropRatio);
                cv::write(ofs, MCA_YUV);
            }
            else if (calib_parser.type() == "RayCalibData")
            {
                const float rotation = std::stof(calib_parser.search("rotation"));
                if (rotation < std::numbers::pi / 4)
                    YUV = cv::Transpose(YUV);

                mca::MI::RaytrixLayout layout(width, height, calib_parser);
                cv::Mat_C3 MCA_YUV = mca::proc::Raytrix_Pre(YUV, layout, cropRatio);

                if (rotation < std::numbers::pi / 4)
                    MCA_YUV = cv::Transpose(MCA_YUV);
                cv::write(ofs, MCA_YUV);
            }
        }

        ifs.close();
        ofs.close();

        log(arg_parser, config_parser, calib_parser);
    }
};

#endif //PREPROC_HPP

//
// Created by ph431 on 2025/2/25.
//

#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <complex>
#include <filesystem>
#include "crop.hpp"
#include "log.hpp"
#include "mca/common/cv/cv2.hpp"
#include "mca/io/parser/parser.hpp"
#include "mca/common/layout/layout.hpp"

namespace fs = std::filesystem;
namespace mca::proc {
    inline std::string get_ouput_path(const int width, const int height,
        const int frames, const fs::path& input_path, fs::path output_dir, const std::string& mode)
    {
        std::string input_file_name = input_path.stem().string();
        const size_t firstUnderscorePos = input_file_name.find('_');

        const std::string seq_name = input_file_name.substr(0, firstUnderscorePos);
        const std::string output_file_name = seq_name + "_" + std::to_string(width) + "x" + std::to_string(height) + "_" +
            std::to_string(frames) + "frames_8bit_yuv420_" + mode + ".yuv";

        output_dir.append(output_file_name);
        return output_dir.string();
    }

    inline void preproc(parser::ArgParser arg_parser,
        parser::ConfigParser& config_parser,
        parser::Calibration::CalibParser& calib_parser)
    {
        mca::proc::Logger logger(arg_parser, config_parser);
        logger.writeBasicData(calib_parser);

        const std::string input_path = arg_parser.get("-i");
        const std::string output_dir = arg_parser.get("-o");

        std::string output_path;
        const int frames = std::stoi(config_parser.get("FramesToBeEncoded"));
        int width = std::stoi(config_parser.get("SourceWidth"));
        int height = std::stoi(config_parser.get("SourceHeight"));

        const int patch_size = std::stoi(arg_parser.get("-patch"));

        float rotation = 1;
        MI::layout_ptr layout;
        if (calib_parser.type() == "RayCalibData")
        {
            layout = std::make_shared<MI::RaytrixLayout>(width, height, calib_parser);
            rotation = std::stof(calib_parser.search("rotation"));
        }
        else if (calib_parser.type() == "TSPCCalibData")
            layout = std::make_shared<MI::TSPCLayout>(width, height, calib_parser);

        int MCA_width = layout->getMCAWidth(patch_size);
        int MCA_height = layout->getMCAHeight(patch_size);
        output_path = get_ouput_path(MCA_width, MCA_height, frames, input_path, output_dir, "pre");

        std::ifstream ifs(input_path, std::ios::binary);
        std::ofstream ofs(output_path, std::ios::binary);
        for (int i = 0; i < frames; i++)
        {
            cv::Mat_C3 YUV = cv::read(ifs, width, height);
            if (rotation < std::numbers::pi / 4)
                YUV = cv::Transpose(YUV);

            cv::Mat_C3 MCA_YUV = proc::crop(YUV, layout, patch_size, proc::PRE);
            logger.writeMetaData(YUV, MCA_YUV, layout, i);

            if (rotation < std::numbers::pi / 4)
                MCA_YUV = cv::Transpose(MCA_YUV);
            cv::write(ofs, MCA_YUV);

            std::cout << (i+1) << std::endl;
        }

        ifs.close();
        ofs.close();
    }
};

#endif //PREPROC_HPP

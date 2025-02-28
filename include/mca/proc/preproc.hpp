//
// Created by ph431 on 2025/2/25.
//

#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <complex>
#include <filesystem>
#include "frame.hpp"
#include "mca/common/cv/cv2.hpp"
#include "mca/io/parser/parser.hpp"
#include "mca/common/layout/layout.hpp"

namespace fs = std::filesystem;
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
        const std::string input_path = arg_parser.get("-i");
        const std::string output_dir = arg_parser.get("-o");

        std::string output_path;
        const int frames = std::stoi(config_parser.get("FramesToBeEncoded"));
        int width = std::stoi(config_parser.get("SourceWidth"));
        int height = std::stoi(config_parser.get("SourceHeight"));

        const float cropRatio = std::stof(arg_parser.get("-ratio"));

        float rotation = 1;
        MI::layout_ptr layout;
        if (calib_parser.type() == "RayCalibData")
        {
            layout = std::make_shared<MI::RaytrixLayout>(width, height, calib_parser);
            rotation = std::stof(calib_parser.search("rotation"));
        }
        else if (calib_parser.type() == "TSPCCalibData")
            layout = std::make_shared<MI::TSPCLayout>(width, height, calib_parser);

        int MCA_width = layout->getMCAWidth(cropRatio);
        int MCA_height = layout->getMCAHeight(cropRatio);
        output_path = get_ouput_path(MCA_width, MCA_height, frames, input_path, output_dir, "pre");

        std::ifstream ifs(input_path, std::ios::binary);
        std::ofstream ofs(output_path, std::ios::binary);
        for (int i = 0; i < frames; i++)
        {
            cv::Mat_C3 YUV = cv::read(ifs, width, height);
            if (rotation < std::numbers::pi / 4)
                YUV = cv::Transpose(YUV);

            cv::Mat_C3 MCA_YUV = proc::single_frame(YUV, layout, cropRatio, proc::PRE);
            if (rotation < std::numbers::pi / 4)
                MCA_YUV = cv::Transpose(MCA_YUV);
            cv::write(ofs, MCA_YUV);

            std::cout << (i+1) << std::endl;
        }

        ifs.close();
        ofs.close();

        log(arg_parser, config_parser, calib_parser);
    }
};

#endif //PREPROC_HPP

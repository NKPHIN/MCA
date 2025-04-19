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
    inline void edge(cv::Mat_C3& src_image, cv::Mat_C3& mca_image, cv::Mat_C3& label_image, const mca::MI::layout_ptr& layout)
    {
        const int width = src_image[0].getCols();
        const int height = src_image[0].getRows();

        std::vector<double> zi, mu, theta;
        zi.resize(100, 0);
        mu.resize(100, 0);
        theta.resize(100, 1.0);

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                unsigned char ch = label_image[0].at(y, x);
                if (ch != 0) continue;

                const cv::PointF closest_center = layout->closestMICenter(cv::PointF(x, y));
                const int distance = static_cast<int>(dis(closest_center, cv::PointF(x, y)));

                const int src_pixel = src_image[0].at(y, x);
                const int pre_pixel = mca_image[0].at(y, x);

                zi[distance] += static_cast<double>(pre_pixel * src_pixel);
                mu[distance] += static_cast<double>(pre_pixel * pre_pixel);
            }
        }

        for (int i = 0; i < 100; i++)
        {
            if (zi[i] != 0) theta[i] = zi[i] / mu[i];
        }

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                unsigned char ch = label_image[0].at(y, x);
                if (ch != 0) continue;

                const auto fx = static_cast<float>(x);
                const auto fy = static_cast<float>(y);

                const cv::PointF closest_center = layout->closestMICenter(cv::PointF(fx, fy));
                const int distance = static_cast<int>(dis(closest_center, cv::PointF(fx, fy)));

                int pixel = mca_image[0].at(y, x);
                pixel = std::min(static_cast<int>(pixel * theta[distance]), 255);

                mca_image[0].set(y, x, static_cast<unsigned char>(pixel));
            }
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
            cv::Mat_C3 MCA_POST_YUV = proc::crop(MCA_YUV, layout, patch_size, proc::POST);
            cv::Mat_C3 MASK_YUV = MCA_POST_YUV;

            prediction::IntraBlockTree block_tree(YUV, MCA_POST_YUV);
            logger.writeIntraMVData(block_tree, i);

            cv::Mat_C3 MCA_INTRA_YUV = block_tree.getCroppedMat();

            for (int t=0; t<3; t++)
                default_padding(MCA_INTRA_YUV[0], {{-76, -42}, {-76, 44}, {0, 86}, {76, 44}, {76, -42}, {0, -86}});

            logger.writeMetaData(YUV, MCA_INTRA_YUV, MASK_YUV, layout, i);

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

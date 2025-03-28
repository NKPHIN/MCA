//
// Created by ph431 on 2025/3/25.
//

#ifndef LOG_HPP
#define LOG_HPP

#include "crop.hpp"
#include "mca/pred/prediction.hpp"
#include "mca/common/cv/cv2.hpp"
#include "mca/common/layout/MI.hpp"
#include "mca/io/parser/parser.hpp"
#include "mca/pred/prediction.hpp"

namespace mca::proc {
    class Logger {
    private:
        int frames;
        int width;
        int height;
        int patch;
        std::string log_path;
        std::vector<std::vector<int>> vecs;

        void initVecs(const std::string& seq_name)
        {
            if (seq_name == "Boys2_3976x2956_300frames_8bit_yuv420.yuv")
                vecs = {{-76, -46}, {-76, 44}, {0, 90}, {76, 44}, {76, -44}, {0, -90}};
            else if (seq_name == "Fujita2_2048x2048_300frames_8bit_yuv420.yuv" || seq_name == "Origami_2048x2048_300frames_8bit_yuv420.yuv")
                vecs = {{14, 8}, {14, -8}, {0, -16}, {-14, -8}, {-14, 8}, {0, 16}};
            else if (seq_name == "Boxer-IrishMan-Gladiator2_3840x2160_300frames_8bit_yuv420.yuv" || seq_name == "TempleBoatGiantR32_6464x4852_300frames_8bit_yuv420.yuv")
                vecs = {{23, 13}, {23, -13}, {0, -26}, {-23, -13}, {-23, 13}, {0, 26}};
            else vecs = {{-76, -42}, {-76, 44}, {0, 86}, {76, 44}, {76, -42}, {0, -86}};
        }

    public:
        Logger(parser::ArgParser& arg_parser, parser::ConfigParser& config_parser)
        {
            frames = std::stoi(config_parser.get("FramesToBeEncoded"));
            width = std::stoi(config_parser.get("SourceWidth"));
            height = std::stoi(config_parser.get("SourceHeight"));
            patch = std::stoi(arg_parser.get("-patch"));
            log_path = arg_parser.get("-log");

            const std::string seq_name = config_parser.get("InputFile");
            initVecs(seq_name);
        }

        void writeBasicData(parser::Calibration::CalibParser& calib_parser) const
        {
            auto ofs = std::ofstream(log_path);

            ofs << "Type:" << calib_parser.type() << std::endl;
            ofs << "Frames:" << frames << std::endl;
            ofs << "Width:" << width << std::endl;
            ofs << "Height:" << height << std::endl;
            ofs << "Patch:" << patch << std::endl;
            ofs << "diameter:" << calib_parser.search("diameter") << std::endl;
            ofs << "vectors:";
            for (auto vec : vecs)
                ofs << vec[0] << "," << vec[1] << ",";
            ofs << std::endl;

            if (calib_parser.type() == "TSPCCalibData")
            {
                ofs << "ltopx:" << calib_parser.search("ltop", "x") << std::endl;
                ofs << "ltopy:" << calib_parser.search("ltop", "y") << std::endl;
                ofs << "rtopx:" << calib_parser.search("rtop", "x") << std::endl;
                ofs << "rtopy:" << calib_parser.search("rtop", "y") << std::endl;
                ofs << "lbotx:" << calib_parser.search("lbot", "x") << std::endl;
                ofs << "lboty:" << calib_parser.search("lbot", "y") << std::endl;
                ofs << "rbotx:" << calib_parser.search("rbot", "x") << std::endl;
                ofs << "rboty:" << calib_parser.search("rbot", "y") << std::endl;
            }
            else if (calib_parser.type() == "RayCalibData")
            {
                ofs << "rotation:" << calib_parser.search("rotation") << std::endl;
                ofs << "offsetx:" << calib_parser.search("offset", "x") << std::endl;
                ofs << "offsety:" << calib_parser.search("offset", "y") << std::endl;
            }
            ofs.close();
        }

        void writeMetaData(cv::Mat_C3& src_image, cv::Mat_C3& mca_image, const mca::MI::layout_ptr& layout, const int index) const
        {
            auto ofs = std::ofstream(log_path, std::ios::app);
            cv::Mat_C3 post_image = crop(mca_image, layout, patch, proc::POST);
            cv::Mat_C3 label_image = post_image;

            // 目前只对 y 通道做边界优化操作
            angle_padding(post_image[0], layout, vecs, 0.5);
            default_padding(post_image[0], vecs);
            default_padding(post_image[0], vecs);

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
                    const int pre_pixel = post_image[0].at(y, x);

                    zi[distance] += static_cast<double>(pre_pixel * src_pixel);
                    mu[distance] += static_cast<double>(pre_pixel * pre_pixel);
                }
            }

            for (int i = 0; i < 100; i++)
            {
                if (zi[i] != 0) theta[i] = zi[i] / mu[i];
            }

            ofs << "Frame" << index << ":";
            bool first = true;
            for (int i = 0; i < 100; i++)
            {
                if (zi[i] != 0)
                {
                    if (first)
                    {
                        first = false;
                        ofs << i << ",";
                    }
                    ofs << theta[i] << ",";
                }
            }
            ofs << std::endl;
            ofs.close();
        }

        void writeMVData(prediction::inter::BlockTree block_tree, int frame) const
        {
            std::ofstream ofs(log_path, std::ios::app);
            const prediction::inter::BlockNodePtr root = block_tree.getRoot();

            ofs << "Frame" << frame << ":";
            std::vector<std::vector<int>> mvs = block_tree.getMVs();
            for (const auto& mv : mvs)
            {
                for (auto elem : mv)
                    ofs << elem << ",";
            }
            ofs << std::endl;
        }
    };

    inline std::vector<std::vector<int>> readVectors(mca::parser::ConfigParser& log_parser)
    {
        std::vector<std::vector<int>> vecs;
        vecs.resize(6);
        for (auto& vec : vecs)
            vec.resize(2);

        const std::string data = log_parser.get("vectors");

        std::stringstream stream(data);
        std::string token;

        int start = 0;
        while (std::getline(stream, token, ',')) { // 以逗号为分隔符
            const int out_index = start / 2;
            const int in_index = start % 2;

            vecs[out_index][in_index] = std::stoi(token);
            start++;
        }
        return vecs;
    }

    inline std::vector<double> readMetaData(mca::parser::ConfigParser& log_parser, const int index)
    {
        std::vector<double> theta;
        theta.resize(100, 1.0);

        const std::string data = log_parser.get("Frame" + std::to_string(index));
        std::stringstream stream(data);
        std::string token;

        int start = 0;
        while (std::getline(stream, token, ',')) { // 以逗号为分隔符
            if (start == 0)
                start = std::stoi(token);
            else theta[start++] = std::stod(token);
        }
        return theta;
    }

    inline std::vector<std::vector<int>> readBlockTree(mca::parser::ConfigParser& log_parser, const int index)
    {
        std::vector<std::vector<int>> vecs;

        const std::string data = log_parser.get("Frame" + std::to_string(index));
        std::stringstream stream(data);

        std::string level, mv_x, mv_y;
        while (std::getline(stream, level, ','))
        {
            std::vector<int> vec;
            if (std::stoi(level) >= 1)
            {
                std::getline(stream, mv_x, ',');
                std::getline(stream, mv_y, ',');
                vec = {std::stoi(level), std::stoi(mv_x), std::stoi(mv_y)};
            }
            else vec = {std::stoi(level)};
            vecs.push_back(vec);
        }
        return vecs;
    }
};

#endif //LOG_HPP

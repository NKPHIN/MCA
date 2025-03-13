//
// Created by ph431 on 2025/2/25.
//

#ifndef YUV420_HPP
#define YUV420_HPP

#include <array>

namespace mca::cv {
    typedef std::array<cv::Mat, 3> Mat_C3;

    inline cv::Mat_C3 Transpose(cv::Mat_C3& src)
    {
        const cv::Mat Y_t = cv::Transpose(src[0]);
        const cv::Mat U_t = cv::Transpose(src[1]);
        const cv::Mat V_t = cv::Transpose(src[2]);

        return {Y_t, U_t, V_t};
    }

    inline cv::Mat UpSample(cv::Mat& src)
    {
        const int rows = src.getRows();
        const int cols = src.getCols();

        cv::Mat res(rows * 2, cols * 2);
        const unsigned char* src_data = src.getData();
        unsigned char* res_data = res.getData();

        for (int i = 0; i < rows * 2; i++)
        {
            for (int j = 0; j < cols * 2; j++)
            {
                const int orig_i = i / 2;
                const int orig_j = j / 2;
                res_data[i * cols * 2 + j] = src_data[orig_i * cols + orig_j];
            }
        }
        return res;
    }

    inline cv::Mat DownSample(cv::Mat& src)
    {
        const int rows = src.getRows();
        const int cols = src.getCols();

        if (rows % 2 != 0 || cols % 2 != 0)
            throw std::invalid_argument("Error: DownSample size must be even! (mca::cv::DownSample)");

        cv::Mat res(rows / 2, cols / 2);
        const unsigned char* src_data = src.getData();
        unsigned char* res_data = res.getData();

        for (int i = 0; i < rows / 2; i++)
        {
            for (int j = 0; j < cols / 2; j++)
            {
                const int orig_i = i * 2;
                const int orig_j = j * 2;

                res_data[i * cols / 2 + j] = src_data[orig_i * cols + orig_j];
            }
        }

        return res;
    }

    inline Mat_C3 read(std::ifstream& ifs, const int width, const int height)
    {
        const int size = width * height;

        cv::Mat Y(height, width);
        cv::Mat U(height / 2, width / 2);
        cv::Mat V(height / 2, width / 2);

        ifs.read(reinterpret_cast<char*>(Y.getData()), Y.size());
        ifs.read(reinterpret_cast<char*>(U.getData()), U.size());
        ifs.read(reinterpret_cast<char*>(V.getData()), V.size());

        const cv::Mat U_up = UpSample(U);
        const cv::Mat V_up = UpSample(V);

        Mat_C3 YUV = {Y, U_up, V_up};
        return YUV;
    }

    inline void write(std::ofstream& ofs, const Mat_C3& YUV)
    {
        cv::Mat Y = YUV[0], U = YUV[1], V = YUV[2];

        cv::Mat U_down = DownSample(U);
        cv::Mat V_down = DownSample(V);

        ofs.write(reinterpret_cast<char*>(Y.getData()), Y.size());
        ofs.write(reinterpret_cast<char*>(U_down.getData()), U_down.size());
        ofs.write(reinterpret_cast<char*>(V_down.getData()), V_down.size());
    }
}

#endif //YUV420_HPP

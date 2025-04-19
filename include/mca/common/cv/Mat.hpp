//
// Created by ph431 on 2025/2/25.
//

#ifndef MAT_HPP
#define MAT_HPP

#include <cstring>
#include <vector>
#include "Point.hpp"
#include "Rect.hpp"

namespace mca::cv {
    class Mat {
    private:
        int rows = 0;
        int cols = 0;
        std::vector<unsigned char> data;

    public:
        Mat() = default;
        Mat(const int rows, const int cols)
        {
            this->rows = rows;
            this->cols = cols;
            data.resize(rows * cols);
        }

        [[nodiscard]] int getRows() const {return rows;}
        [[nodiscard]] int getCols() const {return cols;}

        [[nodiscard]] int size() const {return rows * cols;}

        unsigned char* getData() { return data.data();}

        std::vector<unsigned char> operator[](const int row)   // start from 0
        {
            const int start = row * cols;
            const int end = start + cols;

            return std::vector<unsigned char>{data.begin()+start, data.begin()+end};
        }

        unsigned char& at(const int row, const int col)
        {
            return data[row * cols + col];
        }

        void set(const int row, const int col, const unsigned char value)
        {
            data[row * cols + col] = value;
        }

        [[nodiscard]] bool contains(const Point<int> p) const
        {
            const int x = p.getX();
            const int y = p.getY();
            if (x < 0 || x >= cols || y < 0 || y >= rows) return false;
            return true;
        }

        [[nodiscard]] bool contains(const Region& region) const
        {
            const cv::Point<int> ltop = region.o();
            const cv::Point<int> rtop = region.rtop();
            const cv::Point<int> lbot = region.lbot();
            const cv::Point<int> rbot = region.rbot();

            return contains(lbot) && contains(rtop) && contains(ltop) && contains(rtop);
        }

        [[nodiscard]] Mat sub(const Region& region) const
        {
            const cv::Point<int> ltop = region.o();
            const int width = region.getWidth();
            const int height = region.getHeight();

            if (height > rows || width > cols)
                throw std::out_of_range("The size of Sub matrix must less than origin matrix!");
            Mat result(height, width);

            unsigned char* res_data = result.getData();
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    const int ux = x + ltop.getX();
                    const int uy = y + ltop.getY();

                    res_data[x + y * width] = data[ux + uy * cols];
                }
            }

            return result;
        }
    };

    inline bool copyTo(Mat& src, Mat& dst, const Region& srcRoi, const Region& dstRoi)
    {
        if (!srcRoi.match(dstRoi)) return false;
        if (!src.contains(srcRoi) || !dst.contains(dstRoi)) return false;

        const unsigned char* srcData = src.getData();
        unsigned char* dstData = dst.getData();

        const cv::Point<int> src_o = srcRoi.o();
        const cv::Point<int> dst_o = dstRoi.o();
        const int width = srcRoi.getWidth();
        const int height = srcRoi.getHeight();

        for (int i = 0; i < height; i++)
        {
            const int src_start = (src_o.getY() + i) * src.getCols() + src_o.getX();
            const int dst_start = (dst_o.getY() + i) * dst.getCols() + dst_o.getX();

            strncpy(reinterpret_cast<char *>(dstData + dst_start),
                reinterpret_cast<const char *>(srcData + src_start), sizeof(char)*width);
        }

        return true;
    }

    inline bool copyTo(Mat& src, Mat& dst, Mat& mask, const Region& srcRoi, const Region& dstRoi)
    {
        if (!srcRoi.match(dstRoi)) return false;
        if (!src.contains(srcRoi) || !dst.contains(dstRoi)) return false;

        const unsigned char* srcData = src.getData();
        unsigned char* dstData = dst.getData();

        const cv::Point<int> src_o = srcRoi.o();
        const cv::Point<int> dst_o = dstRoi.o();
        const int width = srcRoi.getWidth();
        const int height = srcRoi.getHeight();

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                const int src_x = src_o.getX() + x;
                const int src_y = src_o.getY() + y;

                const int dst_x = dst_o.getX() + x;
                const int dst_y = dst_o.getY() + y;

                if (mask.at(dst_y, dst_x) > 0) continue;

                unsigned char src_value = src.at(src_y, src_x);
                dst.set(dst_y, dst_x, src_value);
            }
        }

        return true;
    }

    inline cv::Mat Transpose(cv::Mat& src)
    {
        const int rows = src.getRows();
        const int cols = src.getCols();

        cv::Mat res(cols, rows);

        const unsigned char* src_data = src.getData();
        unsigned char* dst_data = res.getData();
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
                dst_data[j * rows + i] = src_data[i * cols + j];
        }

        return res;
    }
}

#endif //MAT_HPP

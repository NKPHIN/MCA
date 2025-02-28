//
// Created by ph431 on 2025/2/27.
//

#ifndef TSPC_HPP
#define TSPC_HPP

#include "mca/common/cv/Mat.hpp"
#include "mca/common/cv/yuv420.hpp"
#include "mca/common/layout/MI.hpp"


namespace  mca::proc {
    inline cv::Mat_C3 TSPC_PRE(cv::Mat_C3& src, const MI::TSPCLayout& layout, const float cropRatio)
    {
        const int rows = layout.getRows();
        const int cols = layout.getCols();
        const float diameter = layout.getDiameter();

        int patch_size = static_cast<int>(diameter * cropRatio);
        if (patch_size % 2 == 1) patch_size++;

        const int dst_width = patch_size * cols;
        const int dst_height = patch_size * rows;
        std::cout << dst_width << "x" << dst_height << std::endl;
        cv::Mat_C3 dst = {cv::Mat(dst_height, dst_width),
            cv::Mat(dst_height, dst_width), cv::Mat(dst_height, dst_width)};

        for (int x = 0; x < cols; x++)
        {
            int col_lens = rows;
            if (!layout.isSecondColOut() && x % 2 == 1) col_lens--;

            for (int y = 0; y < col_lens; y++)
            {
                mca::MI::MicroImage mi = layout.getMI(y, x);
                cv::PointF center = mi.getCenter();

                const float offset = diameter * cropRatio / 2;
                cv::PointF ltop = center - cv::PointF(offset, offset);

                int ltop_x = static_cast<int>(ltop.getX());
                int ltop_y = static_cast<int>(ltop.getY());
                if (ltop_x % 2 == 1) ltop_x++;
                if (ltop_y % 2 == 1) ltop_y++;

                cv::Region srcRoi(ltop_x, ltop_y, patch_size, patch_size);
                cv::Region dstRoi(patch_size * x, patch_size * y, patch_size, patch_size);

                for (int c = 0; c < 3; c++)
                    cv::copyTo(src[c], dst[c], srcRoi, dstRoi);
            }
        }
        return dst;
    }


    inline cv::Mat_C3 TSPC_POST(cv::Mat_C3& src, const MI::TSPCLayout& layout, const float cropRatio)
    {
        const int rows = layout.getRows();
        const int cols = layout.getCols();
        const float diameter = layout.getDiameter();

        int patch_size = static_cast<int>(diameter * cropRatio);
        if (patch_size % 2 == 1) patch_size++;

        const int dst_width = layout.getWidth();
        const int dst_height = layout.getHeight();
        cv::Mat_C3 dst = {cv::Mat(dst_height, dst_width),
            cv::Mat(dst_height, dst_width), cv::Mat(dst_height, dst_width)};

        for (int x = 0; x < cols; x++)
        {
            int col_lens = rows;
            if (!layout.isSecondColOut() && x % 2 == 1) col_lens--;

            for (int y = 0; y < col_lens; y++)
            {
                mca::MI::MicroImage mi = layout.getMI(y, x);
                cv::PointF center = mi.getCenter();

                const float offset = diameter * cropRatio / 2;
                cv::PointF ltop = center - cv::PointF(offset, offset);

                int ltop_x = static_cast<int>(ltop.getX());
                int ltop_y = static_cast<int>(ltop.getY());
                if (ltop_x % 2 == 1) ltop_x++;
                if (ltop_y % 2 == 1) ltop_y++;

                cv::Region srcRoi(patch_size * x, patch_size * y, patch_size, patch_size);
                cv::Region dstRoi(ltop_x, ltop_y, patch_size, patch_size);

                for (int c = 0; c < 3; c++)
                    cv::copyTo(src[c], dst[c], srcRoi, dstRoi);
            }
        }
        return dst;
    }
};

#endif //TSPC_HPP

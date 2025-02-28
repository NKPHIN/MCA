//
// Created by ph431 on 2025/2/27.
//

#ifndef RAYTRIX_HPP
#define RAYTRIX_HPP

#include "mca/common/cv/cv2.hpp"
#include "mca/common/layout/layout.hpp"


namespace mca::proc {
    inline cv::Mat_C3 Raytrix_Pre(cv::Mat_C3& src, const MI::layout_ptr& layout, const float cropRatio)
    {
        const int rows = layout->getRows();
        const int cols = layout->getCols();
        const int diameter = static_cast<int>(layout->getDiameter());
        const int patch_size = static_cast<int>(diameter * cropRatio);

        const int dst_width = patch_size * cols;
        const int dst_height = patch_size * rows;
        // std::cout << dst_width << "x" << dst_height << std::endl;
        cv::Mat_C3 dst = {cv::Mat(dst_height, dst_width),
            cv::Mat(dst_height, dst_width), cv::Mat(dst_height, dst_width)};

        int first_col_rows = layout->getFirstColRows();
        int second_col_rows = layout->getSecondColRows();

        for (int x = 0; x < cols; x++)
        {
            int len;
            if (x % 2 == 0) len = first_col_rows;
            else len = second_col_rows;

            for (int y = 0; y < len; y++)
            {
                mca::MI::MicroImage mi = layout->getMI(y, x);
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

    inline cv::Mat_C3 Raytrix_Post(cv::Mat_C3& src, const MI::RaytrixLayout& layout, const float cropRatio)
    {
        const int rows = layout.getRows();
        const int cols = layout.getCols();
        const int diameter = static_cast<int>(layout.getDiameter());
        const int patch_size = static_cast<int>(diameter * cropRatio);

        const int dst_width = layout.getWidth();
        const int dst_height = layout.getHeight();
        cv::Mat_C3 dst = {cv::Mat(dst_height, dst_width),
            cv::Mat(dst_height, dst_width), cv::Mat(dst_height, dst_width)};


        const int first_col_rows = layout.getFirstColRows();
        const int second_col_rows = layout.getSecondColRows();

        for (int x = 0; x < cols; x++)
        {
            int len;
            if (x % 2 == 0) len = first_col_rows;
            else len = second_col_rows;

            for (int y = 0; y < len; y++)
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
}

#endif //RAYTRIX_HPP

//
// Created by ph431 on 2025/2/28.
//

#ifndef FRAME_HPP
#define FRAME_HPP

#include "padding.hpp"
#include "mca/common/cv/cv2.hpp"
#include "mca/common/layout/MI.hpp"

namespace mca::proc {
    constexpr int PRE = 0;
    constexpr int POST = 1;

    /**
     * @brief The implementation of Module 'MCA'.
     *
     * @param src The original lenslet image with Y、U、V channels when mode='Pre' OR
     *            The MCA image with Y、U、V channels when mode='Post'
     *
     * @param layout The corresponding layout of sequence.
     * @param patch_size The MCA patch size in pixels.
     * @param mode Only 2 values are accepted:
     *             'Pre' for MCA preprocessing.
     *             'Post' for MCA postprocessing.
     *
     * @return The MCA image with Y、U、V channels when mode='Pre' OR
     *         The re-aligned lenslet image with Y、U、V channels when mode='Post'
     */
    inline cv::Mat_C3 crop(cv::Mat_C3& src, const MI::layout_ptr& layout, int patch_size, const int mode)
    {
        const int rows = layout->getRows();
        const int cols = layout->getCols();

        if (patch_size % 2 == 1) patch_size++;

        int dst_width = patch_size * cols;
        int dst_height = patch_size * rows;
        if (mode == proc::POST)
        {
            dst_width = layout->getWidth();
            dst_height = layout->getHeight();
        }

        cv::Mat_C3 dst = {cv::Mat(dst_height, dst_width),
            cv::Mat(dst_height, dst_width), cv::Mat(dst_height, dst_width)};

        const int first_col_rows = layout->getFirstColRows();
        const int second_col_rows = layout->getSecondColRows();

        for (int x = 0; x < cols; x++)
        {
            const int lens = (x % 2 == 0) ? first_col_rows : second_col_rows;
            for (int y = 0; y < lens; y++)
            {
                mca::MI::MicroImage mi = layout->getMI(y, x);
                cv::PointF center = mi.getCenter();

                const auto offset = static_cast<float>(patch_size / 2.0);
                cv::PointF ltop = center - cv::PointF(offset, offset);

                int ltop_x = static_cast<int>(ltop.getX());
                int ltop_y = static_cast<int>(ltop.getY());
                if (ltop_x % 2 == 1) ltop_x++;
                if (ltop_y % 2 == 1) ltop_y++;

                cv::Region srcRoi(ltop_x, ltop_y, patch_size, patch_size);
                cv::Region dstRoi(patch_size * x, patch_size * y, patch_size, patch_size);

                if (mode == proc::POST)
                    std::swap(srcRoi, dstRoi);

                for (int c = 0; c < 3; c++)
                    cv::copyTo(src[c], dst[c], srcRoi, dstRoi);
            }
        }
        return dst;
    }
}

#endif //FRAME_HPP

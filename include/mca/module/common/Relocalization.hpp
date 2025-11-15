//
// Created by ph431 on 2025/7/31.
//

#ifndef RELOCALIZATION_HPP
#define RELOCALIZATION_HPP

#include "../../common/pipeline/module.hpp"


namespace mca::module::common {
    class RelocalizationModule final : public Module<cv::Mat_C3, cv::Mat_C3, MI::layout_ptr, Dict>{
    public:
        cv::Mat_C3 exec(cv::Mat_C3 mca_frame, const MI::layout_ptr layout, Dict config) override {

            const auto patch_size = std::any_cast<int>(config["patch"]);

            cv::Mat_C3 reloc_frame = relocalize(mca_frame, layout, patch_size);

            return std::move(reloc_frame);
        }

    private:
        static cv::Mat_C3 relocalize(cv::Mat_C3& src, const MI::layout_ptr& layout, int patch_size)
        {
            const int rows = layout->getRows();
            const int cols = layout->getCols();

            if (patch_size % 2 == 1) patch_size++;

            const int dst_width = layout->getWidth();
            const int dst_height = layout->getHeight();

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

                    cv::Region dstRoi(ltop_x, ltop_y, patch_size, patch_size);
                    cv::Region srcRoi(patch_size * x, patch_size * y, patch_size, patch_size);

                    for (int c = 0; c < 3; c++)
                        cv::copyTo(src[c], dst[c], srcRoi, dstRoi);
                }
            }
            return dst;
        }
    };
}

#endif //RELOCALIZATION_HPP

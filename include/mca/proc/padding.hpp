//
// Created by ph431 on 2025/3/2.
//

#ifndef PADDING_HPP
#define PADDING_HPP

#include <cmath>
#include "mca/common/cv/cv2.hpp"

namespace mca::proc {
    inline bool in_circle(const cv::PointI p, const cv::PointF center, const float diameter)
    {
        const float delta_x = static_cast<float>(p.getX()) - center.getX();
        const float delta_y = static_cast<float>(p.getY()) - center.getY();

        const float dis = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        return dis <= diameter / 2;
    }

    inline void default_padding(cv::Mat& src, cv::PointF center, const cv::Region rect, float diameter)
    {
        cv::PointI ltop = rect.o();
        cv::PointI rtop = rect.rtop();
        cv::PointI lbot = rect.lbot();
        cv::PointI rbot = rect.rbot();

        const std::vector<cv::PointI> dir = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
        const std::vector<cv::PointI> contour = {ltop, lbot, rbot, rtop};

        int index = 0;
        cv::PointI start = ltop;
        while (index < 4)
        {
            for (int d = 0; d < 20; d++)
            {
                cv::PointI cur = start + dir[index] * d;
                if (in_circle(cur, center, diameter))
                {
                    const char value = src.at(start.getY(), start.getX());
                    src.set(cur.getY(), cur.getX(), value);
                }
                else break;
            }
            start = start + dir[(index + 1) % 4];
            if (start == contour[(index + 1) % 4]) index++;
        }
    }
};

#endif //PADDING_HPP

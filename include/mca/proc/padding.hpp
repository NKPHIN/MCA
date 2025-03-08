//
// Created by ph431 on 2025/3/2.
//

#ifndef PADDING_HPP
#define PADDING_HPP

#include <cmath>
#include <queue>

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
                    char cur_value = src.at(cur.getY(), cur.getX());
                    if (cur_value != 0) continue;

                    const char src_value = src.at(start.getY(), start.getX());
                    src.set(cur.getY(), cur.getX(), src_value);
                }
                else break;
            }
            start = start + dir[(index + 1) % 4];
            if (start == contour[(index + 1) % 4]) index++;
        }
    }

    inline void edge_padding(cv::Mat& src, cv::PointI center, int width, int height)
    {
        std::queue<cv::PointI> Q;
        Q.emplace(center);

        cv::Mat vis(height, width);
        vis.set(center.getY(), center.getX(), 1);

        const std::vector<cv::PointI> dirs = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
        while (!Q.empty())
        {
            cv::PointI cur = Q.front();
            for (auto dir : dirs)
            {
                const int next_x = cur.getX() + dir.getX();
                const int next_y = cur.getY() + dir.getY();

                if (next_x < 0 || next_x >= width || next_y < 0 || next_y >= height || vis.at(next_y, next_x) == 1)
                    continue;

                char val = src.at(next_y, next_x);
                if (val == 0)
                {
                    const char cur_value = src.at(cur.getY(), cur.getX());
                    src.set(next_y, next_x, cur_value);
                }

                cv::PointI next(next_x, next_y);
                Q.emplace(next);
                vis.set(next.getY(), next.getX(), 1);
            }
            Q.pop();
        }
    }
};

#endif //PADDING_HPP

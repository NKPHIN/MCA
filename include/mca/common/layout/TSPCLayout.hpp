//
// Created by ph431 on 2025/2/27.
//

#ifndef TSPCLAYOUT_HPP
#define TSPCLAYOUT_HPP
#include "MI.hpp"
#include "mca/io/parser/config.hpp"

namespace mca::MI {
    class TSPCLayout : public AbstractMILayout{
    private:
        bool is_second_row_out = false;

        cv::PointF ltop;
        cv::PointF rtop;
        cv::PointF lbot;
        cv::PointF rbot;

        void calculateLayout() override
        {
            int gaps = cols - 1;
            if (is_second_row_out) gaps--;
            const cv::PointF offset_top_x = (rtop - ltop) / static_cast<float>(gaps);
            const cv::PointF offset_bot_x = (rbot - lbot) / static_cast<float>(gaps);

            for (int i = 0; i < cols; i++)
            {
                cv::PointF start = ltop + offset_top_x * static_cast<float>(i);
                cv::PointF end = lbot + offset_bot_x * static_cast<float>(i);

                const int lens = (i % 2 == 0) ? first_col_rows : second_col_rows;
                for (int j = 0; j < lens; j++)
                {
                    cv::PointF offset = (end - start) / static_cast<float>(rows - 1);
                    cv::PointF center = start + offset * static_cast<float>(j);

                    if (i % 2 == 1) center = center + cv::PointF(0, diameter / 2);
                    setMI(j, i, center);
                }
            }
        }

    public:
        TSPCLayout(const int width, const int height, parser::Calibration::CalibParser parser)
        {
            diameter = std::stof(parser.search("diameter"));
            this->width = width;
            this->height = height;

            const float ltop_x = std::stof(parser.search("ltop", "x"));
            const float ltop_y = std::stof(parser.search("ltop", "y"));
            const float rtop_x = std::stof(parser.search("rtop", "x"));
            const float rtop_y = std::stof(parser.search("rtop", "y"));
            const float lbot_x = std::stof(parser.search("lbot", "x"));
            const float lbot_y = std::stof(parser.search("lbot", "y"));
            const float rbot_x = std::stof(parser.search("rbot", "x"));
            const float rbot_y = std::stof(parser.search("rbot", "y"));

            rows = static_cast<int>((lbot_y - ltop_y) / diameter) + 1;
            cols = static_cast<int>((rtop_x - ltop_x) / (diameter * std::sqrt(3) / 2)) + 1;

            first_col_rows = rows;
            second_col_rows = rows - 1;

            // 第二行/列可能因为错位多出来一个
            if (rtop_x + diameter <= static_cast<float>(width))
            {
                cols++;
                is_second_row_out = true;
            }
            if (lbot_y + diameter <= static_cast<float>(height))
                second_col_rows = rows;

            ltop = cv::PointF(ltop_x, ltop_y);
            rtop = cv::PointF(rtop_x, rtop_y);
            lbot = cv::PointF(lbot_x, lbot_y);
            rbot = cv::PointF(rbot_x, rbot_y);

            layout.resize(rows * cols);
            TSPCLayout::calculateLayout();
        }

        TSPCLayout(const int width, const int height, parser::ConfigParser parser)
        {
            diameter = std::stof(parser.get("diameter"));
            this->width = width;
            this->height = height;

            const float ltop_x = std::stof(parser.get("ltopx"));
            const float ltop_y = std::stof(parser.get("ltopy"));
            const float rtop_x = std::stof(parser.get("rtopx"));
            const float rtop_y = std::stof(parser.get("rtopy"));
            const float lbot_x = std::stof(parser.get("lbotx"));
            const float lbot_y = std::stof(parser.get("lboty"));
            const float rbot_x = std::stof(parser.get("rbotx"));
            const float rbot_y = std::stof(parser.get("rboty"));

            rows = static_cast<int>((lbot_y - ltop_y) / diameter) + 1;
            cols = static_cast<int>((rtop_x - ltop_x) / (diameter * std::sqrt(3) / 2)) + 1;

            first_col_rows = rows;
            second_col_rows = rows - 1;

            // 第二行/列可能因为错位多出来一个
            if (rtop_x + diameter <= static_cast<float>(width))
            {
                cols++;
                is_second_row_out = true;
            }
            if (lbot_y + diameter <= static_cast<float>(height))
                second_col_rows = rows;

            ltop = cv::PointF(ltop_x, ltop_y);
            rtop = cv::PointF(rtop_x, rtop_y);
            lbot = cv::PointF(lbot_x, lbot_y);
            rbot = cv::PointF(rbot_x, rbot_y);

            layout.resize(rows * cols);
            TSPCLayout::calculateLayout();
        }

        ~TSPCLayout() override = default;
        // [[nodiscard]] bool isSecondColOut() const {return is_second_row_out;}
    };

    typedef std::shared_ptr<TSPCLayout> tspc_layout_ptr;
}


#endif //TSPCLAYOUT_HPP

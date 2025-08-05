//
// Created by ph431 on 2025/2/27.
//

#ifndef RAYTRIXLAYOUT_HPP
#define RAYTRIXLAYOUT_HPP


namespace mca::MI {
    class RaytrixLayout: public AbstractMILayout {
    private:
        cv::PointF center;
        cv::PointF ltop;
        float rotation;

        float x_unit_offset;
        float y_unit_offset;

        cv::PointF col_header_offset;

        void calculateLeftTop()
        {
            const float radius = diameter / 2;
            const int center_y_id = static_cast<int>((center.getY() - radius) / y_unit_offset);

            cv::PointF center_top = center - cv::PointF(0, static_cast<float>(center_y_id) * y_unit_offset);
            if (center_top.getY() >= diameter)
                center_top = center_top - cv::PointF(x_unit_offset, 0.5f * diameter);

            int gaps = static_cast<int>(center_top.getX() / x_unit_offset);
            ltop = center_top - cv::PointF(static_cast<float>(gaps) * x_unit_offset, 0);

            if (ltop.getX() < 0.5f * diameter) gaps--;
            if (gaps % 2 == 0)
            {
                ltop = center_top - cv::PointF(static_cast<float>(gaps) * x_unit_offset, 0);
                col_header_offset = cv::PointF(0, 0.5f * diameter);
            }
            else{
                ltop = ltop = center_top - cv::PointF(static_cast<float>(gaps) * x_unit_offset, -0.5f * diameter);
                col_header_offset = cv::PointF(0, -0.5f * diameter);
            }
        }

        void calculateRowCol()
        {
            cols = static_cast<int>((static_cast<float>(width) - ltop.getX()) / x_unit_offset) + 1;

            cv::PointF rtop = ltop + cv::PointF(static_cast<float>(cols-1) * x_unit_offset, 0);
            if (rtop.getX() + diameter / 2 > static_cast<float>(width)) cols--;

            rows = static_cast<int>((static_cast<float>(height) - ltop.getY()) / y_unit_offset) + 1;
            cv::PointF lbot = ltop + cv::PointF(0, static_cast<float>(rows - 1) * y_unit_offset);
            if (lbot.getY() + diameter / 2 > static_cast<float>(height)) rows--;
            first_col_rows = rows;

            cv::PointF second_col_header = ltop + cv::PointF(x_unit_offset, 0) + col_header_offset;
            second_col_rows = static_cast<int>((static_cast<float>(height) - second_col_header.getY()) / y_unit_offset) + 1;
            cv::PointF second_col_tail = second_col_header + cv::PointF(0, static_cast<float>(second_col_rows-1) * y_unit_offset);
            if (second_col_tail.getY() + diameter / 2 > static_cast<float>(height)) second_col_rows--;

            rows = std::max(first_col_rows, second_col_rows);
            layout.resize(rows * cols);
        }

        void calculateLayout() override
        {
            for (int i=0; i < cols; i++)
            {
                int len;
                if (i % 2 == 0) len = first_col_rows;
                else len = second_col_rows;

                for (int j=0; j < len; j++)
                {
                    float x = ltop.getX() + static_cast<float>(i) * x_unit_offset;
                    float y = ltop.getY() + static_cast<float>(j) * y_unit_offset;

                    cv::PointF mi = cv::PointF(x, y);
                    if (i % 2 == 1)
                        mi = mi + col_header_offset;

                    setMI(j, i, mi);
                }
            }
        }

    public:
        RaytrixLayout(const int width, const int height, std::unordered_map<std::string, std::any> config)
        {
            this->width = width;
            this->height = height;
            rotation = std::any_cast<float>(config["rotation"]);
            diameter = std::any_cast<float>(config["diameter"]);

            const auto [offset_x, offset_y] = std::any_cast<std::pair<float, float>>(config["offset"]);

            float center_x = static_cast<float>(width) / 2 + offset_x;
            float center_y = static_cast<float>(height) / 2 - offset_y;

            if (rotation < std::numbers::pi / 4)
            {
                std::swap(this->width, this->height);
                std::swap(center_x, center_y);
            }
            center = cv::Point(center_x, center_y);

            y_unit_offset = diameter;
            x_unit_offset = static_cast<float>(diameter * std::sqrt(3) / 2);

            calculateLeftTop();
            calculateRowCol();
            RaytrixLayout::calculateLayout();
        }

        [[nodiscard]] int getMCAWidth(int patch_size) const override
        {
            if (patch_size % 2 == 1) patch_size++;

            if (rotation < std::numbers::pi / 4) return patch_size * rows;
            else return patch_size * cols;
        }

        [[nodiscard]] int getMCAHeight(int patch_size) const override
        {
            if (patch_size % 2 == 1) patch_size++;

            if (rotation < std::numbers::pi / 4) return patch_size * cols;
            else return patch_size * rows;
        }
    };
    typedef std::shared_ptr<RaytrixLayout> raytrix_layout_ptr;
};

#endif //RAYTRIXLAYOUT_HPP

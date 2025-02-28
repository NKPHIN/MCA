//
// Created by ph431 on 2025/2/26.
//

#ifndef MI_HPP
#define MI_HPP

#include <cmath>
#include "mca/common/cv/Point.hpp"
#include "mca/io/parser/calib.hpp"

namespace mca::MI {
    class MicroImage {
    private:
        float diameter;
        cv::PointF center;

    public:
        MicroImage() {diameter = 0;}
        MicroImage(const float diameter, const cv::PointF center): diameter(diameter), center(center) {}
        MicroImage(const float diameter, const float x, const float y): diameter(diameter), center(x, y) {}

        [[nodiscard]] float getDiameter() const { return diameter;}
        [[nodiscard]] cv::PointF getCenter() const { return center;}
        [[nodiscard]] float getCenterX() const {return center.getX();}
        [[nodiscard]] float getCenterY() const {return center.getY();}
    };


    class AbstractMILayout{
    protected:
        int rows = 0;
        int cols = 0;
        int width = 0;
        int height = 0;
        float diameter = 0;

        int first_col_rows = 0;
        int second_col_rows = 0;

        std::vector<MicroImage> layout;

        void setMI(const int row, const int col, const cv::PointF center)
        {
            layout[row * cols + col] = MicroImage(diameter, center);
        }
        virtual void calculateLayout() = 0;

    public:
        virtual ~AbstractMILayout() = default;

        [[nodiscard]] MicroImage getMI(const int row, const int col) const {return layout[row * cols + col];}

        [[nodiscard]] int getRows() const {return rows;}
        [[nodiscard]] int getCols() const {return cols;}
        [[nodiscard]] int getWidth() const {return width;}
        [[nodiscard]] int getHeight() const {return height;}
        [[nodiscard]] float getDiameter() const {return diameter;}

        [[nodiscard]] int getFirstColRows() const {return first_col_rows;}
        [[nodiscard]] int getSecondColRows() const {return second_col_rows;}

        [[nodiscard]] virtual int getMCAWidth(const float cropRatio) const
        {
            int patch_size = static_cast<int>(diameter * cropRatio);
            if (patch_size % 2 == 1) patch_size++;

            return patch_size * cols;
        }

        [[nodiscard]] virtual int getMCAHeight(const float cropRatio) const
        {
            int patch_size = static_cast<int>(diameter * cropRatio);
            if (patch_size % 2 == 1) patch_size++;
            return patch_size * rows;
        }
    };

    typedef std::shared_ptr<AbstractMILayout> layout_ptr;
};

#endif //MI_HPP

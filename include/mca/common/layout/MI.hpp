//
// Created by ph431 on 2025/2/26.
//

#ifndef MI_HPP
#define MI_HPP

#include <cmath>
#include "mca/common/cv/Point.hpp"
#include "mca/io/parser/calib.hpp"

namespace mca::MI {
    /**
     * @brief The implementation of data type 'MicroImage'
     */
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
        [[nodiscard]] float distance(const cv::PointF &p) const
        {
            const float delta_x = p.getX() - center.getX();
            const float delta_y = p.getY() - center.getY();
            return std::sqrt(delta_x * delta_x + delta_y * delta_y);
        }
    };

    /**
     * @brief The implementation of module 'Microimage Center Localization' and 'Microimage Block Re-localization'
     *
     * @note This class is the base class of both 'TSPCLayout' and 'RaytrixLayout'
     *       You must instantiate the appropriate subclass depend on the sequence type.
     */
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

        /**
         * @brief Return the corresponding Microimage for the given row and column indice.
         *
         * @param row The row index
         * @param col The col index
         * @return Corresponding Microimage, include diameter and center coordinates.
         */
        [[nodiscard]] MicroImage getMI(const int row, const int col) const {return layout[row * cols + col];}

        /**
         * @return The total number of Microimage rows.
         */
        [[nodiscard]] int getRows() const {return rows;}

        /**
         * @return The total number of Microimage columns.
         */
        [[nodiscard]] int getCols() const {return cols;}

        /**
         * @return The width of original lenslet image in pixels.
         */
        [[nodiscard]] int getWidth() const {return width;}

        /**
         * @return The height of original lenslet image in pixels.
         */
        [[nodiscard]] int getHeight() const {return height;}

        /**
         * @return The diameter of Microimage.
         */
        [[nodiscard]] float getDiameter() const {return diameter;}

        /**
         * @brief Since the number of Microimage in odd column and even column may be different,
         *        you need to obtain them respectively.
         *
         * @return The number of Microimage in odd(first) column.
         */
        [[nodiscard]] int getFirstColRows() const {return first_col_rows;}

        /**
         * @return The number of Microimage in even column.
         */
        [[nodiscard]] int getSecondColRows() const {return second_col_rows;}

        /**
         * @param patch_size The MCA patch size in pixels.
         * @return The width of image cropped by MCA module in pixels.
         */
        [[nodiscard]] virtual int getMCAWidth(int patch_size) const
        {
            if (patch_size % 2 == 1) patch_size++;
            return patch_size * cols;
        }

        /**
         * @param patch_size The MCA patch size in pixels.
         * @return The height of image cropped by MCA module in pixels.
         */
        [[nodiscard]] virtual int getMCAHeight(int patch_size) const
        {
            if (patch_size % 2 == 1) patch_size++;
            return patch_size * rows;
        }

        /**
         * @brief Return the corresponding row and column indices depend on Microimage center.
         *
         * @param center The center coordinate of Microimage
         * @return A pair of corresponding row and column indices.
         */
        [[nodiscard]] std::pair<int, int> getMIRowColIndex(const cv::PointF center) const
        {
            const int approx_col = static_cast<int>(center.getX() / (diameter * std::sqrt(3) / 2));
            const int approx_row = static_cast<int>(center.getY() / diameter);

            int i = approx_row, j = approx_col;
            for (i = std::max(0, approx_row - 3); i < std::min(rows, approx_row + 3); i++)
            {
                for (j = std::max(0, approx_col - 3); j < std::min(cols, approx_col + 3); j++)
                {
                    auto mi = getMI(i, j);
                    if (mi.getCenterX() == center.getX() && mi.getCenterY() == center.getY())
                        return std::make_pair(i, j);
                }
            }
            return std::make_pair(i, j);
        }

        /**
         * @brief Returns the coordinates of the MI center closest to the current point.
         * @param cur current point coordinate (x, y)
         * @return closest Microimage center coordinate (cx, cy)
         */
        [[nodiscard]] cv::PointF closestMICenter(const cv::PointF cur) const
        {
            const int approx_col = static_cast<int>(cur.getX() / (diameter * std::sqrt(3) / 2));
            const int approx_row = static_cast<int>(cur.getY() / diameter);

            float dis = diameter * 5;
            cv::PointF center;
            for (int i = std::max(0, approx_row - 3); i < std::min(rows, approx_row + 3); i++)
            {
                for (int j = std::max(0, approx_col - 3); j < std::min(cols, approx_col + 3); j++)
                {
                    auto mi = getMI(i, j);
                    if (mi.getCenterX() == 0 && mi.getCenterY() == 0)
                        continue;
                    if (mi.distance(cur) < dis)
                    {
                        center = mi.getCenter();
                        dis = mi.distance(cur);
                    }
                }
            }

            return center;
        }
    };

    typedef std::shared_ptr<AbstractMILayout> layout_ptr;
};

#endif //MI_HPP

//
// Created by ph431 on 2025/2/26.
//

#ifndef POINT_HPP
#define POINT_HPP
#include <stdexcept>

namespace mca::cv {
    template <typename T>
    class Point {
    private:
        T x;
        T y;

    public:
        Point(): x(0), y(0) {}
        Point(T x, T y): x(x), y(y) {}

        T getX() const { return x;}
        T getY() const { return y;}
        void setX(T x) { this->x = x;}
        void setY(T y) { this->y = y;}

        Point operator+(const Point &other) const
        {
            return Point(x + other.x, y + other.y);
        }

        Point operator-(const Point &other) const
        {
            return Point(x - other.x, y - other.y);
        }

        Point operator*(T scalar) const
        {
            return Point(x * scalar, y * scalar);
        }

        Point operator/(T scalar) const
        {
            if (scalar == 0)
                throw std::invalid_argument("Error: Scalar cannot be zero!");
            return Point(x / scalar, y / scalar);
        }
    };

    typedef mca::cv::Point<int> PointI;
    typedef mca::cv::Point<float> PointF;
}

#endif //POINT_HPP

//
// Created by ph431 on 2025/2/26.
//

#ifndef RECT_HPP
#define RECT_HPP

namespace mca::cv {
    template <typename T>
    class Rect {
    private:
        cv::Point<T> ltop;
        T width;
        T height;

    public:
        Rect(const T x, const T y, const T width, const T height): ltop(x, y), width(width), height(height) {}
        Rect(const cv::Point<T> ltop, const T width, const T height): ltop(ltop), width(width), height(height) {}

        [[nodiscard]] cv::Point<T> o() const { return cv::Point<T>(ltop.getX(), ltop.getY()); }
        [[nodiscard]] cv::Point<T> rtop() const {return cv::Point<T>(ltop.getX() + width, ltop.getY());}
        [[nodiscard]] cv::Point<T> lbot() const {return cv::Point<T>(ltop.getX(), ltop.getY() + height);}
        [[nodiscard]] cv::Point<T> rbot() const {return cv::Point<T>(ltop.getX() + width, ltop.getY() + height);}

        [[nodiscard]] T getWidth() const {return width;}
        [[nodiscard]] T getHeight() const {return height;}
        [[nodiscard]] T getArea() const {return width * height;}
    };

    class Region {
    private:
        cv::Point<int> ltop;
        int width;
        int height;

    public:
        Region() {width = height = 0;}
        explicit Region(const Rect<int> rect)
        {
            ltop = rect.o();
            width = rect.getWidth();
            height = rect.getHeight();
        }
        Region(const cv::Point<int> ltop, const int width, const int height) : ltop(ltop), width(width), height(height) {}
        Region(const int x, const int y, const int width, const int height) : ltop(x, y), width(width), height(height) {}

        [[nodiscard]] cv::Point<int> o() const {return {ltop.getX(), ltop.getY()};}
        [[nodiscard]] cv::Point<int> rtop() const {return {ltop.getX() + width - 1, ltop.getY()};}
        [[nodiscard]] cv::Point<int> lbot() const {return {ltop.getX(), ltop.getY() + height - 1};}
        [[nodiscard]] cv::Point<int> rbot() const {return {ltop.getX() + width - 1, ltop.getY() + height - 1};}

        [[nodiscard]] int getWidth() const {return width;}
        [[nodiscard]] int getHeight() const {return height;}
        [[nodiscard]] int getArea() const {return width * height;}

        [[nodiscard]] bool match(const Region &other) const
        {
            if (other.getWidth() != getWidth() || other.getHeight() != getHeight())
                return false;
            return true;
        }

        [[nodiscard]] bool contains(const Region &sub) const
        {
            int x = ltop.getX();
            int y = ltop.getY();

            if (sub.o().getX() < x) return false;
            if (sub.o().getY() < y) return false;
            if (sub.o().getX() + sub.getWidth() > x + width) return false;
            if (sub.o().getY() + sub.getHeight() > y + height) return false;

            return true;
        }
    };
}

#endif //RECT_HPP

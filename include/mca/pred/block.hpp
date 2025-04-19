//
// Created by ph431 on 2025/4/9.
//

#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <memory>

#include "mca/common/cv/cv2.hpp"
#include "mca/utils/math.hpp"
#include "mca/proc/crop.hpp"

namespace mca::prediction {
    constexpr int MIN_BLOCK_SIZE = 8;
    constexpr int MAX_BLOCK_SIZE = 16;
    constexpr int INTER_PREDICTION = 0;
    constexpr int INTRA_PREDICTION = 1;
    constexpr double PSNR_THRESHOLD = 30.0;

    class BlockNode {
    private:
        int width;
        int height;
        cv::PointI vertex;
        std::vector<std::shared_ptr<BlockNode>> children;

        int mode = INTER_PREDICTION;
        bool is_leaf = false;
        bool is_valid = true;
        std::vector<int> mv;
        double psnr = 0;

    public:
        BlockNode(const cv::PointI vertex, const int width, const int height)
        {
            this->vertex = vertex;
            this->width = width;
            this->height = height;
        }

        [[nodiscard]] int getWidth() const {return this->width;}
        [[nodiscard]] int getHeight() const {return this->height;}
        [[nodiscard]] cv::PointI getVertex() const {return this->vertex;}

        [[nodiscard]] bool isLeaf() const {return this->is_leaf;}
        void setLeaf() {this->is_leaf = true;}

        [[nodiscard]] bool isValid() const {return this->is_valid;}
        void setUnValid() {this->is_valid = false;}

        [[nodiscard]] std::vector<int> getMv() const {return this->mv;}
        void setMv(const std::vector<int> &mv) {this->mv = mv;}

        [[nodiscard]] double getPsnr() const {return this->psnr;}
        void setPsnr(double psnr) {this->psnr = psnr;}

        void appendChild(const std::shared_ptr<BlockNode>& child) {children.push_back(child);}
        [[nodiscard]] std::vector<std::shared_ptr<BlockNode>> getChildren() const {return this->children;}
    };
    typedef std::shared_ptr<BlockNode> BlockNodePtr;

    class AbstractBlockTree {
    protected:
        int rows = 0;
        int cols = 0;
        BlockNodePtr root;

        cv::Mat_C3 cur;
        cv::Mat_C3 cropped;

        int index = 0;
        std::vector<std::vector<int>> mvs;

        bool checkEmptyRegion(const cv::Region region)
        {
            const int cnt = countValidPixel(region);
            const int width = region.getWidth();
            const int height = region.getHeight();

            return 1.0 * cnt / (width * height) < 0.9;
        }

        bool checkFullRegion(const cv::Region region)
        {
            const int cnt = countValidPixel(region);
            const int width = region.getWidth();
            const int height = region.getHeight();

            return cnt == width * height;
        }

        int countValidPixel(const cv::Region region)
        {
            const cv::PointI vertex = region.o();
            const int width = region.getWidth();
            const int height = region.getHeight();

            int cnt = 0;
            for (int x = vertex.getX(); x < vertex.getX() + width; x++)
            {
                for (int y = vertex.getY(); y < vertex.getY() + height; y++)
                {
                    unsigned char ch = cropped[0].at(y, x);
                    if (ch != 0) cnt++;
                }
            }

            return cnt;
        }

        virtual void divideBlock(const BlockNodePtr& parent) = 0;
        [[nodiscard]] virtual std::pair<std::vector<int>, double> search(cv::PointI vertex, int width, int height) = 0;
        virtual void restore(cv::PointI vertex, int width, int height, std::vector<int> mv) = 0;
        virtual bool pruning(const BlockNodePtr& parent, double psnr) = 0;

    public:
        virtual ~AbstractBlockTree() = default;
        virtual void build(const BlockNodePtr& ptr, int level) = 0;
        virtual void rebuild(const BlockNodePtr& ptr, int level) = 0;

        [[nodiscard]] BlockNodePtr getRoot() const {return root;}
        [[nodiscard]] std::vector<std::vector<int>> getMVs() {return mvs;}
        [[nodiscard]] cv::Mat_C3 getCurMat() const {return cur;}

        [[nodiscard]] cv::Mat_C3 getCroppedMat() const {return cropped;}
    };
}

#endif //BLOCK_HPP

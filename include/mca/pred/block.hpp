//
// Created by ph431 on 2025/4/9.
//

#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <memory>
#include "mca/common/cv/cv2.hpp"

namespace mca::prediction {
    constexpr int INTER_PREDICTION = 0;
    constexpr int INTRA_PREDICTION = 1;
    constexpr double PSNR_THRESHOLD = 30.0;

    class BlockNode {
    private:
        int width;
        int height;
        cv::PointI vertex;
        std::vector<std::shared_ptr<BlockNode>> children;

        bool is_leaf = false;
        bool is_valid = true;
        std::vector<int> mv;

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

        void appendChild(const std::shared_ptr<BlockNode>& child) {children.push_back(child);}
        [[nodiscard]] std::vector<std::shared_ptr<BlockNode>> getChildren() const {return this->children;}
    };
    typedef std::shared_ptr<BlockNode> BlockNodePtr;
}

#endif //BLOCK_HPP

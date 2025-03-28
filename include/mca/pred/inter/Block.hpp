//
// Created by ph431 on 2025/3/27.
//

#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <memory>

#include "mca/common/cv/cv2.hpp"
#include "mca/utils/math.hpp"

namespace mca::prediction::inter {
    constexpr int MIN_BLOCK_SIZE = 16;
    constexpr int INTER_PREDICTION = 0;
    constexpr int INTRA_PREDICTION = 1;

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

    class BlockTree {
    private:
        int rows;
        int cols;
        BlockNodePtr root;

        cv::Mat cur;
        cv::Mat pre;
        cv::Mat cropped;

        int index = 0;
        std::vector<std::vector<int>> mvs;

        bool checkRegion(const cv::Region region)
        {
            const cv::PointI vertex = region.o();
            const int width = region.getWidth();
            const int height = region.getHeight();

            for (int x = vertex.getX(); x < vertex.getX() + width; x++)
            {
                for (int y = vertex.getY(); y < vertex.getY() + height; y++)
                {
                    unsigned char ch = cropped.at(y, x);
                    if (ch == 0) return true;
                }
            }
            return false;
        }

        [[nodiscard]] std::vector<int> search(const cv::PointI vertex, const int width, const int height) const
        {
            const int x_start = std::max(0, vertex.getX() - width * 2);
            const int y_start = std::max(0, vertex.getY() - height * 2);

            const int x_end = std::min(cols, vertex.getX() + width * 2);
            const int y_end = std::min(rows, vertex.getY() + height * 2);

            std::vector<int> mv;
            mv.resize(2);

            double final_psnr = 0.0;
            const auto cur_region = cv::Region(vertex, width, height);
            for (int x = x_start; x < x_end; x++)
            {
                for (int y = y_start; y < y_end; y++)
                {
                    const auto search_region = cv::Region(x, y, width, height);

                    cv::Mat cur_block = cur.sub(cur_region);
                    cv::Mat search_block = pre.sub(search_region);

                    const double tmp_psnr = proc::psnr(cur_block, search_block);
                    if (tmp_psnr > final_psnr)
                    {
                        final_psnr = tmp_psnr;
                        mv = {x, y};
                    }
                }
            }

            return {mv[0] - vertex.getX(), mv[1] - vertex.getY()};
        }

    public:
        BlockTree(const cv::Mat& cur, const cv::Mat& pre, const cv::Mat& cropped)
        {
            rows = cur.getRows();
            cols = cur.getCols();

            root = std::make_shared<BlockNode>(cv::PointI(0, 0), cols, rows);
            this->cur = cur;
            this->pre = pre;
            this->cropped = cropped;

            build(root, 1, proc::PRE);
        }

        BlockTree(const cv::Mat& cur, const cv::Mat& pre, const std::vector<std::vector<int>>& mvs)
        {
            rows = cur.getRows();
            cols = cur.getCols();

            root = std::make_shared<BlockNode>(cv::PointI(0, 0), cols, rows);
            this->cur = cur;
            this->pre = pre;
            this->cropped = cur;
            this->mvs = mvs;
        }

        void build(const BlockNodePtr& ptr, const int level, const int mode)
        {
            const cv::PointI vertex = ptr->getVertex();
            const int width = ptr->getWidth();
            const int height = ptr->getHeight();

            const cv::Region region(vertex, width, height);

            if (mode == proc::PRE)
            {
                cv::Mat cur_block = cur.sub(region);
                cv::Mat pre_block = pre.sub(region);

                if (checkRegion(region) == false)
                {
                    ptr->setUnValid();
                    const std::vector cur_block_mv = {-1 * level};
                    mvs.push_back(cur_block_mv);
                    return;
                }
                else if (proc::psnr(cur_block, pre_block) >= 35.0)
                {
                    ptr->setLeaf();
                    ptr->setMv({0, 0});
                    mvs.push_back({level, 0, 0});
                    return;
                }
                else if (width <= MIN_BLOCK_SIZE && height <= MIN_BLOCK_SIZE)
                {
                    ptr->setLeaf();

                    const std::vector cur_block_mv = search(vertex, width, height);
                    ptr->setMv(cur_block_mv);
                    mvs.push_back({level, cur_block_mv[0], cur_block_mv[1]});
                    return;
                }
            }
            else if (mode == proc::POST)
            {
                const int end = mvs[index][0];
                if (level == std::abs(end))
                {
                    if (end > 0)
                    {
                        const cv::PointI src_vertex = vertex + cv::PointI(mvs[index][1], mvs[index][2]);
                        const cv::Region src_region(src_vertex, width, height);
                        cv::copyTo(pre, cur, cropped, src_region,region);
                    }
                    index++;
                    return;
                }
            }

            const int sub_block_width = (width > MIN_BLOCK_SIZE) ? (width / 2) : width;
            const int sub_block_height = (height > MIN_BLOCK_SIZE) ? (height / 2) : height;

            std::vector sub_vertex = {
                vertex,
                cv::PointI(vertex.getX() + sub_block_width, vertex.getY()),
                cv::PointI(vertex.getX(), vertex.getY() + sub_block_height),
                cv::PointI(vertex.getX() + sub_block_width, vertex.getY() + sub_block_height)
            };

            const std::vector<std::pair<int, int>> sub_size = {
                {sub_block_width, sub_block_height},
                {width - sub_block_width, sub_block_height},
                {sub_block_width, height - sub_block_height},
                {width - sub_block_width, height - sub_block_height}
            };

            for (int i=0; i < sub_size.size(); i++)
            {
                const int cur_width = sub_size[i].first;
                const int cur_height = sub_size[i].second;

                if (cur_width <= 0 || cur_height <= 0)
                    continue;
                auto child = std::make_shared<BlockNode>(sub_vertex[i], cur_width, cur_height);
                ptr->appendChild(child);
                build(child, level + 1, mode);
            }
        }

        [[nodiscard]] BlockNodePtr getRoot() const {return root;}
        [[nodiscard]] std::vector<std::vector<int>> getMVs() {return mvs;}
        [[nodiscard]] cv::Mat getCurMat() const {return cur;}
    };
}

#endif //BLOCK_HPP

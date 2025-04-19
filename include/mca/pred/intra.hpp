//
// Created by ph431 on 2025/4/9.
//

#ifndef INTRA_HPP
#define INTRA_HPP
#include "block.hpp"
#include "mca/proc/crop.hpp"

namespace mca::prediction {
    class IntraBlockTree final : public AbstractBlockTree {
    private:
        [[nodiscard]] std::pair<std::vector<int>, double> search(const cv::PointI vertex, const int width, const int height) override
        {
            std::vector<int> mv;
            const std::vector<std::vector<int>> vecs = {{-76, -42}, {-76, 44}, {0, 86}, {76, 44}, {76, -42}, {0, -86}};

            const int cur_x = vertex.getX();
            const int cur_y = vertex.getY();

            const auto total_region = cv::Region(0, 0, cols, rows);
            const auto cur_region = cv::Region(cur_x, cur_y, width, height);
            cv::Mat cur_block = cur[0].sub(cur_region);

            double _psnr = 0;
            for (int i = 0; i < vecs.size(); i++)
            {
                const int offset_x = vecs[i][0];
                const int offset_y = vecs[i][1];

                const int start_x = cur_x + offset_x - 3;
                const int end_x = cur_x + offset_x + 3;
                const int start_y = cur_y + offset_y - 3;
                const int end_y = cur_y + offset_y + 3;

                for (int x = start_x; x <= end_x; x++)
                {
                    for (int y = start_y; y <= end_y; y++)
                    {
                        const auto ref_region = cv::Region(x, y, width, height);
                        if (!total_region.contains(ref_region)) continue;
                        if (!checkFullRegion(ref_region)) continue;

                        cv::Mat ref_block = cropped[0].sub(ref_region);
                        double res = proc::psnr(cur_block, ref_block);
                        if (res > _psnr)
                        {
                            _psnr = res;
                            mv = {i, x - cur_x - offset_x, y - cur_y - offset_y};
                        }
                    }
                }
            }
            return std::make_pair(mv, _psnr);
        }

        void restore(const cv::PointI vertex, const int width, const int height, const std::vector<int> mv) override
        {
            const std::vector<std::vector<int>> vecs = {{-76, -42}, {-76, 44}, {0, 86}, {76, 44}, {76, -42}, {0, -86}};
            const int cur_x = vertex.getX();
            const int cur_y = vertex.getY();
            const auto cur_region = cv::Region(cur_x, cur_y, width, height);

            const int target_x = mv[1] + vecs[mv[0]][0] + cur_x;
            const int target_y = mv[2] + vecs[mv[0]][1] + cur_y;
            const cv::Region target_region(target_x, target_y, width, height);
            cv::copyTo(cropped[0], cropped[0], cropped[0],target_region, cur_region);
        }

        void divideBlock(const BlockNodePtr &parent) override
        {
            const int width = parent->getWidth();
            const int height = parent->getHeight();
            const cv::PointI vertex = parent->getVertex();

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
                parent->appendChild(child);
            }
        }

        bool pruning(const BlockNodePtr &parent, const double psnr) override
        {
            const int width = parent->getWidth();
            const int height = parent->getHeight();
            if (width > MAX_BLOCK_SIZE || height > MAX_BLOCK_SIZE) return false;

            divideBlock(parent);

            std::vector<std::pair<int, double>> vecs;
            for (const auto &child : parent->getChildren())
            {
                const int sub_width = child->getWidth();
                const int sub_height = child->getHeight();
                const cv::PointI vertex = child->getVertex();

                const cv::Region region(vertex, sub_width, sub_height);
                if (checkEmptyRegion(region) == false) continue;

                auto [sub_block_mv, sub_block_psnr] = search(vertex, sub_width, sub_height);
                if (!sub_block_mv.empty())
                {
                    double sub_block_mse = proc::psnr2mse(sub_block_psnr);
                    int pixel_cnt = sub_width * sub_height;
                    vecs.emplace_back(pixel_cnt, sub_block_mse);
                }
            }

            double mse = 0;
            int cnt = 0;
            for (auto [fst, snd] : vecs)
            {
                mse += fst * snd;
                cnt += fst;
            }
            if (cnt > 0)
            {
                mse /= cnt;
                double sub_psnr = proc::mse2psnr(mse);
                return sub_psnr - psnr < 0.5;
            }
            return true;
        }

    public:
        IntraBlockTree(const cv::Mat_C3& cur, const cv::Mat_C3& cropped)
        {
            this->cur = cur;
            this->cropped = cropped;

            rows = cur[0].getRows();
            cols = cur[0].getCols();
            root = std::make_shared<BlockNode>(cv::PointI(0, 0), cols, rows);

            IntraBlockTree::build(root, 0);
            // std::cout << proc::psnr(this->cur[0], this->cropped[0]) << std::endl;
        }

        IntraBlockTree(const cv::Mat_C3& cropped, const std::vector<std::vector<int>>& mvs)
        {
            this->cropped = cropped;
            this->mvs = mvs;

            rows = cropped[0].getRows();
            cols = cropped[0].getCols();
            root = std::make_shared<BlockNode>(cv::PointI(0, 0), cols, rows);

            IntraBlockTree::rebuild(root, 0);
        }

        void build(const BlockNodePtr &ptr, int level) override
        {
            const cv::PointI vertex = ptr->getVertex();
            const int width = ptr->getWidth();
            const int height = ptr->getHeight();

            if (width <= MAX_BLOCK_SIZE && height <= MAX_BLOCK_SIZE)
            {
                const cv::Region region(vertex, width, height);
                if (checkEmptyRegion(region) == false)
                {
                    ptr->setUnValid();
                    return;
                }

                auto [cur_block_mv, cur_block_psnr] = search(vertex, width, height);
                if ((width <= MIN_BLOCK_SIZE && height <= MIN_BLOCK_SIZE) || cur_block_psnr >= PSNR_THRESHOLD || pruning(ptr, cur_block_psnr))
                {
                    ptr->setLeaf();
                    ptr->setMv(cur_block_mv);
                    if (!cur_block_mv.empty())
                    {
                        restore(vertex, width, height, cur_block_mv);
                        mvs.push_back({level, cur_block_mv[0], cur_block_mv[1], cur_block_mv[2]});
                    }
                    else mvs.push_back({level, 6});
                    return;
                }
            }

            if (ptr->getChildren().empty())
                divideBlock(ptr);
            for (const auto& child : ptr->getChildren())
            {
                if (width <= MAX_BLOCK_SIZE && height <= MAX_BLOCK_SIZE)
                    build(child, level + 1);
                else build(child, level);
            }
        }

        void rebuild(const BlockNodePtr &ptr, int level) override
        {
            const cv::PointI vertex = ptr->getVertex();
            const int width = ptr->getWidth();
            const int height = ptr->getHeight();

            if (width <= MAX_BLOCK_SIZE && height <= MAX_BLOCK_SIZE)
            {
                const cv::Region region(vertex, width, height);
                if (checkEmptyRegion(region) == false)
                {
                    ptr->setUnValid();
                    return;
                }

                if (level == mvs[index][0])
                {
                    if (const int dir = mvs[index][1]; dir < 6)
                    {
                        const int offset_x = mvs[index][2];
                        const int offset_y = mvs[index][3];
                        const std::vector cur_block_mv = {dir, offset_x, offset_y};

                        ptr->setLeaf();
                        ptr->setMv(cur_block_mv);
                        restore(vertex, width, height, cur_block_mv);
                    }
                    index++;
                    return;
                }
            }

            divideBlock(ptr);
            for (const auto& child : ptr->getChildren())
            {
                if (width <= MAX_BLOCK_SIZE && height <= MAX_BLOCK_SIZE)
                    rebuild(child, level + 1);
                else rebuild(child, level);
            }
        }
    };
}

#endif //INTRA_HPP

//
// Created by ph431 on 2025/4/9.
//

#ifndef INTRA_HPP
#define INTRA_HPP

#include "block.hpp"
#include "mca/utils/math.hpp"

namespace mca::prediction {
    class BlockMap {
    private:
        int rows = 0;
        int cols = 0;

        int MIN_BLOCK_SIZE = 0;
        int MAX_BLOCK_SIZE = 0;

        cv::Mat_C3 raw;
        cv::Mat_C3 mca;

        std::vector<std::vector<std::vector<int>>> mvs;
        std::vector<std::vector<int>> vecs;
        std::vector<std::vector<BlockNodePtr>> blocks;

        void init()
        {
            const int width = mca[0].getCols();
            const int height = mca[0].getRows();

            rows = std::ceil(1.0 * height / MAX_BLOCK_SIZE);
            cols = std::ceil(1.0 * width / MAX_BLOCK_SIZE);

            blocks.resize(rows);
            for (int i = 0; i < rows; i++)
                blocks[i].resize(cols);
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
                    unsigned char ch = mca[0].at(y, x);
                    if (ch != 0) cnt++;
                }
            }
            return cnt;
        }

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

        std::pair<std::vector<int>, double> search(const cv::Region region)
        {
            std::vector<int> mv;

            const cv::PointI vertex = region.o();
            const int cur_x = vertex.getX();
            const int cur_y = vertex.getY();

            const int width = region.getWidth();
            const int height = region.getHeight();

            const auto total_region = cv::Region(0, 0, raw[0].getCols(), raw[0].getRows());
            cv::Mat cur_block = raw[0].sub(region);

            double psnr = 0;
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

                        cv::Mat ref_block = mca[0].sub(ref_region);
                        if (const double tmp = proc::PSNR(cur_block, ref_block); tmp > psnr)
                        {
                            psnr = tmp;
                            mv = {i, x - cur_x - offset_x, y - cur_y - offset_y};
                        }
                    }
                }
            }

            std::vector<cv::Region> adjacent = {
                cv::Region(cur_x - MAX_BLOCK_SIZE, cur_y, width, height),
                cv::Region(cur_x, cur_y - MAX_BLOCK_SIZE, width, height),
                cv::Region(cur_x - MAX_BLOCK_SIZE, cur_y - MAX_BLOCK_SIZE, width, height)
            };

            for (int i = 0; i < adjacent.size(); i++)
            {
                auto adj = adjacent[i];
                if (adj.o().getX() < 0 || adj.o().getY() < 0)
                    continue;
                cv::Mat ref_block = mca[0].sub(adj);
                if (const double tmp = proc::PSNR(cur_block, ref_block); tmp > psnr)
                {
                    psnr = tmp;
                    mv = {7, i, 0};
                }
            }

            return std::make_pair(mv, psnr);
        }

        // std::pair<std::vector<int>, double> search(const cv::Region region)
        // {
        //     std::vector<int> mv;
        //
        //     const cv::PointI vertex = region.o();
        //     const int cur_x = vertex.getX();
        //     const int cur_y = vertex.getY();
        //
        //     const int width = region.getWidth();
        //     const int height = region.getHeight();
        //
        //     const auto total_region = cv::Region(0, 0, raw[0].getCols(), raw[0].getRows());
        //     cv::Mat cur_block = raw[0].sub(region);
        //
        //     double psnr = 0;
        //     const int start_x = cur_x - 30;
        //     const int end_x = cur_x + 30;
        //     const int start_y = cur_y - 30;
        //     const int end_y = cur_y + 30;
        //
        //     for (int x = start_x; x <= end_x; x++)
        //     {
        //         for (int y = start_y; y <= end_y; y++)
        //         {
        //             const auto ref_region = cv::Region(x, y, width, height);
        //             if (!total_region.contains(ref_region)) continue;
        //             if (!checkFullRegion(ref_region)) continue;
        //
        //             cv::Mat ref_block = mca[0].sub(ref_region);
        //             if (const double tmp = proc::PSNR(cur_block, ref_block); tmp > psnr)
        //             {
        //                 psnr = tmp;
        //                 mv = {x - cur_x, y - cur_y};
        //             }
        //         }
        //     }
        //     return std::make_pair(mv, psnr);
        // }

        void restore(const cv::PointI vertex, const int width, const int height, const std::vector<int>& mv)
        {
            const int cur_x = vertex.getX();
            const int cur_y = vertex.getY();
            const auto cur_region = cv::Region(cur_x, cur_y, width, height);
            cv::Region target_region;

            if (mv[0] < 6)
            {
                const int target_x = mv[1] + vecs[mv[0]][0] + cur_x;
                const int target_y = mv[2] + vecs[mv[0]][1] + cur_y;
                target_region = cv::Region(target_x, target_y, width, height);
            }
            else if (mv[0] == 7)
            {
                const std::vector<cv::Region> adjacent = {
                    cv::Region(cur_x - MAX_BLOCK_SIZE, cur_y, width, height),
                    cv::Region(cur_x, cur_y - MAX_BLOCK_SIZE, width, height),
                    cv::Region(cur_x - MAX_BLOCK_SIZE, cur_y - MAX_BLOCK_SIZE, width, height)
                };
                target_region = adjacent[mv[1]];
            }
            for (int i=0; i<1; i++)
                cv::copyTo(mca[i], mca[i], mca[i],target_region, cur_region);
        }

        // void restore(const cv::PointI vertex, const int width, const int height, const std::vector<int>& mv)
        // {
        //     const int cur_x = vertex.getX();
        //     const int cur_y = vertex.getY();
        //     const auto cur_region = cv::Region(cur_x, cur_y, width, height);
        //
        //     const int target_x = mv[0] + cur_x;
        //     const int target_y = mv[1] + cur_y;
        //     const auto target_region = cv::Region(target_x, target_y, width, height);
        //
        //     for (int i=0; i<3; i++)
        //         cv::copyTo(mca[i], mca[i], mca[i],target_region, cur_region);
        // }

        void divideBlocks(const BlockNodePtr& parent) const
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

        static std::vector<int> readIntraMVData(const std::string& bitstream, int& pos)
        {
            std::vector<int> vec;
            const std::bitset<3> b_dir(bitstream.substr(pos, 3));

            if (const int dir = static_cast<int>(b_dir.to_ulong()); dir < 6)
            {
                const std::bitset<3> b_offset_x(bitstream.substr(pos+3, 3));
                const std::bitset<3> b_offset_y(bitstream.substr(pos+6, 3));

                const int offset_x = static_cast<int>(b_offset_x.to_ulong()) - 3;
                const int offset_y = static_cast<int>(b_offset_y.to_ulong()) - 3;
                vec = {dir, offset_x, offset_y};
                pos += 9;
            }
            else if (dir == 7)
            {
                const std::bitset<2> b_index(bitstream.substr(pos+3, 2));
                const int index = static_cast<int>(b_index.to_ulong());
                vec = {dir, index};
                pos += 5;
            }
            else pos += 3;

            return vec;
        }

    public:
        BlockMap(const cv::Mat_C3 &raw_image, const cv::Mat_C3 &mca_image, const std::vector<std::vector<int>> &vecs, const std::pair<int, int> &block_size)
        {
            this->raw = raw_image;
            this->mca = mca_image;
            this->vecs = vecs;

            this->MIN_BLOCK_SIZE = block_size.first;
            this->MAX_BLOCK_SIZE = block_size.second;

            init();
        }

        BlockMap(const cv::Mat_C3 &mca_image, const std::vector<std::vector<int>> &vecs, const std::pair<int, int> &block_size)
        {
            this->mca = mca_image;
            this->vecs = vecs;

            this->MIN_BLOCK_SIZE = block_size.first;
            this->MAX_BLOCK_SIZE = block_size.second;

            init();
        }

        void build()
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    const cv::PointI vertex(j * MAX_BLOCK_SIZE, i * MAX_BLOCK_SIZE);

                    const int width = std::min((j + 1) * MAX_BLOCK_SIZE, mca[0].getCols()) - j * MAX_BLOCK_SIZE;
                    const int height = std::min((i + 1) * MAX_BLOCK_SIZE, mca[0].getRows()) - i * MAX_BLOCK_SIZE;

                    const cv::Region region(vertex, width, height);
                    blocks[i][j] = std::make_shared<BlockNode>(vertex, width, height);

                    if (!checkEmptyRegion(region)) continue;

                    if (auto [mv, psnr] = search(region); psnr >= PSNR_THRESHOLD)
                    {
                        restore(vertex, width, height, mv);
                        mvs.push_back({{0, mv[0], mv[1], mv[2]}});
                        continue;
                    }

                    divideBlocks(blocks[i][j]);
                    std::vector<std::vector<int>> group;
                    for (const auto& child : blocks[i][j]->getChildren())
                    {
                        const cv::PointI child_vertex = child->getVertex();
                        const int child_width = child->getWidth();
                        const int child_height = child->getHeight();

                        const cv::Region child_region(child_vertex, child_width, child_height);
                        if (auto [child_mv, child_psnr] = search(child_region); !child_mv.empty())
                        {
                            restore(child_vertex, child_width, child_height, child_mv);
                            group.push_back({1, child_mv[0], child_mv[1], child_mv[2]});
                        }
                        else group.push_back({1, 6});
                    }
                    mvs.push_back(group);
                }
            }
        }

        void rebuild(const std::string& bitstream)
        {
            int pos = 0;
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    const cv::PointI vertex(j * MAX_BLOCK_SIZE, i * MAX_BLOCK_SIZE);

                    const int width = std::min((j + 1) * MAX_BLOCK_SIZE, mca[0].getCols()) - j * MAX_BLOCK_SIZE;
                    const int height = std::min((i + 1) * MAX_BLOCK_SIZE, mca[0].getRows()) - i * MAX_BLOCK_SIZE;

                    const cv::Region region(vertex, width, height);
                    blocks[i][j] = std::make_shared<BlockNode>(vertex, width, height);

                    if (!checkEmptyRegion(region)) continue;

                    std::bitset<1> b_layer(bitstream[pos++]);
                    if (const int layer = static_cast<int>(b_layer.to_ulong()); layer == 0)
                    {
                        std::vector<int> vec = readIntraMVData(bitstream, pos);
                        if (!vec.empty()) restore(vertex, width, height, vec);
                    }
                    else if (layer == 1)
                    {
                        divideBlocks(blocks[i][j]);
                        for (const auto& child : blocks[i][j]->getChildren())
                        {
                            const cv::PointI child_vertex = child->getVertex();
                            const int child_width = child->getWidth();
                            const int child_height = child->getHeight();

                            std::vector<int> vec = readIntraMVData(bitstream, pos);
                            if (!vec.empty()) restore(child_vertex, child_width, child_height, vec);
                        }
                    }
                }
            }
        }

        cv::Mat_C3 getMca() {return mca;}
        std::vector<std::vector<std::vector<int>>> getMvs() {return mvs;}
    };
}

#endif //INTRA_HPP

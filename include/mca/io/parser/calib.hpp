//
// Created by ph431 on 2025/2/25.
//

#ifndef CALIB_HPP
#define CALIB_HPP

#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <regex>


namespace mca::parser::Calibration {
    class CalibNode {
    private:
        std::string key;
        std::string val;
        std::shared_ptr<CalibNode> parent;
        std::vector<std::shared_ptr<CalibNode>> children;

    public:
        CalibNode(const std::string& key, const std::shared_ptr<CalibNode> &parent)
        {
            this->key = key;
            this->parent = parent;
        }

        CalibNode(const std::string& key, const std::string& val, const std::shared_ptr<CalibNode> &parent)
        {
            this->key = key;
            this->val = val;
            this->parent = parent;
        }

        void appendChild(const std::shared_ptr<CalibNode>& child)
        {
            children.push_back(child);
        }

        std::shared_ptr<CalibNode> getParent()
        {
            return parent;
        }

        std::pair<std::string, std::string> getKeyAndVal()
        {
            return std::make_pair(key, val);
        }

        std::vector<std::shared_ptr<CalibNode>>  getChildren()
        {
            return children;
        }
    };

    typedef std::shared_ptr<CalibNode> CalibNodePtr;
    class CalibParser {
    private:
        std::string path;
        CalibNodePtr root;

        CalibNodePtr DFS(const CalibNodePtr &ptr, const std::string &key)
        {
            if (ptr->getKeyAndVal().first == key)
                return ptr;

            auto children = ptr->getChildren();
            if (!children.empty())
            {
                for (const auto& child : children)
                {
                    CalibNodePtr res = DFS(child, key);
                    if (res != nullptr) return res;
                }
                return nullptr;
            }

            return nullptr;
        }

    public:
        explicit CalibParser(const std::string& path)
        {
            this->path = path;
            this->root = std::make_shared<CalibNode>("", nullptr);
        }

        [[nodiscard]] int load() const
        {
            std::ifstream file(path);
            if (!file.is_open())
            {
                std::cerr << "Error: Unable to open calib file! " << std::endl;
                return -1;
            }

            std::string line;
            auto cur = root;
            while (std::getline(file, line))
            {
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);

                if (line.empty() || line[0] == '#')
                    continue;

                std::regex pattern1(R"(<(\w+)[^>]*>(-?[\d.]+)</\1>)");
                std::regex pattern2(R"(<(\w+)[^>]*>)");
                std::regex pattern3(R"(</(\w+)[^>]*>)");
                std::smatch match;

                if (std::regex_search(line.cbegin(), line.cend(), match, pattern1))
                {
                    auto calib_node = std::make_shared<CalibNode>(match[1], match[2], cur);
                    cur->appendChild(calib_node);
                }
                else if (std::regex_search(line.cbegin(), line.cend(), match, pattern2))
                {
                    std::string key = cur->getKeyAndVal().first;

                    auto calib_node = std::make_shared<CalibNode>(match[1], cur);
                    cur->appendChild(calib_node);
                    cur = calib_node;
                }
                else if (std::regex_search(line.cbegin(), line.cend(), match, pattern3))
                {
                    std::string key = cur->getKeyAndVal().first;

                    if (key == match[1]) cur = cur->getParent();
                }
            }
            return 0;
        }

        [[nodiscard]] std::string type() const
        {
            const CalibNodePtr header = root->getChildren()[0];
            return header->getKeyAndVal().first;
        }

        std::string search(const std::string& key)
        {
            CalibNodePtr res = DFS(root, key);
            if (res == nullptr) return "";

            return res->getKeyAndVal().second;
        }

        std::string search(const std::string& key_father, const std::string& key)
        {
            const CalibNodePtr father = DFS(root, key_father);
            if (father ==  nullptr) return "";

            const CalibNodePtr res = DFS(father, key);
            if (res == nullptr) return "";
            return res->getKeyAndVal().second;
        }
    };
}

#endif //CALIB_HPP

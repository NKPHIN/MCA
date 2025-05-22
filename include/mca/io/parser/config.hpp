//
// Created by ph431 on 2025/2/25.
//

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <fstream>
#include <iostream>
#include <unordered_map>

namespace mca::parser {
    class ConfigParser {
    private:
        std::string path;
        std::unordered_map<std::string, std::string> config;

    public:
        explicit ConfigParser(const std::string& path)
        {
            this->path = path;
        }

        ~ConfigParser() = default;

        int load()
        {
            std::ifstream configFile(path);
            if (!configFile.is_open())
            {
                std::cerr << "Error: Unable to open config file!" << std::endl;
                return -1;
            }

            std::string line;
            while (std::getline(configFile, line))
            {
                // 跳过空行和注释行
                if (line.empty() || line[0] == '#') {
                    continue;
                }

                // 找到注释的起始位置（如果存在）
                size_t commentPos = line.find('#');
                if (commentPos != std::string::npos) {
                    line = line.substr(0, commentPos);  // 去掉注释部分
                }

                // 分割键和值
                size_t pos = line.find(':');
                if (pos != std::string::npos)
                {
                    std::string key = line.substr(0, pos);
                    std::string value = line.substr(pos + 1);

                    // 去掉键和值的空白字符
                    key.erase(0, key.find_first_not_of(" \t"));
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);

                    if (!value.empty() && value.back() == '\r') {
                        value.pop_back(); // 删除最后一个字符
                    }

                    // 存储键值对
                    config[key] = value;
                }
            }
            return 0;
        }

        std::string get(const std::string& key)
        {
            return config[key];
        }

        void print()
        {
            for (const auto& [key, value] : config)
                std::cout << key << " = " << value << std::endl;
        }
    };
}

#endif //CONFIG_HPP

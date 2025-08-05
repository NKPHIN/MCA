//
// Created by ph431 on 2025/8/5.
//

#ifndef CONFIGLOADER_HPP
#define CONFIGLOADER_HPP

#include <any>
#include <vector>
#include <string>
#include <unordered_map>

#include "mca/io/parser/config.hpp"
#include "mca/common/pipeline/module.hpp"

namespace mca::module::common {
    typedef std::unordered_map<std::string, std::any> Dict;

    class ConfigLoader final : public Module<Dict, std::string> {
    public:
        Dict exec(std::string path) override
        {
            parser::ConfigParser parser(path);

            if (parser.load() == -1)
                throw std::runtime_error("Failed to load config file: " + std::string(path));

            Dict config;

            // File I/O params
            config["input"] = parser.get("InputFile");
            config["output"] = parser.get("OutputFile");
            config["metadata"] = parser.get("MetaDataFile");

            // Sequence params
            config["type"] = std::stoi(parser.get("Type"));
            config["width"] = std::stoi(parser.get("SourceWidth"));
            config["height"] = std::stoi(parser.get("SourceHeight"));
            config["frames"] = std::stoi(parser.get("FramesToBeEncoded"));

            // MCA params
            config["mode"] = std::stoi(parser.get("mode"));
            config["patch"] = std::stoi(parser.get("patch"));
            config["vectors"] = str2vec(parser.get("vectors"));
            config["optimize"] = std::stoi(parser.get("optimize"));
            config["shuffle"] = std::stoi(parser.get("shuffle"));

            // Layout params
            config["diameter"] = std::stof(parser.get("diameter"));
            config["rotation"] = std::stof(parser.get("rotation"));

            if (std::any_cast<int>(config["type"]) == 1)
            {
                config["ltop"] = str2tuple(parser.get("ltop"));
                config["rtop"] = str2tuple(parser.get("rtop"));
                config["lbot"] = str2tuple(parser.get("lbot"));
                config["rbot"] = str2tuple(parser.get("rbot"));
            }
            else config["offset"] = str2tuple(parser.get("offset"));

            return std::move(config);
        }

    private:
        static std::pair<float, float> str2tuple(const std::string& str)
        {
            const std::regex pattern(R"(\[([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\])");
            std::smatch match;

            if (std::regex_search(str, match, pattern) && match.size() == 3) {
                float a = std::stof(match[1].str());
                float b = std::stof(match[2].str());
                return {a, b};
            }
            else throw std::invalid_argument("Invalid format. Expected format: [a, b]");
        }

        static std::vector<std::vector<int>> str2vec(const std::string& str)
        {
            std::vector<std::vector<int>> result;
            const std::regex pattern(R"(\[(-?\d+),\s*(-?\d+)\])");
            std::smatch match;

            auto searchStart(str.cbegin());
            while (std::regex_search(searchStart, str.cend(), match, pattern)) {
                std::vector<int> innerVec;
                innerVec.push_back(std::stoi(match[1].str()));
                innerVec.push_back(std::stoi(match[2].str()));
                result.push_back(innerVec);
                searchStart = match.suffix().first;
            }

            return result;
        }
    };
}

#endif //CONFIGLOADER_HPP

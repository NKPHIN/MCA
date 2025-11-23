//
// Created by ph431 on 2025/8/5.
//

#pragma once


namespace mca::module::decoder {
    struct MetaDataResult {
        std::vector<std::vector<std::vector<int>>> all_vecs;
        std::vector<std::vector<double>> a;
        std::vector<std::vector<double>> b;
    };

    class MetaDataLoader final : public Module<MetaDataResult, common::Dict> {
    public:
        MetaDataResult exec(common::Dict config) override
        {
            const auto path = std::any_cast<std::string>(config["metadata"]);

            const auto frames = std::any_cast<int>(config["frames"]);

            parser::ConfigParser parser(path);
            if (parser.load() == -1)
                throw std::runtime_error("Failed to load metadata file: " + std::string(path));

            MetaDataResult res;
            std::vector<std::vector<std::vector<int>>> all_vecs;
            std::vector<std::vector<double>> a, b;
            for (int i = 0; i < frames; i++)
            {
                auto vecs = readVectorData(parser, i);
                auto theta = readMetaData(parser, i);
                all_vecs.push_back(vecs);
                a.push_back(theta.first);
                b.push_back(theta.second);
            }
            res.all_vecs = all_vecs;
            res.a = a;
            res.b = b;
            return res;
        }

    private:
        static std::vector<std::vector<int>> readVectorData(mca::parser::ConfigParser& log_parser, const int index)
        {
            std::vector<std::vector<int>> vecs(6, std::vector<int>(2));

            const std::string data = log_parser.get("Vector" + std::to_string(index));
            std::stringstream stream(data);
            std::string token;

            int idx = 0;
            while (std::getline(stream, token, ',')) { // 以逗号为分隔符
                vecs[idx / 2][idx % 2] = stoi(token);
                idx++;
            }
            return vecs;
        }

        static std::pair<std::vector<double>, std::vector<double>> readMetaData(mca::parser::ConfigParser& log_parser, const int index)
        {
            std::vector<double> a, b;
            a.resize(100, 1.0);
            b.resize(100, 1.0);

            const std::string data = log_parser.get("Frame" + std::to_string(index));
            std::stringstream stream(data);
            std::string token;

            int start = 0, cnt = 0;
            while (std::getline(stream, token, ',')) { // 以逗号为分隔符
                if (start == 0)
                    start = std::stoi(token);
                else if (cnt % 2 == 0 )
                {
                    a[start] = std::stod(token);
                    cnt++;
                }
                else
                {
                    b[start++] = std::stod(token);
                    cnt++;
                }
            }
            return {a, b};
        }
    };
}

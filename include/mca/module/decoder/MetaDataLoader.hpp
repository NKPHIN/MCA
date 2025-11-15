//
// Created by ph431 on 2025/8/5.
//

#pragma once


namespace mca::module::decoder {
    class MetaDataLoader final : public Module<std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>>, common::Dict> {
    public:
        std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> exec(common::Dict config) override
        {
            const auto path = std::any_cast<std::string>(config["metadata"]);

            const auto frames = std::any_cast<int>(config["frames"]);

            parser::ConfigParser parser(path);
            if (parser.load() == -1)
                throw std::runtime_error("Failed to load metadata file: " + std::string(path));

            std::vector<std::vector<double>> a, b;
            for (int i = 0; i < frames; i++)
            {
                auto theta = readMetaData(parser, i);
                a.push_back(theta.first);
                b.push_back(theta.second);
            }

            return {a, b};
        }

    private:
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

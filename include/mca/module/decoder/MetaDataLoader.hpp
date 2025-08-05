//
// Created by ph431 on 2025/8/5.
//

#pragma once


namespace mca::module::decoder {
    class MetaDataLoader final : public Module<std::vector<std::vector<double>>, common::Dict> {
    public:
        std::vector<std::vector<double>> exec(common::Dict config) override
        {
            const auto path = std::any_cast<std::string>(config["metadata"]);

            const auto frames = std::any_cast<int>(config["frames"]);

            parser::ConfigParser parser(path);
            if (parser.load() == -1)
                throw std::runtime_error("Failed to load metadata file: " + std::string(path));

            std::vector<std::vector<double>> thetas;
            for (int i = 0; i < frames; i++)
            {
                std::vector<double> theta = readMetaData(parser, i);
                thetas.push_back(theta);
            }

            return std::move(thetas);
        }

    private:
        static std::vector<double> readMetaData(mca::parser::ConfigParser& log_parser, const int index)
        {
            std::vector<double> theta;
            theta.resize(100, 1.0);

            const std::string data = log_parser.get("Frame" + std::to_string(index));
            std::stringstream stream(data);
            std::string token;

            int start = 0;
            while (std::getline(stream, token, ',')) { // 以逗号为分隔符
                if (start == 0)
                    start = std::stoi(token);
                else theta[start++] = std::stod(token);
            }
            return theta;
        }
    };
}

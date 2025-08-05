//
// Created by ph431 on 2025/8/5.
//

#ifndef METADATAWRITER_HPP
#define METADATAWRITER_HPP

#include <iomanip>

namespace mca::module::encoder {
    class MetaDataWriter final : public Module<void, std::vector<std::vector<double>>, common::Dict> {
    public:

        void exec(const std::vector<std::vector<double>> thetas, common::Dict config) override {
            const auto frames = std::any_cast<int>(config["frames"]);

            const auto path = std::any_cast<std::string>(config["metadata"]);

            const auto opt = std::any_cast<int>(config["optimize"]);

            writeBasicData(path);

            if (opt == 1) writeMetaData(thetas, path, frames);
        }

    private:
        static void writeBasicData(const std::string& path)
        {
            auto ofs = std::ofstream(path, std::ios::app);
            ofs.close();
        }

        static void writeMetaData(const std::vector<std::vector<double>> &thetas, const std::string& log_path, const int frames)
        {
            auto ofs = std::ofstream(log_path, std::ios::app);

            for (int k=0; k<frames; k++)
            {
                const auto& theta = thetas[k];

                ofs << "Frame" << k << ":";
                bool first = true;
                for (int i = 0; i < 100; i++)
                {
                    if (theta[i] != -1)
                    {
                        if (first)
                        {
                            first = false;
                            ofs << i << ",";
                        }
                        ofs << std::fixed << std::setprecision(3) << theta[i] << ",";
                        // ofs << theta[i] << ",";
                    }
                }
                ofs << std::endl;
            }
            ofs.close();
        }
    };
}

#endif //METADATAWRITER_HPP

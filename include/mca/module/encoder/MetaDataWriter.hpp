//
// Created by ph431 on 2025/8/5.
//

#ifndef METADATAWRITER_HPP
#define METADATAWRITER_HPP

#include <iomanip>

namespace mca::module::encoder {
    class MetaDataWriter final : public Module<void, std::vector<std::vector<double>>, std::vector<std::vector<double>>, common::Dict, MI::layout_ptr> {
    public:

        void exec(const std::vector<std::vector<double>> a, const std::vector<std::vector<double>> b, common::Dict config, const MI::layout_ptr layout) override {
            const auto frames = std::any_cast<int>(config["frames"]);

            const auto path = std::any_cast<std::string>(config["metadata"]);

            const auto opt = std::any_cast<int>(config["optimize"]);

            writeBasicData(path, config, layout);

            if (opt == 1) writeMetaData(a, b, path, frames);
        }

    private:
        static void writeBasicData(const std::string& path, common::Dict& config, const MI::layout_ptr& layout)
        {
            auto ofs = std::ofstream(path, std::ios::app);
            const auto patch = std::any_cast<int>(config["patch"]);

            ofs << "Width : " << layout->getMCAWidth(patch) << std::endl;
            ofs << "Height : " << layout->getMCAHeight(patch) << std::endl;
            ofs.close();
        }

        static void writeMetaData(const std::vector<std::vector<double>> & a_list, const std::vector<std::vector<double>> & b_list, const std::string& log_path, const int frames)
        {
            auto ofs = std::ofstream(log_path, std::ios::app);

            for (int k=0; k<frames; k++)
            {
                const auto& a = a_list[k];
                const auto& b = b_list[k];

                ofs << "Frame" << k << ":";
                bool first = true;
                for (int i = 0; i < 100; i++)
                {
                    if (a[i] != -1000)
                    {
                        if (first)
                        {
                            first = false;
                            ofs << i << ",";
                        }
                        ofs << std::fixed << std::setprecision(2) << a[i] << "," << b[i] << ",";
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

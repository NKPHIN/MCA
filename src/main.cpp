#include <iostream>

#include "mca/io/parser/parser.hpp"
#include "mca/module/common/ConfigLoader.hpp"
#include "mca/module/pipeline.hpp"

using namespace mca::module;
using namespace mca;


void encoder_workflow(common::Dict config)
{
    encoder::YUV420Loader yuv420_loader;
    const auto raw_video = yuv420_loader.exec(config);

    common::LocalizationModule localization;
    const auto layout = localization.exec(config);

    const auto frames = std::any_cast<int>(config["frames"]);
    const auto opt = std::any_cast<int>(config["optimize"]);
    const auto init_vecs = std::any_cast<std::vector<std::vector<int>>>(config["vectors"]);

    std::vector<cv::Mat_C3> output_video;
    std::vector<std::vector<double>> a, b;
    std::vector<std::vector<std::vector<int>>> all_vecs;
    const std::vector<int> offsets = {-2, -1, 0, 1, 2};
    for (int i = 0; i < frames; i++)
    {
        encoder::MCAModule mca;
        const auto mca_frame = mca.exec(raw_video[i], layout, config);
        output_video.push_back(mca_frame);

        common::RelocalizationModule reloc;
        const auto reloc_frame = reloc.exec(mca_frame, layout, config);

        common::EstimationModule estimation;
        auto recon_frame = estimation.exec(reloc_frame, layout, config, init_vecs);
        int index=0;
        auto vecs = init_vecs;
        auto raw_frame = raw_video[i];
        for (auto& vec : vecs) {
            double max_psnr = 0;
            std::vector<int> best_vec = init_vecs[index];
            for (const int dx : offsets){
                for (const int dy : offsets) {
                    vec[0] = init_vecs[index][0] + dx;
                    vec[1] = init_vecs[index][1] + dy;
                    recon_frame = estimation.exec(reloc_frame, layout, config, vecs);
                    double cur_psnr = utils::psnr(raw_frame[0], recon_frame[0]);
                    if (max_psnr < cur_psnr) {
                        max_psnr = cur_psnr;
                        best_vec = vec;
                    }
                }
            }
            vec = best_vec;
            index++;
        }
        all_vecs.push_back(vecs);

        if (opt == 0) continue;

        encoder::FittingModule fitting;
        recon_frame = estimation.exec(reloc_frame, layout, config, vecs);
        auto theta = fitting.exec(raw_video[i], reloc_frame, recon_frame, layout);
        a.push_back(theta.first);
        b.push_back(theta.second);
    }

    common::YUV420Writer yuv420_writer;
    yuv420_writer.exec(output_video, layout, config);

    encoder::MetaDataWriter metadata_writer;
    metadata_writer.exec(all_vecs, a, b, config, layout);

    std::cout << "done" << std::endl;
}


void decode_workflow(common::Dict config)
{
    common::LocalizationModule localization;
    const auto layout = localization.exec(config);

    decoder::YUV420Loader yuv420_loader;
    const auto mca_video = yuv420_loader.exec(layout, config);

    decoder::MetaDataLoader metadata_loader;
    auto res = metadata_loader.exec(config);
    auto all_vecs = res.all_vecs;
    const auto opt = std::any_cast<int>(config["optimize"]);
    std::vector<std::vector<double>> a, b;

    if (opt == 1)
    {
        a = res.a;
        b = res.b;
    }

    const auto frames = std::any_cast<int>(config["frames"]);
    std::vector<cv::Mat_C3> output_video;

    for (int i = 0; i < frames; i++)
    {
        common::RelocalizationModule reloc;
        const auto reloc_frame = reloc.exec(mca_video[i], layout, config);

        common::EstimationModule estimation;
        const auto recon_frame = estimation.exec(reloc_frame, layout, config, all_vecs[i]);

        if (opt == 1)
        {
            decoder::OptimizeModule optimize;
            const auto output_frame = optimize.exec(recon_frame, reloc_frame, a[i], b[i], layout);
            output_video.push_back(output_frame);
        }
        else output_video.push_back(recon_frame);
    }

    common::YUV420Writer yuv420_writer;
    yuv420_writer.exec(output_video, layout, config);

    std::cout << "done" << std::endl;
}


int main(const int argc, char *argv[]) {
    parser::ArgParser arg_parser(argc, argv);

    arg_parser.setHelp("--help");
    arg_parser.add("-config", "config file path");
    arg_parser.parse();

    common::ConfigLoader config_loader;
    const std::string config_path = arg_parser.get("-config");
    auto config = config_loader.exec(config_path);

    const auto mode = std::any_cast<int>(config["mode"]);

    if (mode == 0) encoder_workflow(config);
    else decode_workflow(config);

    return 0;
}

#include <iostream>
#include "mca.hpp"
#include "mca/proc/postproc.hpp"


int main(const int argc, char *argv[]) {
    mca::parser::ArgParser parser(argc, argv);

    parser.setHelp("--help");
    parser.add("-proc", "Pre or Post");
    parser.add("-i", "input yuv file path");
    parser.add("-o", "output yuv file path");
    parser.add("-config", "config file path");
    parser.add("-calib", "calibration file path");
    parser.add("-patch", "the size of patch in mi");
    parser.add("-log", "generate log info and key params");
    parser.add("-min_block_size", "min block size");
    parser.add("-max_block_size", "max block size");
    parser.add("-interval", "interval size");
    parser.add("-bin", "binary file path");

    parser.parse();

    if (parser.get("-proc") == "Pre")
    {
        std::string config_path = parser.get("-config");
        mca::parser::ConfigParser config(config_path);
        if (config.load() != -1)
            std::cout << "Loaded config file: " << config_path << std::endl;

        std::string calib_path = parser.get("-calib");
        mca::parser::Calibration::CalibParser calib(calib_path);
        if (calib.load() != -1)
            std::cout << "Loaded calibration file: " << calib_path << std::endl;

        mca::proc::preproc(parser, config, calib);
    }
    else if (parser.get("-proc") == "Post")
    {
        std::string log_path = parser.get("-log");
        mca::parser::ConfigParser config(log_path);

        if (config.load() != -1)
            std::cout << "Loaded config file: " << log_path << std::endl;
        mca::proc::postproc(parser, config);
    }

    return 0;
}

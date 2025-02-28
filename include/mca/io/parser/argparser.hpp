//
// Created by ph431 on 2025/2/25.
//

#ifndef ARGPARSER_HPP
#define ARGPARSER_HPP

#include <string>
#include <iostream>
#include <unordered_map>


namespace mca::parser {
    class ArgParser {
    private:
        int argc;
        char **argv;

        std::string helper;
        std::unordered_map<std::string, std::string> option;
        std::unordered_map<std::string, std::string> map;

    public:
        ArgParser(const int argc, char **argv)
        {
            this->argc = argc;
            this->argv = argv;
        }

        ~ArgParser() = default;

        void add(const std::string& opt, const std::string& tip)
        {
            option.insert(std::make_pair(opt, tip));
        }

        void setHelp(const std::string &opt)
        {
            helper = opt;
        }

        int parse()
        {
            int index = 1;

            while (index < argc)
            {
                std::string arg = argv[index];
                if (helper == arg)
                {
                    for (const auto& [key, value] : option)
                        std::cout << key << ":  " << value << std::endl;
                    return 0;
                }

                if (!option.contains(arg))
                {
                    std::cerr << "Error: Unknown argument: " << arg << std::endl;
                    return -1;
                }

                if (index + 1 >= argc)
                {
                    std::cerr << "Error: Missing value for option " << arg << std::endl;
                    return -1;
                }

                map[arg] = argv[++index];
                index++;
            }

            return 0;
        }

        std::string get(const std::string& key)
        {
            return map[key];
        }

        void print()
        {
            for (const auto& [key, value] : map)
                std::cout << key << ":  " << value << std::endl;
        }
    };
}

#endif //ARGPARSER_HPP

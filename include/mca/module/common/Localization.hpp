//
// Created by ph431 on 2025/7/31.
//

#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "../../common/pipeline/module.hpp"
#include "../../common/layout/MI.hpp"
#include "../../common/layout/TSPCLayout.hpp"
#include "../../common/layout/RaytrixLayout.hpp"

namespace mca::module::common {
    class LocalizationModule final : public Module<MI::layout_ptr, Dict> {
    public:
        MI::layout_ptr exec(Dict config) override
        {
            const int width = std::any_cast<int>(config["width"]);
            const int height = std::any_cast<int>(config["height"]);
            const int type = std::any_cast<int>(config["type"]);

            MI::layout_ptr layout;

            switch (type){
                case 0:
                    layout = std::make_shared<MI::RaytrixLayout>(width, height, config);
                    break;
                case 1:
                    layout = std::make_shared<MI::TSPCLayout>(width, height, config);
                default: ;
            }

            return layout;
        }
    };
}

#endif //LOCALIZATION_HPP

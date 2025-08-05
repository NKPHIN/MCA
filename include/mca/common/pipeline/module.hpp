//
// Created by ph431 on 2025/8/5.
//

#ifndef MODULE_HPP
#define MODULE_HPP

template <typename ReturnType, typename... Args>
class Module {
public:
    virtual ~Module() = default;
    virtual ReturnType exec(Args... args) = 0;
};

#endif //MODULE_HPP

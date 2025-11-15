cmake_minimum_required(VERSION 3.28)
project(MCA)

set(CMAKE_CXX_STANDARD 20)

include_directories(${PROJECT_SOURCE_DIR}/include)

# 解决 exe 无法直接运行的问题
set(CMAKE_EXE_LINKER_FLAGS "-static")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

add_executable(MCA src/main.cpp
        include/mca.hpp
        include/mca/io/parser/argparser.hpp
        include/mca/io/parser/parser.hpp
        include/mca/io/parser/config.hpp
        include/mca/io/parser/calib.hpp
        include/mca/common/cv/cv2.hpp
        include/mca/common/cv/Mat.hpp
        include/mca/common/cv/yuv420.hpp
        include/mca/common/cv/Point.hpp
        include/mca/common/layout/layout.hpp
        include/mca/common/layout/MI.hpp
        include/mca/common/cv/Rect.hpp
        include/mca/common/layout/TSPCLayout.hpp
        include/mca/common/layout/RaytrixLayout.hpp
        include/mca/utils/math.hpp
        include/mca/module/common/Localization.hpp
        include/mca/module/Encoder/MCAModule.hpp
        include/mca/module/common/Relocalization.hpp
        include/mca/module/common/Estimation.hpp
        include/mca/module/Encoder/Fitting.hpp
        include/mca/common/pipeline/module.hpp
        include/mca/module/pipeline.hpp
        include/mca/module/common/ConfigLoader.hpp
        include/mca/module/Encoder/YUV420Loader.hpp
        include/mca/module/common/YUV420Writer.hpp
        include/mca/module/Encoder/MetaDataWriter.hpp
        include/mca/module/decoder/YUV420Loader.hpp
        include/mca/module/decoder/MetaDataLoader.hpp
        include/mca/module/decoder/Optimize.hpp
)

cmake_minimum_required(VERSION 3.30)
project(MCA)

set(CMAKE_CXX_STANDARD 20)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(MCA src/main.cpp
        include/mca.hpp
        include/mca/io/parser/argparser.hpp
        include/mca/io/parser/parser.hpp
        include/mca/io/parser/config.hpp
        include/mca/io/parser/calib.hpp
        include/mca/common/cv/cv2.hpp
        include/mca/common/cv/Mat.hpp
        include/mca/common/cv/yuv420.hpp
        include/mca/proc/preproc.hpp
        include/mca/proc/postproc.hpp
        include/mca/common/cv/Point.hpp
        include/mca/common/layout/layout.hpp
        include/mca/common/layout/MI.hpp
        include/mca/common/cv/Rect.hpp
        include/mca/proc/tspc.hpp
        include/mca/common/layout/TSPCLayout.hpp
        include/mca/common/layout/RaytrixLayout.hpp
        include/mca/proc/raytrix.hpp
)

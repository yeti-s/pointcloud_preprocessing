cmake_minimum_required(VERSION 3.0)
project(lasManipC)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PDAL REQUIRED)
find_package(libLAS REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)


# nlohmann/json 라이브러리를 다운로드하고 포함합니다.
include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.10.2
)
FetchContent_MakeAvailable(json)


add_executable(lasManipC test.cpp)


target_link_libraries(lasManipC PRIVATE ${PDAL_LIBRARIES} ${libLAS_LIBRARIES} ${OpenCV_LIBRARIES})

target_include_directories(lasManipC PRIVATE ${PDAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${json_SOURCE_DIR}/include)
cmake_minimum_required(VERSION 3.0)
project(lasManipC)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PDAL REQUIRED)
find_package(libLAS REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)


include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.10.2
)
FetchContent_MakeAvailable(json)

include_directories(json/include)


add_executable(lasManipC test.cpp)


target_link_libraries(lasManipC PRIVATE ${PDAL_LIBRARIES} ${libLAS_LIBRARIES} ${OpenCV_LIBRARIES})

target_include_directories(lasManipC PRIVATE ${PDAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${json_SOURCE_DIR}/include)
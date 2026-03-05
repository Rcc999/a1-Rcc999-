
include(ExternalProject)
include(FetchContent)

# Eigen library
# Download as zip, so it's a bit smaller than downloading the whole repo
FetchContent_Declare(
    eigen
    URL                 https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    URL_HASH            MD5=1a914f7b208df0c46c01d0e265a6f341
)
FetchContent_MakeAvailable(eigen)
if(NOT TARGET eigen)
    add_library(eigen INTERFACE)
    target_include_directories(eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif()

# stb_image
add_subdirectory(stb_image)

# json
FetchContent_Declare(
    json
    URL                 https://github.com/nlohmann/json/releases/download/v3.9.1/include.zip
    URL_HASH            MD5=d2f66c608af689e21d69a33c220e974e
)
FetchContent_MakeAvailable(json)
add_library(nlohmann_json INTERFACE)
add_library(nlohmann::json ALIAS nlohmann_json)
target_include_directories(nlohmann_json SYSTEM INTERFACE $<BUILD_INTERFACE:${json_SOURCE_DIR}/include>)

# GUI stuff below
if(CMM_BUILD_GUI)
message(STATUS "building CMM GUI")
# glad
set(OpenGL_GL_PREFERENCE GLVND)
add_subdirectory(glad)

# glfw
set( GLFW_BUILD_EXAMPLES    OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_TESTS       OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_DOCS        OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_INSTALL     OFF CACHE BOOL  "GLFW lib only" )
FetchContent_Declare(
    glfw
    URL                 https://github.com/glfw/glfw/archive/3.3.2.zip
    URL_HASH            MD5=69d0aa90a9278c0dd8be975d1ae4aaf2
)
FetchContent_MakeAvailable(glfw)
set( GLFW_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../ext/glfw/include)

# glm
FetchContent_Declare(
    glm
    URL                 https://github.com/g-truc/glm/releases/download/0.9.9.7/glm-0.9.9.7.zip
    URL_HASH            MD5=da4740b5b41a1e2ccb2955af74e7c298
)
FetchContent_MakeAvailable(glm)
set( GLM_TEST_ENABLE OFF CACHE BOOL "" )
if(NOT TARGET glm AND TARGET glm::glm)
    add_library(glm ALIAS glm::glm)
endif()

# nanovg
FetchContent_Declare(
    nanovg
    GIT_REPOSITORY      https://github.com/memononen/nanovg
    GIT_TAG             077b65e0cf3e22ee4f588783e319b19b0a608065
)
FetchContent_MakeAvailable(nanovg)
file(GLOB sources ${nanovg_SOURCE_DIR}/src/*.h ${nanovg_SOURCE_DIR}/src/*.c)
add_library(nanovg ${sources})
target_include_directories(nanovg INTERFACE ${nanovg_SOURCE_DIR}/src/)
# set(NANOVG_SRC_DIR ${NANOVG_SRC_DIR} PARENT_SCOPE)

# imgui
FetchContent_Declare(
    imgui
    URL                 https://github.com/ocornut/imgui/archive/v1.81.zip
    URL_HASH            MD5=78af0cf170fb3f2a431eb39cac7f55b8
)
FetchContent_MakeAvailable(imgui)
file(GLOB sources ${imgui_SOURCE_DIR}/*.h ${imgui_SOURCE_DIR}/*.cpp)
add_library(imgui STATIC ${sources})
set_target_properties(imgui PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})
set( CMM_IMGUI_FONT_FOLDER "\"${imgui_SOURCE_DIR}/misc/fonts\"" CACHE STRING "")

# tinyobjloader
FetchContent_Declare(
    tinyobjloader #
    URL https://github.com/tinyobjloader/tinyobjloader/archive/v2.0-rc1.zip #
    URL_HASH MD5=f660a9f91fc2ea5c4f33d7530f6a69bc #
)
FetchContent_MakeAvailable(tinyobjloader)


endif(CMM_BUILD_GUI)

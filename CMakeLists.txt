cmake_minimum_required(VERSION 3.20)

# CMake 4.x dropped compatibility with < 3.5; keep FetchContent deps working.
# Also suppress deprecation warnings from deps that still declare < 3.10.
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

# Adopt newer policy behaviors when available to avoid deprecation warnings.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 NEW)
endif()

# set name of the project
project(a2)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

# data folder path
set(CRL_DATA_FOLDER
    "\"${CMAKE_CURRENT_LIST_DIR}/data\""
    CACHE STRING ""
)

# -----------------------------------------------------------------------------
# options
# -----------------------------------------------------------------------------

option(CMM_BUILD_GUI "build GUI" ON)

# -----------------------------------------------------------------------------
# macro and functions
# -----------------------------------------------------------------------------

# create static library named LIB_NAME
function(
    create_crl_library #
    LIB_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} STATIC ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    add_dependencies(${LIB_NAME} ${DEPENDENCY})
    target_include_directories(${LIB_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

# create executable named APP_NAME
function(
    create_crl_app #
    APP_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
)
    add_executable(${APP_NAME} ${SOURCE})
    add_dependencies(${APP_NAME} ${DEPENDENCY})
    target_include_directories(${APP_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${APP_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        target_compile_definitions(${APP_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${APP_NAME} PROPERTY FOLDER "Apps")
endfunction()

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# thirdparty code
add_subdirectory(ext)

# our code
add_subdirectory(src)

include(FetchContent)

FetchContent_Declare(
	get_cpm
	URL	https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/get_cpm.cmake
	DOWNLOAD_NO_EXTRACT ON
)

FetchContent_MakeAvailable(
	get_cpm
)

FetchContent_GetProperties(
	get_cpm
	SOURCE_DIR get_cpm_DIR
)

include(${get_cpm_DIR}/get_cpm.cmake)

set(TINYOBJLOADER_USE_DOUBLE ON)
CPMAddPackage(
    tinyObj
    GIT_REPOSITORY git@github.com:tinyobjloader/tinyobjloader.git
    GIT_TAG ee45fb41db95bf9563f2a41bc63adfa18475c2ee
)

find_package(Eigen3 3.4.0 EXACT REQUIRED)
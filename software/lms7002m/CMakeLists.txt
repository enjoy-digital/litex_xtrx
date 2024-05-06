########################################################################
# Project setup for LMS7 driver
#
# This project builds the sources into a static library
# and had install rules for the library and headers
########################################################################
cmake_minimum_required(VERSION 3.15)
project(LMS7002MDriver C CXX)

set(MAIN_LIBRARY_NAME lms7002mNG)

########################################################################
## public headers
########################################################################
file(
	GLOB LMS7002MNG_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/lms7002mNG/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/include/lms7002mNG/*.hpp")
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_HEADERS "Enable install headers" ON "TRUE" OFF)
add_feature_info(HEADERS ENABLE_HEADERS "The lms7002mNG headers")
if(ENABLE_HEADERS)
	install(FILES ${LMS7002MNG_PUBLIC_HEADERS} DESTINATION include/lms7002mNG)
endif()

#file(GLOB LMS7002M_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_*.cpp")
set(LMS7002M_SOURCES
	#src/lms7002m/MCU_BD.cpp
    #src/lms7002m/MCU_File.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_RegistersMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_parameters.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M.cpp
	#${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_validation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_RxTxCalibrations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_BaseCalibrations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mcu_dc_iq_calibration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_filtersCalibration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LMS7002M_gainCalibrations.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/MCU_BD.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/MCU_File.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/logger/Logger.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/logger/LoggerCString.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/toString.cpp
)

set(LMS7002M_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger
)

########################################################################
# Compiler setup
########################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#if(CMAKE_COMPILER_IS_GNUCC)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
#    add_definitions(-D_GNU_SOURCE)
#    add_definitions(-Wall)
#    add_definitions(-Wextra)
#    add_definitions(-pedantic)
#endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    #enable warnings in Debug configurations
    add_compile_options(-Wall -Wpedantic)

    #symbols are only exported from libraries/modules explicitly
    add_compile_options(-fvisibility=hidden)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)

    # enable C-style cast warnings in C++
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>)

    #disable gcc caller-saves flag for O2-O3 optimizations
    #workaround fix for gcc 9.3+
    if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.3 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 9.9)
            add_compile_options(-fno-caller-saves)
        endif()
    endif()

    #default SIMD configuration uses native build flags
    #when packaging and x86, use sse3 so the binaries work across multiple x86 variants
    if(NOT DEFAULT_SIMD_FLAGS)
        set(DEFAULT_SIMD_FLAGS "native")
    endif()
    if("${CMAKE_INSTALL_PREFIX}" STREQUAL "/usr" AND X86)
        set(DEFAULT_SIMD_FLAGS "SSE3")
    endif()
    if(RV64)
        set(DEFAULT_SIMD_FLAGS "rv64")
    endif()

    set(ENABLE_SIMD_FLAGS
        "${DEFAULT_SIMD_FLAGS}"
        CACHE STRING "Set compiler SIMD flags")
    set_property(CACHE ENABLE_SIMD_FLAGS PROPERTY STRINGS none native SSE3 SSE4.2 AVX AVX2)

    if(${ENABLE_SIMD_FLAGS} MATCHES "native")
        add_compile_options(-march=native)
        if(X86)
            add_compile_options(-mfpmath=sse)
        endif()
        message(STATUS "Enabling native optimizations")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX2")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfma -mavx2 -mfpmath=sse)
        message(STATUS "Enabling AVX2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfpmath=sse)
        message(STATUS "Enabling AVX instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE4.2")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mfpmath=sse)
        message(STATUS "Enabling SSE4.2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE3")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mfpmath=sse)
        message(STATUS "Enabling SSE3 instructions")
    endif()

endif()


########################################################################
# Build library
# Defaults to static, set BUILD_SHARED_LIBS=ON for shared
########################################################################
include_directories(include)
#add_library(LMS7002M ${LMS7002M_SOURCES})
#install(TARGETS LMS7002M DESTINATION lib${LIB_SUFFIX})
#set_property(TARGET LMS7002M PROPERTY POSITION_INDEPENDENT_CODE TRUE)

########################################################################
# install headers
########################################################################
#install(DIRECTORY include/LMS7002M DESTINATION include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/utilities)

option(ENABLE_LIBRARY "Enable build library" ON)
#add_feature_info(LIBRARY ENABLE_LIBRARY "The lms7002mNG library")
if(ENABLE_LIBRARY)
    add_library(${MAIN_LIBRARY_NAME} ${LMS7002M_SOURCES})
    target_link_libraries(
        ${MAIN_LIBRARY_NAME}
        PUBLIC ${LIME_SUITE_LIBRARIES}
		#PRIVATE $<BUILD_INTERFACE:kissFFT> $<BUILD_INTERFACE:samplesConversion>
	)
    set_property(TARGET ${MAIN_LIBRARY_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	#set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES SOVERSION ${LIME_SUITE_SOVER})
	#set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES VERSION ${LIME_SUITE_LIBVER})
	set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES DEFINE_SYMBOL "LIME_DLL_EXPORTS")
	target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${LMS7002M_INCLUDES})
    target_include_directories(
        ${MAIN_LIBRARY_NAME}
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external>
                  $<INSTALL_INTERFACE:include>)

    if(BUILD_SHARED_LIBS)
        target_compile_definitions(${MAIN_LIBRARY_NAME} PUBLIC "LIME_DLL")
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug"
       AND ENABLE_CODE_COVERAGE
       AND (NOT MSVC))
        include(CodeCoverage)
        append_coverage_compiler_flags_to_target(${MAIN_LIBRARY_NAME})

        if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_link_options(${MAIN_LIBRARY_NAME} PUBLIC --coverage)
        endif()
    endif()

    install(
        TARGETS ${MAIN_LIBRARY_NAME}
        EXPORT lms7002mNG
        LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
        ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
        RUNTIME DESTINATION bin # .dll file
    )

	#add_subdirectory(GFIR)
	#set(LIME_SUITE_LIBS LimeSuite)
endif()


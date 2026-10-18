cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(FOC LANGUAGES C)
endif()


set(FOC_SOURCES
    Motor_port_template_G431.c
    Src/FOC.c
    Src/RsSamp.c
    Src/Sensor.c
    Src/SinCosTab.c
    Src/Svpwm.c
    Sensor/ThreePhaseHall.c
)

add_library(FOC STATIC ${FOC_SOURCES})
add_library(FOC::FOC ALIAS FOC)

target_include_directories(FOC
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/Inc"
        "${CMAKE_CURRENT_LIST_DIR}/Sensor"
)

target_compile_features(FOC PUBLIC c_std_11)
set_target_properties(FOC PROPERTIES
    C_EXTENSIONS ON
    C_STANDARD_REQUIRED ON
)

if(TARGET stm32cubemx)
    target_link_libraries(FOC PUBLIC stm32cubemx)
endif()

if(FOC_PLATFORM_LIBRARIES)
    target_link_libraries(FOC PUBLIC ${FOC_PLATFORM_LIBRARIES})
endif()

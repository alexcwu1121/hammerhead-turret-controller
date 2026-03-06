cmake_minimum_required(VERSION 3.22)

# Build the HAL
add_subdirectory(hal)

# Build QPCPP
set(QPCPP_CFG_KERNEL QK)
set(QPCPP_CFG_PORT arm-cm)
add_subdirectory(qpcpp)
target_link_libraries(qpcpp
    PUBLIC
        hal
)

# Build htc application
add_executable(${CMAKE_PROJECT_NAME})
target_include_directories(${CMAKE_PROJECT_NAME}
    PRIVATE
        ./
)
target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        startup_stm32f302xc.s
        main.cpp
)
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
        qpcpp
)
target_compile_definitions(${CMAKE_PROJECT_NAME}
    PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
)
target_link_options(${CMAKE_PROJECT_NAME}
    PRIVATE
        -T${CMAKE_CURRENT_LIST_DIR}/STM32F302XX_FLASH.ld
)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

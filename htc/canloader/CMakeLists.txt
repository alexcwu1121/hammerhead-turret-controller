cmake_minimum_required(VERSION 3.22)

# Build can loader
set(CAN_LOADER_PROJECT_NAME ${CMAKE_PROJECT_NAME}-canloader)

add_executable(${CAN_LOADER_PROJECT_NAME})
target_include_directories(${CAN_LOADER_PROJECT_NAME}
    PRIVATE
        ./
)
target_sources(${CAN_LOADER_PROJECT_NAME}
    PRIVATE
        startup_stm32f302xc_canloader.s
        main.cpp
)
target_link_libraries(${CAN_LOADER_PROJECT_NAME}
    PUBLIC
        hal
)
target_compile_definitions(${CAN_LOADER_PROJECT_NAME}
    PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
)
target_link_options(${CAN_LOADER_PROJECT_NAME}
    PRIVATE
        -T${CMAKE_CURRENT_LIST_DIR}/STM32F302XX_FLASH_CANLOADER.ld
)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CAN_LOADER_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CAN_LOADER_PROJECT_NAME}.map)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

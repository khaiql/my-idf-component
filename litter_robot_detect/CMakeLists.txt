set(requires esp32-camera)
set(impl_srcs "litter_robot_detect_common.cpp")
# set(image_file "test_image.jpg") # Use relative name for internal logic

if(CONFIG_LITTER_ROBOT_MODEL_TFLITE)
    message(STATUS "Litter Robot Detect: Using TF LITE model")
    list(APPEND impl_srcs "litter_robot_detect_tflite.cpp")
endif()

if(CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ)
    message(STATUS "Litter Robot Detect: Using ESP-DL PPQ model")
    
    # ... (Your logic to find espdl_dir remains the same) ...
    set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
    include(${cmake_dir}/utilities.cmake)

    list(APPEND model_files "lite_model_esp32s3.espdl")
    list(APPEND impl_srcs "litter_robot_detect_ppq.cpp")
    list(APPEND requires esp-dl)
endif()

# Register the component
idf_component_register(SRCS ${impl_srcs}
                INCLUDE_DIRS "include"
                REQUIRES ${requires}
                # We leave EMBED_FILES empty here to bypass the PIO scanner bug
                )

# --- Binary Embedding ---

# 1. Embed the Image
# If using ESP-DL/PPQ, we use the aligned version. 
# Otherwise, we use the standard absolute path trick which also fixes the PIO bug.
if(COMMAND target_add_aligned_binary_data)
    # target_add_aligned_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/${image_file}" BINARY)
    
    # 2. Also embed the Model if in PPQ mode
    if(model_files)
        target_add_aligned_binary_data(${COMPONENT_LIB} "${model_files}" BINARY)
    endif()
else()
    # Standard ESP-IDF way with an absolute path (bypasses the PIO scanner bug)
    # target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/${image_file}" BINARY)
endif()
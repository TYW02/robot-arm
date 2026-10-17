idf_component_register(
    SRCS "aruco_detect.cpp" "vector.cc"
    INCLUDE_DIRS "include"
    REQUIRES esp32-camera esp_driver_uart driver
)

# Tell the compiler to allow legacy C++ template code and suppress the warnings
target_compile_options(${COMPONENT_LIB} PRIVATE -fpermissive -Wno-error)
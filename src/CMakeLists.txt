if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    add_subdirectory(cuda_points_downsampler)
endif()
add_subdirectory(ga_points_downsampler)
add_subdirectory(pcl_points_downsampler)

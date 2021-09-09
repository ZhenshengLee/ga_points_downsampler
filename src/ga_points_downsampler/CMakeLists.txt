set(EXE_NAME "ga_points_downsampler")
# node
cuda_add_executable(${EXE_NAME} ./${EXE_NAME}_node.cpp ./${EXE_NAME}.cu)
target_compile_definitions(${EXE_NAME} PRIVATE
        ${CUPOCH_DEFINITIONS}
)
target_include_directories(${EXE_NAME} PRIVATE
        ${CUDA_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CUPOCH_INCLUDE_DIRS}
        )
target_link_libraries(${EXE_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CUPOCH_LIBRARIES}
        ${Open3D_LIBRARIES}
        ${PCL_LIBRARIES}
        cupoch_conversions
        # open3d_conversions
)
add_dependencies(${EXE_NAME}
        ${catkin_EXPORTED_TARGETS}
)
install(TARGETS ${EXE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
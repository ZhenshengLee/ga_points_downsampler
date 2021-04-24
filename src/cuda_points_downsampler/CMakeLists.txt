set(EXE_NAME "cuda_points_downsampler")
# node
cuda_add_executable(${EXE_NAME} ./${EXE_NAME}_node.cpp ./${EXE_NAME}.cu)
target_compile_definitions(${EXE_NAME} PRIVATE
)
target_include_directories(${EXE_NAME} PRIVATE
        ${CUDA_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
target_link_libraries(${EXE_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)
add_dependencies(${EXE_NAME}
        ${catkin_EXPORTED_TARGETS}
)
install(TARGETS ${EXE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
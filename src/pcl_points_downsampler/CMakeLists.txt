set(EXE_NAME "pcl_points_downsampler")
# node
add_executable(${EXE_NAME} ./${EXE_NAME}_node.cpp ./${EXE_NAME}.cpp)
target_compile_definitions(${EXE_NAME} PRIVATE
)
target_include_directories(${EXE_NAME} PRIVATE
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
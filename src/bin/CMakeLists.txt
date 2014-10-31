add_executable(o3d3xx-version o3d3xx-version.cpp)
target_link_libraries(o3d3xx-version o3d3xx)

add_executable(o3d3xx-reset o3d3xx-reset.cpp)
target_link_libraries(o3d3xx-reset o3d3xx)

add_executable(o3d3xx-viewer o3d3xx-viewer.cpp)
target_link_libraries(o3d3xx-viewer
  o3d3xx ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
         o3d3xx-version
	 o3d3xx-viewer
	 o3d3xx-reset
	DESTINATION bin
	)

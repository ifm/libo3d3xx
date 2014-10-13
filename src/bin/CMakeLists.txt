add_executable(o3d3xx-version o3d3xx-version.cpp)
target_link_libraries(o3d3xx-version o3d3xx)

add_executable(o3d3xx-cloud-viewer o3d3xx-cloud-viewer.cpp)
target_link_libraries(o3d3xx-cloud-viewer
  o3d3xx ${PCL_VISUALIZATION_LIBRARIES})

add_executable(o3d3xx-depth-viewer o3d3xx-depth-viewer.cpp)
target_link_libraries(o3d3xx-depth-viewer o3d3xx ${OpenCV_LIBS})

add_executable(o3d3xx-amplitude-viewer o3d3xx-amplitude-viewer.cpp)
target_link_libraries(o3d3xx-amplitude-viewer o3d3xx ${OpenCV_LIBS})

install(TARGETS
         o3d3xx-version
	 o3d3xx-cloud-viewer
	 o3d3xx-depth-viewer
	 o3d3xx-amplitude-viewer
	DESTINATION bin
	)

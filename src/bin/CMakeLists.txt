add_executable(o3d3xx-version o3d3xx-version.cpp)
target_link_libraries(o3d3xx-version o3d3xx)

add_executable(o3d3xx-cloud-viewer o3d3xx-cloud-viewer.cpp)
target_link_libraries(o3d3xx-cloud-viewer
  o3d3xx ${PCL_VISUALIZATION_LIBRARIES})

install(TARGETS
         o3d3xx-version
	 o3d3xx-cloud-viewer
	DESTINATION bin
	)

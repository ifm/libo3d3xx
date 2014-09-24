add_executable(o3d3xx-version o3d3xx-version.cpp)
target_link_libraries(o3d3xx-version o3d3xx)

install(TARGETS
         o3d3xx-version
	DESTINATION bin
	)


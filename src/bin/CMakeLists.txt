add_executable(o3d3xx-version o3d3xx-version.cpp)
target_link_libraries(o3d3xx-version o3d3xx)

add_executable(o3d3xx-reset o3d3xx-reset.cpp)
target_link_libraries(o3d3xx-reset o3d3xx)

add_executable(o3d3xx-ls o3d3xx-ls.cpp)
target_link_libraries(o3d3xx-ls o3d3xx)

add_executable(o3d3xx-viewer o3d3xx-viewer.cpp)
target_link_libraries(o3d3xx-viewer
  o3d3xx
  ${PCL_VISUALIZATION_LIBRARIES}
  ${OpenCV_LIBS}
  )
set_target_properties(o3d3xx-viewer PROPERTIES AUTOMOC TRUE)

add_executable(o3d3xx-dump o3d3xx-dump.cpp)
target_link_libraries(o3d3xx-dump o3d3xx)

add_executable(o3d3xx-config o3d3xx-config.cpp)
target_link_libraries(o3d3xx-config o3d3xx)

add_executable(o3d3xx-rm o3d3xx-rm.cpp)
target_link_libraries(o3d3xx-rm o3d3xx)

add_executable(o3d3xx-reboot o3d3xx-reboot.cpp)
target_link_libraries(o3d3xx-reboot o3d3xx)

add_executable(o3d3xx-hz o3d3xx-hz.cpp)
target_link_libraries(o3d3xx-hz o3d3xx)

install(TARGETS
         o3d3xx-version
	 o3d3xx-viewer
	 o3d3xx-reset
	 o3d3xx-ls
	 o3d3xx-dump
	 o3d3xx-config
	 o3d3xx-rm
	 o3d3xx-reboot
	 o3d3xx-hz
	DESTINATION bin
	)

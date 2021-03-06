set(SRCS
            emu_window/emu_window_glfw.cpp
            citra.cpp
            config.cpp
            citra.rc
            )
set(HEADERS
            emu_window/emu_window_glfw.h
            config.h
            default_ini.h
            resource.h
            )

create_directory_groups(${SRCS} ${HEADERS})

include_directories(${GLFW_INCLUDE_DIRS})
include_directories(${OPENAL_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})
link_directories(${OPENAL_LIBRARY_DIRS})

add_executable(citra ${SRCS} ${HEADERS})
target_link_libraries(citra core video_core common)
target_link_libraries(citra ${GLFW_LIBRARIES} ${OPENGL_gl_LIBRARY} inih glad ${OPENAL_LIBRARIES})
if (MSVC)
    target_link_libraries(citra getopt)
endif()
target_link_libraries(citra ${PLATFORM_LIBRARIES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|FreeBSD|OpenBSD|NetBSD")
    install(TARGETS citra RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
endif()

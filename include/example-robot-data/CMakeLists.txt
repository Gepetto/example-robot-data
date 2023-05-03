configure_file(path.hpp.in path.hpp)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/path.hpp
        DESTINATION include/${CUSTOM_HEADER_DIR})

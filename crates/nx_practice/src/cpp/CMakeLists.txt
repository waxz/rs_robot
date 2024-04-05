

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)


add_library(math_op_static STATIC math_op.cpp )
add_library(math_op_shared SHARED math_op.cpp )

install(TARGETS math_op_static
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
)
install(TARGETS math_op_shared
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
)

#build_external_project_command(tinycc lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinycc-mob_2023_1220.zip "./configure --prefix=${CMAKE_BINARY_DIR}/force_tinycc/install --enable-static --debug --extra-cflags='-std=c11 -Wpedantic -fsanitize=address,undefined -g' " "make -j6" " make install" )
#set(tinycc_COMPILER_DIR ${tinycc_ROOT}/lib/tcc)
#set(tinycc_LIBTCC_INCLUDE_DIR ${tinycc_ROOT}/include)
#set(tinycc_TCCLIB_INCLUDE_DIR ${tinycc_ROOT}/lib/tcc/include)
#set(tinycc_LIB ${tinycc_ROOT}/lib/libtcc.a)

#install(TARGETS ${tinycc_LIB}
#		LIBRARY DESTINATION lib
#		RUNTIME DESTINATION bin
#)


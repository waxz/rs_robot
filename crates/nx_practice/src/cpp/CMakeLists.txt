

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
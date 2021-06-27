

FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")

SET(impl_hdrs  impl/linear.h    impl/kdtree.h       impl/kmeansindex.h  impl/kmeansindexcreator.h  impl/distances.h impl/heap.h impl/resultset.h  )
SET(impl_srcs  impl/linear.cpp impl/kdtree.cpp     impl/kmeansindex.cpp  impl/kmeansindexcreator.cpp  )

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )



ADD_LIBRARY(xflann ${srcs} ${hdrs} ${impl_hdrs} ${impl_srcs})

SET_TARGET_PROPERTIES(xflann PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL XFLANN_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "xflann${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)


INSTALL(TARGETS xflann
	RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
	LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory



INSTALL(FILES ${hdrs_base}
        DESTINATION include/xflann
        COMPONENT main)




INCLUDE_DIRECTORIES(.)
FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")

SET( opt_src   optimization/globaloptimizer.cpp     optimization/ippe.cpp     optimization/loopclosure.cpp      optimization/solvepnp.cpp)
SET( opt_hdr   optimization/globaloptimizer.h optimization/levmarq.h         optimization/ippe.h   optimization/sparselevmarq.h  optimization/proj_info.h optimization/loopclosure.h   optimization/solvepnp.h)

SET( opt_src ${opt_src} optimization/globaloptimizer_g2o.cpp optimization/solvepnp_g2o.cpp optimization/solvepnp_ucoslam.cpp)
SET( opt_hdr ${opt_hdr} optimization/globaloptimizer_g2o.h)

SET(basictypes_hdr basic_types/frame.h  basic_types/mappoint.h  basic_types/marker.h)
SET(basictypes_src basic_types/frame.cpp  basic_types/mappoint.cpp  basic_types/marker.cpp)
#IF(WITH_CVBA)
#    SET( opt_src ${opt_src} optimization/globaloptimizer_cvba_pba.cpp optimization/globaloptimizer_cvba_ucosba.cpp)
#    SET( opt_hdr ${opt_hdr} optimization/globaloptimizer_cvba_pba.h  optimization/globaloptimizer_cvba_ucosba.h)


#ENDIF()


#  VIEWERS
SET(viewers_src mapviewer.cpp )
SET(viewers_hdr mapviewer.h  viewers/slam_sglcvviewer.h  viewers/sgl.h)
#IF(USEQT)
#    SET(viewers_src ${viewers_src} viewers/qtgl_viewer.h )
#    SET(viewers_hdr ${viewers_hdr} viewers/qtgl_viewer.cpp )
#ENDIF()
#IF( Pangolin_FOUND)
#    SET(viewers_src ${viewers_src} viewers/pangolin_viewer.h )
#    SET(viewers_hdr ${viewers_hdr} viewers/pangolin_viewer.cpp )
#ENDIF()

 
SET(stuff_src stuff/debug.cpp  stuff/ferns.cpp  stuff/covisgraph.cpp  stuff/io_utils.cpp  stuff/utils.cpp stuff/matcher.cpp
      stuff/frameextractor.cpp stuff/framedatabase.cpp stuff/mapinitializer.cpp stuff/loopdetector.cpp)
SET(stuff_hdr stuff/debug.h                stuff/ferns.h   stuff/heap.h      stuff/picoflann.h       stuff/se3.h     stuff/utils.h
    stuff/expansiblecontainer.h  stuff/covisgraph.h  stuff/io_utils.h  stuff/reusablecontainer.h  stuff/timers.h stuff/matcher.h stuff/se3transform.h
     stuff/tsqueue.h stuff/frameextractor.h stuff/framedatabase.h stuff/mapinitializer.h stuff/minmaxbags.h stuff/hash.h   stuff/loopdetector.h)

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )



ADD_LIBRARY(${PROJECT_NAME} ${srcs} ${hdrs} ${opt_src} ${opt_hdr}
		${fextractors_src} ${fextractors_hdr}
		${viewers_src} ${viewers_hdr} ${stuff_src}
		${stuff_hdr} ${basictypes_hdr}
		${basictypes_src}
		)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL UCOSLAM_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
	OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES}
		apriltag::apriltag)

 INSTALL(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
	LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory



INSTALL(FILES ${hdrs_base}
        DESTINATION include/${PROJECT_NAME}
        COMPONENT main)

INSTALL(FILES ${utils_hdr}
        DESTINATION include/${PROJECT_NAME}/utils
        COMPONENT main)
INSTALL(FILES ${optimizers_hdr}
        DESTINATION include/${PROJECT_NAME}/optimizers
        COMPONENT main)


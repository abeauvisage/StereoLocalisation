if (CDFF_INCLUDE_DIRS AND CDFF_LIBRARIES)
  # Set <packagename>_FOUND if in cache already
  set(CDFF_FOUND TRUE)

else (CDFF_INCLUDE_DIRS AND CDFF_LIBRARIES)
  # Look for the path to EdresWrapper headers
  find_path(CDFF_INCLUDE_COMMON Core PATHS ${CMAKE_CURRENT_LIST_DIR}/../../include/CDFF/Common)
	find_path(CDFF_INCLUDE_DFNS BundleAdjustment PATHS ${CMAKE_CURRENT_LIST_DIR}/../../include/CDFF/DFNs)
	find_path(CDFF_INCLUDE_DFPCS VisualSlamStereo PATHS ${CMAKE_CURRENT_LIST_DIR}/../../include/CDFF/DFPCs)
	find_path(CDFF_LIB_DIR NAMES ibcdff_converters.a libcdff_helpers.a libcdff_logger.a libcdff_types.a libcdff_validators.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	set(CDFF_INCLUDE_DIRS ${CDFF_INCLUDE_COMMON} ${CDFF_INCLUDE_DFNS} ${CDFF_INCLUDE_DFPCS})

  # Look for the path to EdresWrapper library
  find_library(CDFF_CVT libcdff_converters.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	find_library(CDFF_HELP libcdff_helpers.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	find_library(CDFF_LOG libcdff_logger.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	find_library(CDFF_TYPES libcdff_types.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	find_library(CDFF_VAL libcdff_validators.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	#find_library(CDFF_VIZCV libcdff_visualizers_opencv.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)
	#find_library(CDFF_VIZPCL libcdff_visualizers_pcl.a PATHS ${CMAKE_CURRENT_LIST_DIR}/../../lib)

	set(CDFF_LIBRARIES ${CDFF_CVT} ${CDFF_HELP} ${CDFF_LOG} ${CDFF_TYPES} ${CDFF_VAL})# ${CDFF_VIZCV} ${CDFF_VIZPCL})


  # Set <packagename>_FOUND if CDFF_INCLUDE_DIRS and 
  # CDFF_LIBRARIES contains valid filepaths
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(CDFF DEFAULT_MSG CDFF_INCLUDE_DIRS CDFF_LIBRARIES)

link_directories(${CDFF_LIB_DIR})
message("adding ${CMAKE_CURRENT_LIST_DIR}/../../lib as link_directory")

endif(CDFF_INCLUDE_DIRS AND CDFF_LIBRARIES)

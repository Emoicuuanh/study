# Findglog.cmake
# Find Google glog library and headers.
#
# Sets:
#   glog_FOUND
#   glog_INCLUDE_DIRS
#   glog_LIBRARIES
#   glog::glog (Imported target)

find_path(glog_INCLUDE_DIR NAMES glog/logging.h)
find_library(glog_LIBRARY NAMES glog)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog
  DEFAULT_MSG
  glog_LIBRARY
  glog_INCLUDE_DIR
)

if(glog_FOUND)
  find_package(gflags REQUIRED)
  set(glog_INCLUDE_DIRS ${glog_INCLUDE_DIR})
  set(glog_LIBRARIES ${glog_LIBRARY} gflags)
  if(NOT TARGET glog::glog)
    add_library(glog::glog INTERFACE IMPORTED)
    set_target_properties(glog::glog PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${glog_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${glog_LIBRARIES}"
    )
  endif()
endif()

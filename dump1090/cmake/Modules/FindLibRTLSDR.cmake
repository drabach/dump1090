if(NOT LIBRTLSDR_FOUND)
  pkg_check_modules (LIBRTLSDR_PKG librtlsdr)
  find_path(LIBRTLSDR_INCLUDE_DIR NAMES rtl-sdr.h
    PATHS
    ${LIBRTLSDR_PKG_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
  )

  find_library(LIBRTLSDR_LIBRARIES NAMES rtlsdr
    PATHS
    ${LIBRTLSDR_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
  )

if(LIBRTLSDR_INCLUDE_DIR AND LIBRTLSDR_LIBRARIES)
  set(LIBRTLSDR_FOUND TRUE CACHE INTERNAL "librtlsdr-1.0 found")
  message(STATUS "Found librtlsdr-1.0: ${LIBRTLSDR_INCLUDE_DIR}, ${LIBRTLSDR_LIBRARIES}")
else(LIBRTLSDR_INCLUDE_DIR AND LIBRTLSDR_LIBRARIES)
  set(LIBRTLSDR_FOUND FALSE CACHE INTERNAL "librtlsdr-1.0 found")
  message(STATUS "librtlsdr-1.0 not found.")
endif(LIBRTLSDR_INCLUDE_DIR AND LIBRTLSDR_LIBRARIES)

mark_as_advanced(LIBRTLSDR_INCLUDE_DIR LIBRTLSDR_LIBRARIES)

endif(NOT LIBRTLSDR_FOUND)

#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CppLinuxSerial::CppLinuxSerial" for configuration ""
set_property(TARGET CppLinuxSerial::CppLinuxSerial APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(CppLinuxSerial::CppLinuxSerial PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libCppLinuxSerial.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS CppLinuxSerial::CppLinuxSerial )
list(APPEND _IMPORT_CHECK_FILES_FOR_CppLinuxSerial::CppLinuxSerial "${_IMPORT_PREFIX}/lib/libCppLinuxSerial.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

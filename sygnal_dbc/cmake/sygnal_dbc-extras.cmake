# sygnal_dbc extras included by ament's find_package

# Determine the installed share directory from the location of this config
get_filename_component(_pkg_share_dir "${sygnal_dbc_DIR}/.." ABSOLUTE)

# Public variable for consumers to locate the DBC database
set(SYGNAL_DBC_DIR "${_pkg_share_dir}/database")

if(NOT sygnal_dbc_FIND_QUIETLY)
  message(STATUS "sygnal_dbc: SYGNAL_DBC_DIR='${SYGNAL_DBC_DIR}'")
endif()


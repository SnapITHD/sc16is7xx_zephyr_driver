# SPDX-License-Identifier: Apache-2.0

# Point to this project's root directory.
set(SC16IS75_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "SC16IS75XX module root directory")


if(CONFIG_SC16IS7XX)
  zephyr_library()
  
  # Add the source file
  zephyr_library_sources(drivers/serial/sc16is7xx.c)
  
  # Make the header directory available globally
  zephyr_include_directories(drivers/serial)
  
  # Or if you want a cleaner include path:
  # zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
endif()

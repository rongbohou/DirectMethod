# - Try to find GLFW3
#
# If no pkgconfig, define GLFW_ROOT to installation tree
# Will define the following:
# GLFW3_FOUND
# GLFW3_INCLUDE_DIRS
# GLFW3_LIBRARIES

  FIND_LIBRARY(GLFW3_LIBRARY
    NAMES glfw
    PATHS
   /usr/lib/i386-linux-gnu
 )
  SET(GLFW3_LIBRARIES ${GLFW3_LIBRARY})


FIND_PATH(GLFW3_INCLUDE_DIRS
  GLFW/glfw3.h
  PATHS
  /usr/include
)


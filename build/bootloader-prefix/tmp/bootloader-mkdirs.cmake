# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/millerrabin/projects/millerrabin/esp-idf/components/bootloader/subproject"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/tmp"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/src/bootloader-stamp"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/src"
  "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/millerrabin/projects/millerrabin/thinker-robot-host/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

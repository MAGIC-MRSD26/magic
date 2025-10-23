# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(kinova_binary_api-populate NONE)



include(ExternalProject)
ExternalProject_Add(kinova_binary_api-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip" "URL_HASH" "MD5=64bd86e7ab8bda90ef1fc7d6a356e080"
                    SOURCE_DIR          "/home/emma/Documents/GitHub/magic/build/kortex_driver/_deps/kinova_binary_api-src"
                    BINARY_DIR          "/home/emma/Documents/GitHub/magic/build/kortex_driver/_deps/kinova_binary_api-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)



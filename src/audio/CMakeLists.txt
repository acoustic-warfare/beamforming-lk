# Portaudio
find_library(portaudio
        portaudio
        REQUIRED
)

# MP3 Lame
find_library(LAME_LIB mp3lame REQUIRED)

# SndFiles Wav
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LIBSNDFILE_PKGCONF sndfile)
endif(PKG_CONFIG_FOUND)

## Include dir
find_path(LIBSNDFILE_INCLUDE_DIR
        NAMES sndfile.h
        PATHS ${LIBSNDFILE_PKGCONF_INCLUDE_DIRS}
        )

## Library
find_library(LIBSNDFILE_LIBRARY
        NAMES sndfile libsndfile-1
        PATHS ${LIBSNDFILE_PKGCONF_LIBRARY_DIRS}
        )

find_package(PackageHandleStandardArgs)
find_package_handle_standard_args(LibSndFile  DEFAULT_MSG  LIBSNDFILE_LIBRARY LIBSNDFILE_INCLUDE_DIR)

if(LIBSNDFILE_FOUND)
    set(LIBSNDFILE_LIBRARIES ${LIBSNDFILE_LIBRARY})
    set(LIBSNDFILE_INCLUDE_DIRS ${LIBSNDFILE_INCLUDE_DIR})
endif(LIBSNDFILE_FOUND)

mark_as_advanced(LIBSNDFILE_LIBRARY LIBSNDFILE_LIBRARIES LIBSNDFILE_INCLUDE_DIR LIBSNDFILE_INCLUDE_DIRS)

include(FindPkgConfig)
pkg_search_module(SndFile REQUIRED sndfile)

# Include directories for SndFile
include_directories(${SndFile_INCLUDE_DIRS})

set(AUDIO_SOURCES 
    audio_wrapper.h
    audio_wrapper.cpp
)

set(SOURCES ${SOURCES} ${AUDIO_SOURCES})

add_library(audio STATIC ${AUDIO_SOURCES})
target_include_directories(audio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(audio PUBLIC
    fpga
    geometry
    mp3lame
    portaudio
    ${LIBSNDFILE_LIBRARIES}
)


#pragma once
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HUSKY_BASE_EXPORT __attribute__ ((dllexport))
    #define HUSKY_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define HUSKY_BASE_EXPORT __declspec(dllexport)
    #define HUSKY_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef HUSKY_BASE_BUILDING_LIBRARY
    #define HUSKY_BASE_PUBLIC HUSKY_BASE_EXPORT
  #else
    #define HUSKY_BASE_PUBLIC HUSKY_BASE_IMPORT
  #endif
#else
  #define HUSKY_BASE_EXPORT __attribute__ ((visibility("default")))
  #define HUSKY_BASE_IMPORT
  #if __GNUC__ >= 4
    #define HUSKY_BASE_PUBLIC __attribute__ ((visibility("default")))
  #else
    #define HUSKY_BASE_PUBLIC
  #endif
#endif

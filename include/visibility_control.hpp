#ifndef TINYMOVR_ROS2__VISIBILITY_CONTROL_HPP_
#define TINYMOVR_ROS2__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TINYMOVR_ROS2_EXPORT __attribute__ ((dllexport))
    #define TINYMOVR_ROS2_IMPORT __attribute__ ((dllimport))
  #else
    #define TINYMOVR_ROS2_EXPORT __declspec(dllexport)
    #define TINYMOVR_ROS2_IMPORT __declspec(dllimport)
  #endif
  #ifdef TINYMOVR_ROS2_BUILDING_LIBRARY
    #define TINYMOVR_ROS2_PUBLIC TINYMOVR_ROS2_EXPORT
  #else
    #define TINYMOVR_ROS2_PUBLIC TINYMOVR_ROS2_IMPORT
  #endif
  #define TINYMOVR_ROS2_PUBLIC_TYPE TINYMOVR_ROS2_PUBLIC
  #define TINYMOVR_ROS2_LOCAL
#else
  #define TINYMOVR_ROS2_EXPORT __attribute__ ((visibility("default")))
  #define TINYMOVR_ROS2_IMPORT
  #if __GNUC__ >= 4
    #define TINYMOVR_ROS2_PUBLIC __attribute__ ((visibility("default")))
    #define TINYMOVR_ROS2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TINYMOVR_ROS2_PUBLIC
    #define TINYMOVR_ROS2_LOCAL
  #endif
  #define TINYMOVR_ROS2_PUBLIC_TYPE
#endif

#endif  // TINYMOVR_ROS2__VISIBILITY_CONTROL_HPP_
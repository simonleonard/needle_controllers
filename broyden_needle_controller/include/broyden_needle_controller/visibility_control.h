#ifndef BROYDEN_NEEDLE_CONTROLLER_H_
#define BROYDEN_NEEDLE_CONTROLLER_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BROYDEN_NEEDLE_CONTROLLER_EXPORT __attribute__((dllexport))
#define BROYDEN_NEEDLE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define BROYDEN_NEEDLE_CONTROLLER_EXPORT __declspec(dllexport)
#define BROYDEN_NEEDLE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef BROYDEN_NEEDLE_CONTROLLER_BUILDING_DLL
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC BROYDEN_NEEDLE_CONTROLLER_EXPORT
#else
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC BROYDEN_NEEDLE_CONTROLLER_IMPORT
#endif
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC_TYPE BROYDEN_NEEDLE_CONTROLLER_PUBLIC
#define BROYDEN_NEEDLE_CONTROLLER_LOCAL
#else
#define BROYDEN_NEEDLE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define BROYDEN_NEEDLE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define BROYDEN_NEEDLE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC
#define BROYDEN_NEEDLE_CONTROLLER_LOCAL
#endif
#define BROYDEN_NEEDLE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // BROYDEN_NEEDLE_CONTROLLER_H_

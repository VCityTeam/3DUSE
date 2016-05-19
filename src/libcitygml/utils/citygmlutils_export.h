
#ifndef CITYGMLUTILS_EXPORT_H
#define CITYGMLUTILS_EXPORT_H

#ifdef CITYGMLUTILS_STATIC_DEFINE
#  define CITYGMLUTILS_EXPORT
#  define CITYGMLUTILS_NO_EXPORT
#else
#  ifndef CITYGMLUTILS_EXPORT
#    ifdef citygmlutils_EXPORTS
        /* We are building this library */
#      define CITYGMLUTILS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CITYGMLUTILS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CITYGMLUTILS_NO_EXPORT
#    define CITYGMLUTILS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CITYGMLUTILS_DEPRECATED
#  define CITYGMLUTILS_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CITYGMLUTILS_DEPRECATED_EXPORT
#  define CITYGMLUTILS_DEPRECATED_EXPORT CITYGMLUTILS_EXPORT CITYGMLUTILS_DEPRECATED
#endif

#ifndef CITYGMLUTILS_DEPRECATED_NO_EXPORT
#  define CITYGMLUTILS_DEPRECATED_NO_EXPORT CITYGMLUTILS_NO_EXPORT CITYGMLUTILS_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CITYGMLUTILS_NO_DEPRECATED
#endif

#endif


#ifndef CITYGML_EXPORT_H
#define CITYGML_EXPORT_H

#ifdef CITYGML_STATIC_DEFINE
#  define CITYGML_EXPORT
#  define CITYGML_NO_EXPORT
#else
#  ifndef CITYGML_EXPORT
#    ifdef citygml_EXPORTS
        /* We are building this library */
#      define CITYGML_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CITYGML_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CITYGML_NO_EXPORT
#    define CITYGML_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CITYGML_DEPRECATED
#  define CITYGML_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CITYGML_DEPRECATED_EXPORT
#  define CITYGML_DEPRECATED_EXPORT CITYGML_EXPORT CITYGML_DEPRECATED
#endif

#ifndef CITYGML_DEPRECATED_NO_EXPORT
#  define CITYGML_DEPRECATED_NO_EXPORT CITYGML_NO_EXPORT CITYGML_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CITYGML_NO_DEPRECATED
#endif

#endif

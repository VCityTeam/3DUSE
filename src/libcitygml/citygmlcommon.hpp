#ifndef __CITYGML_COMMON_HPP__
#define __CITYGML_COMMON_HPP__
////////////////////////////////////////////////////////////////////////////////
#define LIBCITYGML_VERSION_MAJOR 0
#define LIBCITYGML_VERSION_MINOR 1
#define LIBCITYGML_VERSION_REVISION 4

#define LIBCITYGML_VERSIONSTR "0.1.4"

#if defined( _MSC_VER ) && defined( LIBCITYGML_DYNAMIC )
#   ifdef LIBCITYGML_BUILD
#       define LIBCITYGML_EXPORT __declspec( dllexport )
#   else
#       define LIBCITYGML_EXPORT __declspec( dllimport )
#   endif
#else
#   define LIBCITYGML_EXPORT
#endif
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_COMMON_HPP__

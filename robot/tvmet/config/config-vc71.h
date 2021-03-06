/*
 * Tiny Vector Matrix Library
 * Dense Vector Matrix Libary of Tiny size using Expression Templates
 *
 * Copyright (C) 2001 - 2003 Olaf Petzold <opetzold@users.sourceforge.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: config-vc71.h.in,v 1.2 2004-11-04 16:47:12 opetzold Exp $
 */

#ifndef TVMET_CONFIG_VC71_H
#define TVMET_CONFIG_VC71_H


/*******************************************************************
 * equivalent hand made header to configure.ac's autoheader.
 ******************************************************************/


/* define if the compiler has complex<T> */
#ifndef TVMET_HAVE_COMPLEX
#define TVMET_HAVE_COMPLEX 1
#endif

/* define if the compiler has complex math functions */
#ifndef TVMET_HAVE_COMPLEX_MATH1
#define TVMET_HAVE_COMPLEX_MATH1 1
#endif

/* define if the compiler has more complex math functions */
/* #undef TVMET_HAVE_COMPLEX_MATH2 */

/* Define to 1 if you have the <dlfcn.h> header file. */
#ifdef TVMET_HAVE_DLFCN_H
#undef TVMET_HAVE_DLFCN_H
#endif

/* Define to 1 if you have the `floor' function. */
#ifndef TVMET_HAVE_FLOOR
#define TVMET_HAVE_FLOOR  1
#endif

/* Define if the compiler supports IEEE math library */
#ifndef TVMET_HAVE_IEEE_MATH
#define TVMET_HAVE_IEEE_MATH 1
#endif

/* Define to 1 if you have the <inttypes.h> header file. */
#ifdef TVMET_HAVE_INTTYPES_H
#undef TVMET_HAVE_INTTYPES_H
#endif

/* Define to 1 if you have the `dl' library (-ldl). */
#ifdef TVMET_HAVE_LIBDL
#undef TVMET_HAVE_LIBDL
#endif

/* Define to 1 if long double works and has more range or precision than
   double. */
#ifndef TVMET_HAVE_LONG_DOUBLE
#define TVMET_HAVE_LONG_DOUBLE  1
#endif

/* Define if the compiler supports the long_long type */
// enable MS extension for long long
#ifndef TVMET_HAVE_LONG_LONG
#define TVMET_HAVE_LONG_LONG 1
#endif

/* Define to 1 if you have the <memory.h> header file. */
#ifndef TVMET_HAVE_MEMORY_H
#define TVMET_HAVE_MEMORY_H  1
#endif

/* Define if the compiler supports the mutable keyword */
#ifndef TVMET_HAVE_MUTABLE
#define TVMET_HAVE_MUTABLE 1
#endif

/* Define if the compiler implements namespaces */
#ifndef TVMET_HAVE_NAMESPACES
#define TVMET_HAVE_NAMESPACES 1
#endif

/* Define if the compiler supports partial specialization */
#ifndef TVMET_HAVE_PARTIAL_SPECIALIZATION
#define TVMET_HAVE_PARTIAL_SPECIALIZATION 1
#endif

/* Define to 1 if you have the `pow' function. */
#ifndef TVMET_HAVE_POW
#define TVMET_HAVE_POW  1
#endif

/* Define to 1 if you have the `rint' function. */

#ifdef TVMET_HAVE_RINT
#undef TVMET_HAVE_RINT
#endif

/* Define to 1 if you have the `sqrt' function. */
#ifndef TVMET_HAVE_SQRT
#define TVMET_HAVE_SQRT  1
#endif

/* Define to 1 if stdbool.h conforms to C99. */
/* #undef TVMET_HAVE_STDBOOL_H */

/* Define to 1 if you have the <stdint.h> header file. */
#ifdef TVMET_HAVE_STDINT_H
#undef TVMET_HAVE_STDINT_H
#endif

/* Define to 1 if you have the <stdlib.h> header file. */
#ifndef TVMET_HAVE_STDLIB_H
#define TVMET_HAVE_STDLIB_H  1
#endif

/* Define to 1 if you have the <strings.h> header file. */
#ifdef TVMET_HAVE_STRINGS_H
#undef TVMET_HAVE_STRINGS_H
#endif

/* Define to 1 if you have the <string.h> header file. */
#ifndef TVMET_HAVE_STRING_H
#define TVMET_HAVE_STRING_H  1
#endif

/* Define if the compiler supports SYSV math library */
/* #undef TVMET_HAVE_SYSV_MATH */

/* Define to 1 if you have the <sys/stat.h> header file. */
#ifdef TVMET_HAVE_SYS_STAT_H
#undef TVMET_HAVE_SYS_STAT_H
#endif

/* Define to 1 if you have the <sys/time.h> header file. */
#ifdef TVMET_HAVE_SYS_TIME_H
#undef TVMET_HAVE_SYS_TIME_H
#endif

/* Define to 1 if you have the <sys/types.h> header file. */
#ifdef TVMET_HAVE_SYS_TYPES_H
#undef TVMET_HAVE_SYS_TYPES_H
#endif

/* Define if the compiler recognizes typename */
#ifndef TVMET_HAVE_TYPENAME
#define TVMET_HAVE_TYPENAME 1
#endif

/* Define to 1 if you have the <unistd.h> header file. */
#ifdef TVMET_HAVE_UNISTD_H
#undef TVMET_HAVE_UNISTD_H
#endif

/* Define to 1 if the system has the type `_Bool'. */
/* #undef TVMET_HAVE__BOOL */

/* Define to the address where bug reports for this package should be sent. */
#ifndef TVMET_PACKAGE_BUGREPORT
#define TVMET_PACKAGE_BUGREPORT  "opetzold@users.sourceforge.net"
#endif

/* Define to the full name of this package. */
#ifndef TVMET_PACKAGE_NAME
#define TVMET_PACKAGE_NAME  "tvmet"
#endif

/* Define to the full name and version of this package. */
#ifndef TVMET_PACKAGE_STRING
#define TVMET_PACKAGE_STRING  "tvmet 1.7.2"
#endif

/* Define to the one symbol short name of this package. */
#ifndef TVMET_PACKAGE_TARNAME
#define TVMET_PACKAGE_TARNAME  "tvmet"
#endif

/* Define to the version of this package. */
#ifndef TVMET_PACKAGE_VERSION
#define TVMET_PACKAGE_VERSION  "1.7.2"
#endif

/* Define to 1 if you have the ANSI C header files. */
#ifndef TVMET_STDC_HEADERS
#define TVMET_STDC_HEADERS  1
#endif

/* Define to 1 if your <sys/time.h> declares `struct tm'. */
/* #undef TVMET_TM_IN_SYS_TIME */

/* Define to empty if `const' does not conform to ANSI C. */
/* #undef _tvmet_const */

/* Define to `__inline__' or `__inline' if that's what the C compiler
   calls it, or to nothing if 'inline' is not supported under any name.  */
#ifndef __cplusplus
/* #undef _tvmet_inline */
#endif

/* Define to equivalent of C99 restrict keyword, or to nothing if this is not
   supported. Do not define if restrict is supported directly. */
// unfortunally, VC++ 7.1 doesn't have restrict.
#ifndef _tvmet_restrict
#define _tvmet_restrict
#endif

/* Define to `unsigned' if <sys/types.h> does not define. */
/* #undef _tvmet_size_t */



/*******************************************************************
 * tvmet's config for special handling on MS VC
 ******************************************************************/


#if defined(_MSC_VER)

/* The undefined case of TVMET_CXX_ALWAYS_INLINE is handled inside
 * tvmet.h, so there there is no need to do this here! */

#else // !defined(_MSC_VER)

   // paranoia
#  warning "config header for MS VC 7.1 included without defined _MSC_VER"

#endif

#endif // TVMET_CONFIG_VC71_H

// Local Variables:
// mode:C++
// End:
//  LocalWords:  autoheader

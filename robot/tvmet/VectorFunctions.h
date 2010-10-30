/*
 * Tiny Vector Matrix Library
 * Dense Vector Matrix Libary of Tiny size using Expression Templates
 *
 * Copyright (C) 2001 - 2007 Olaf Petzold <opetzold@users.sourceforge.net>
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
 * $Id: VectorFunctions.h,v 1.37 2007-06-23 15:58:58 opetzold Exp $
 */

#ifndef TVMET_VECTOR_FUNCTIONS_H
#define TVMET_VECTOR_FUNCTIONS_H

#include <tvmet/Extremum.h>

namespace tvmet {


/*********************************************************
 * PART I: DECLARATION
 *********************************************************/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Vector arithmetic functions add, sub, mul and div
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*
 * function(Vector<T1, Sz>, Vector<T2, Sz>)
 * function(Vector<T, Sz>, XprVector<E, Sz>)
 * function(XprVector<E, Sz>, Vector<T, Sz>)
 */
#define TVMET_DECLARE_MACRO(NAME)				\
template<class T1, class T2, std::size_t Sz>			\
XprVector<							\
  XprBinOp<							\
    Fcnl_##NAME<T1, T2>,					\
    VectorConstReference<T1, Sz>,				\
    VectorConstReference<T2, Sz>				\
  >,								\
  Sz								\
>								\
NAME (const Vector<T1, Sz>& lhs,				\
      const Vector<T2, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;	\
								\
template<class E, class T, std::size_t Sz>			\
XprVector<							\
  XprBinOp<							\
    Fcnl_##NAME<typename E::value_type, T>,			\
    XprVector<E, Sz>,						\
    VectorConstReference<T, Sz>					\
  >,								\
  Sz								\
>								\
NAME (const XprVector<E, Sz>& lhs,				\
      const Vector<T, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;	\
								\
template<class E, class T, std::size_t Sz>			\
XprVector<							\
  XprBinOp<							\
    Fcnl_##NAME<T, typename E::value_type>,			\
    VectorConstReference<T, Sz>,				\
    XprVector<E, Sz>						\
  >,								\
  Sz								\
>								\
NAME (const Vector<T, Sz>& lhs,					\
      const XprVector<E, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;

TVMET_DECLARE_MACRO(add)		// per se element wise
TVMET_DECLARE_MACRO(sub)		// per se element wise
TVMET_DECLARE_MACRO(mul)		// per se element wise
namespace element_wise {
  TVMET_DECLARE_MACRO(div)		// not defined for vectors
}

#undef TVMET_DECLARE_MACRO


/*
 * function(Vector<T, Sz>, POD)
 * function(POD, Vector<T, Sz>)
 * Note: - operations +,-,*,/ are per se element wise
 */
#define TVMET_DECLARE_MACRO(NAME, POD)				\
template<class T, std::size_t Sz>				\
XprVector<							\
  XprBinOp<							\
    Fcnl_##NAME< T, POD >,					\
    VectorConstReference<T, Sz>,				\
    XprLiteral< POD >						\
  >,								\
  Sz								\
>								\
NAME (const Vector<T, Sz>& lhs, 				\
      POD rhs) TVMET_CXX_ALWAYS_INLINE;				\
								\
template<class T, std::size_t Sz>				\
XprVector<							\
  XprBinOp<							\
    Fcnl_##NAME< POD, T>,					\
    XprLiteral< POD >,						\
    VectorConstReference<T, Sz>					\
  >,								\
  Sz								\
>								\
NAME (POD lhs, 							\
      const Vector<T, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;

TVMET_DECLARE_MACRO(add, int)
TVMET_DECLARE_MACRO(sub, int)
TVMET_DECLARE_MACRO(mul, int)
TVMET_DECLARE_MACRO(div, int)

#if defined(TVMET_HAVE_LONG_LONG)
TVMET_DECLARE_MACRO(add, long long int)
TVMET_DECLARE_MACRO(sub, long long int)
TVMET_DECLARE_MACRO(mul, long long int)
TVMET_DECLARE_MACRO(div, long long int)
#endif

TVMET_DECLARE_MACRO(add, float)
TVMET_DECLARE_MACRO(sub, float)
TVMET_DECLARE_MACRO(mul, float)
TVMET_DECLARE_MACRO(div, float)

TVMET_DECLARE_MACRO(add, double)
TVMET_DECLARE_MACRO(sub, double)
TVMET_DECLARE_MACRO(mul, double)
TVMET_DECLARE_MACRO(div, double)

#if defined(TVMET_HAVE_LONG_DOUBLE)
TVMET_DECLARE_MACRO(add, long double)
TVMET_DECLARE_MACRO(sub, long double)
TVMET_DECLARE_MACRO(mul, long double)
TVMET_DECLARE_MACRO(div, long double)
#endif

#undef TVMET_DECLARE_MACRO


#if defined(TVMET_HAVE_COMPLEX)
/*
 * function(Vector<std::complex<T>, Sz>, std::complex<T>)
 * function(std::complex<T>, Vector<std::complex<T>, Sz>)
 * Note: per se element wise
 * \todo type promotion
 */
#define TVMET_DECLARE_MACRO(NAME)					\
template<class T, std::size_t Sz>					\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,			\
    VectorConstReference< std::complex<T>, Sz>,				\
    XprLiteral< std::complex<T> >					\
  >,									\
  Sz									\
>									\
NAME (const Vector<std::complex<T>, Sz>& lhs,				\
      const std::complex<T>& rhs) TVMET_CXX_ALWAYS_INLINE;		\
									\
template<class T, std::size_t Sz>					\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,			\
    XprLiteral< std::complex<T> >,					\
    VectorConstReference< std::complex<T>, Sz>				\
  >,									\
  Sz									\
>									\
NAME (const std::complex<T>& lhs,					\
      const Vector< std::complex<T>, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;

TVMET_DECLARE_MACRO(add)
TVMET_DECLARE_MACRO(sub)
TVMET_DECLARE_MACRO(mul)
TVMET_DECLARE_MACRO(div)

#undef TVMET_DECLARE_MACRO

#endif // defined(TVMET_HAVE_COMPLEX)


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * vector specific functions
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


template<class T, std::size_t Sz>
typename NumericTraits<T>::sum_type
sum(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
typename NumericTraits<T>::sum_type
product(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class T1, class T2, std::size_t Sz>
typename PromoteTraits<T1, T2>::value_type
dot(const Vector<T1, Sz>& lhs,
    const Vector<T2, Sz>& rhs) TVMET_CXX_ALWAYS_INLINE;


template<class T1, class T2>
Vector<typename PromoteTraits<T1, T2>::value_type, 3>
cross(const Vector<T1, 3>& lhs,
      const Vector<T2, 3>& rhs) TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
typename NumericTraits<T>::sum_type
norm1(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
typename NumericTraits<T>::sum_type
norm2(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
XprVector<
  XprBinOp<
    Fcnl_div<T, T>,
    VectorConstReference<T, Sz>,
    XprLiteral< T >
  >,
  Sz
>
normalize(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * min/max unary functions
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

template<class E, std::size_t Sz>
Extremum<typename E::value_type, std::size_t, vector_tag>
maximum(const XprVector<E, Sz>& e); // NOT TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
Extremum<T, std::size_t, vector_tag>
maximum(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class E, std::size_t Sz>
Extremum<typename E::value_type, std::size_t, vector_tag>
minimum(const XprVector<E, Sz>& e); // NOT TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
Extremum<T, std::size_t, vector_tag>
minimum(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class E, std::size_t Sz>
typename E::value_type
max(const XprVector<E, Sz>& e); // NOT TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
T max(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class E, std::size_t Sz>
typename E::value_type
min(const XprVector<E, Sz>& e); // NOT TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
T min(const Vector<T, Sz>& v) TVMET_CXX_ALWAYS_INLINE;


template<class T, std::size_t Sz>
XprVector<
  VectorConstReference<T, Sz>,
  Sz
>
cvector_ref(const T* mem) TVMET_CXX_ALWAYS_INLINE;


/*********************************************************
 * PART II: IMPLEMENTATION
 *********************************************************/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Vector arithmetic functions add, sub, mul and div
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*
 * function(Vector<T1, Sz>, Vector<T2, Sz>)
 * function(Vector<T, Sz>, XprVector<E, Sz>)
 * function(XprVector<E, Sz>, Vector<T, Sz>)
 */
#define TVMET_IMPLEMENT_MACRO(NAME)					\
template<class T1, class T2, std::size_t Sz>				\
inline									\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME<T1, T2>,						\
    VectorConstReference<T1, Sz>,					\
    VectorConstReference<T2, Sz>					\
  >,									\
  Sz									\
>									\
NAME (const Vector<T1, Sz>& lhs, const Vector<T2, Sz>& rhs) {		\
  typedef XprBinOp <							\
    Fcnl_##NAME<T1, T2>,						\
    VectorConstReference<T1, Sz>,					\
    VectorConstReference<T2, Sz>					\
  >							expr_type;	\
  return XprVector<expr_type, Sz>(					\
    expr_type(lhs.const_ref(), rhs.const_ref()));			\
}									\
									\
template<class E, class T, std::size_t Sz>				\
inline									\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME<typename E::value_type, T>,				\
    XprVector<E, Sz>,							\
    VectorConstReference<T, Sz>						\
  >,									\
  Sz									\
>									\
NAME (const XprVector<E, Sz>& lhs, const Vector<T, Sz>& rhs) {		\
  typedef XprBinOp<							\
     Fcnl_##NAME<typename E::value_type, T>,				\
    XprVector<E, Sz>,							\
    VectorConstReference<T, Sz>						\
  > 							 expr_type;	\
  return XprVector<expr_type, Sz>(					\
    expr_type(lhs, rhs.const_ref()));					\
}									\
									\
template<class E, class T, std::size_t Sz>				\
inline									\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME<T, typename E::value_type>,				\
    VectorConstReference<T, Sz>,					\
    XprVector<E, Sz>							\
  >,									\
  Sz									\
>									\
NAME (const Vector<T, Sz>& lhs, const XprVector<E, Sz>& rhs) {		\
  typedef XprBinOp<							\
    Fcnl_##NAME<T, typename E::value_type>,				\
    VectorConstReference<T, Sz>,					\
    XprVector<E, Sz>							\
  > 						 	expr_type;	\
  return XprVector<expr_type, Sz>(					\
    expr_type(lhs.const_ref(), rhs));					\
}

TVMET_IMPLEMENT_MACRO(add)		// per se element wise
TVMET_IMPLEMENT_MACRO(sub)		// per se element wise
TVMET_IMPLEMENT_MACRO(mul)		// per se element wise
namespace element_wise {
  TVMET_IMPLEMENT_MACRO(div)		// not defined for vectors
}

#undef TVMET_IMPLEMENT_MACRO


/*
 * function(Vector<T, Sz>, POD)
 * function(POD, Vector<T, Sz>)
 * Note: - operations +,-,*,/ are per se element wise
 */
#define TVMET_IMPLEMENT_MACRO(NAME, POD)				\
template<class T, std::size_t Sz>					\
inline									\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME< T, POD >,						\
    VectorConstReference<T, Sz>,					\
    XprLiteral< POD >							\
  >,									\
  Sz									\
>									\
NAME (const Vector<T, Sz>& lhs, POD rhs) {				\
  typedef XprBinOp<							\
    Fcnl_##NAME<T, POD >,						\
    VectorConstReference<T, Sz>,					\
    XprLiteral< POD >							\
  >							expr_type;	\
  return XprVector<expr_type, Sz>(					\
    expr_type(lhs.const_ref(), XprLiteral< POD >(rhs)));		\
}									\
									\
template<class T, std::size_t Sz>					\
inline									\
XprVector<								\
  XprBinOp<								\
    Fcnl_##NAME< POD, T>,						\
    XprLiteral< POD >,							\
    VectorConstReference<T, Sz>						\
  >,									\
  Sz									\
>									\
NAME (POD lhs, const Vector<T, Sz>& rhs) {				\
  typedef XprBinOp<							\
    Fcnl_##NAME< POD, T>,						\
    XprLiteral< POD >,							\
    VectorConstReference<T, Sz>						\
  >							expr_type;	\
  return XprVector<expr_type, Sz>(					\
    expr_type(XprLiteral< POD >(lhs), rhs.const_ref()));		\
}

TVMET_IMPLEMENT_MACRO(add, int)
TVMET_IMPLEMENT_MACRO(sub, int)
TVMET_IMPLEMENT_MACRO(mul, int)
TVMET_IMPLEMENT_MACRO(div, int)

#if defined(TVMET_HAVE_LONG_LONG)
TVMET_IMPLEMENT_MACRO(add, long long int)
TVMET_IMPLEMENT_MACRO(sub, long long int)
TVMET_IMPLEMENT_MACRO(mul, long long int)
TVMET_IMPLEMENT_MACRO(div, long long int)
#endif

TVMET_IMPLEMENT_MACRO(add, float)
TVMET_IMPLEMENT_MACRO(sub, float)
TVMET_IMPLEMENT_MACRO(mul, float)
TVMET_IMPLEMENT_MACRO(div, float)

TVMET_IMPLEMENT_MACRO(add, double)
TVMET_IMPLEMENT_MACRO(sub, double)
TVMET_IMPLEMENT_MACRO(mul, double)
TVMET_IMPLEMENT_MACRO(div, double)

#if defined(TVMET_HAVE_LONG_DOUBLE)
TVMET_IMPLEMENT_MACRO(add, long double)
TVMET_IMPLEMENT_MACRO(sub, long double)
TVMET_IMPLEMENT_MACRO(mul, long double)
TVMET_IMPLEMENT_MACRO(div, long double)
#endif

#undef TVMET_IMPLEMENT_MACRO


#if defined(TVMET_HAVE_COMPLEX)
/*
 * function(Vector<std::complex<T>, Sz>, std::complex<T>)
 * function(std::complex<T>, Vector<std::complex<T>, Sz>)
 * Note: per se element wise
 * \todo type promotion
 */
#define TVMET_IMPLEMENT_MACRO(NAME)						\
template<class T, std::size_t Sz>						\
inline										\
XprVector<									\
  XprBinOp<									\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,				\
    VectorConstReference< std::complex<T>, Sz>,					\
    XprLiteral< std::complex<T> >						\
  >,										\
  Sz										\
>										\
NAME (const Vector<std::complex<T>, Sz>& lhs, const std::complex<T>& rhs) {	\
  typedef XprBinOp<								\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,				\
    VectorConstReference< std::complex<T>, Sz>,					\
    XprLiteral< std::complex<T> >						\
  >							expr_type;		\
  return XprVector<expr_type, Sz>(						\
    expr_type(lhs.const_ref(), XprLiteral< std::complex<T> >(rhs)));		\
}										\
										\
template<class T, std::size_t Sz>						\
inline										\
XprVector<									\
  XprBinOp<									\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,				\
    XprLiteral< std::complex<T> >,						\
    VectorConstReference< std::complex<T>, Sz>					\
  >,										\
  Sz										\
>										\
NAME (const std::complex<T>& lhs, const Vector< std::complex<T>, Sz>& rhs) {	\
  typedef XprBinOp<								\
    Fcnl_##NAME< std::complex<T>, std::complex<T> >,				\
    XprLiteral< std::complex<T> >,						\
    VectorConstReference< std::complex<T>, Sz>					\
  >							expr_type;		\
  return XprVector<expr_type, Sz>(						\
    expr_type(XprLiteral< std::complex<T> >(lhs), rhs.const_ref()));		\
}

TVMET_IMPLEMENT_MACRO(add)
TVMET_IMPLEMENT_MACRO(sub)
TVMET_IMPLEMENT_MACRO(mul)
TVMET_IMPLEMENT_MACRO(div)

#undef TVMET_IMPLEMENT_MACRO

#endif // defined(TVMET_HAVE_COMPLEX)


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * vector specific functions
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/**
 * \fn sum(const Vector<T, Sz>& v)
 * \brief Compute the sum of the vector.
 * \ingroup _unary_function
 *
 * Simply compute the sum of the given vector as:
 * \f[
 * \sum_{i = 0}^{Sz-1} v[i]
 * \f]
 */
template<class T, std::size_t Sz>
inline
typename NumericTraits<T>::sum_type
sum(const Vector<T, Sz>& v) {
  return meta::Vector<Sz>::sum(v);
}


/**
 * \fn product(const Vector<T, Sz>& v)
 * \brief Compute the product of the vector elements.
 * \ingroup _unary_function
 *
 * Simply computer the product of the given vector as:
 * \f[
 * \prod_{i = 0}^{Sz - 1} v[i]
 * \f]
 */
template<class T, std::size_t Sz>
inline
typename NumericTraits<T>::sum_type
product(const Vector<T, Sz>& v) {
  return meta::Vector<Sz>::product(v);
}


/**
 * \fn dot(const Vector<T1, Sz>& lhs, const Vector<T2, Sz>& rhs)
 * \brief Compute the dot/inner product
 * \ingroup _binary_function
 *
 * Compute the dot product as:
 * \f[
 * \sum_{i = 0}^{Sz - 1} ( lhs[i] * rhs[i] )
 * \f]
 * where lhs is a column vector and rhs is a row vector, both vectors
 * have the same dimension.
 */
template<class T1, class T2, std::size_t Sz>
inline
typename PromoteTraits<T1, T2>::value_type
dot(const Vector<T1, Sz>& lhs, const Vector<T2, Sz>& rhs) {
  return meta::Vector<Sz>::dot(lhs, rhs);
}


/**
 * \fn cross(const Vector<T1, 3>& lhs, const Vector<T2, 3>& rhs)
 * \brief Compute the cross/outer product
 * \ingroup _binary_function
 * \note working only for vectors of size = 3
 * \todo Implement vector outer product as ET and MT, returning a XprVector
 */
template<class T1, class T2>
inline
Vector<typename PromoteTraits<T1, T2>::value_type, 3>
cross(const Vector<T1, 3>& lhs, const Vector<T2, 3>& rhs) {
  typedef typename PromoteTraits<T1, T2>::value_type	value_type;
  return Vector<value_type, 3>(lhs(1)*rhs(2) - rhs(1)*lhs(2),
			       rhs(0)*lhs(2) - lhs(0)*rhs(2),
			       lhs(0)*rhs(1) - rhs(0)*lhs(1));
}


/**
 * \fn norm1(const Vector<T, Sz>& v)
 * \brief The \f$l_1\f$ norm of a vector v.
 * \ingroup _unary_function
 * The norm of any vector is just the square root of the dot product of
 * a vector with itself, or
 *
 * \f[
 * |Vector<T, Sz> v| = |v| = \sum_{i=0}^{Sz-1}\,|v[i]|
 * \f]
 */
template<class T, std::size_t Sz>
inline
typename NumericTraits<T>::sum_type
norm1(const Vector<T, Sz>& v) {
  return sum(abs(v));
}


/**
 * \fn norm2(const Vector<T, Sz>& v)
 * \brief The euklidian norm (or \f$l_2\f$ norm) of a vector v.
 * \ingroup _unary_function
 * The norm of any vector is just the square root of the dot product of
 * a vector with itself, or
 *
 * \f[
 * |Vector<T, Sz> v| = |v| = \sqrt{ \sum_{i=0}^{Sz-1}\,v[i]^2 }
 * \f]
 *
 * \note The internal cast for Vector<int> avoids warnings on sqrt.
 */
template<class T, std::size_t Sz>
inline
typename NumericTraits<T>::sum_type
norm2(const Vector<T, Sz>& v) {
  return static_cast<T>( std::sqrt(static_cast<typename NumericTraits<T>::float_type>(dot(v, v))) );
}


/**
 * \fn normalize(const Vector<T, Sz>& v)
 * \brief Normalize the given vector.
 * \ingroup _unary_function
 * \sa norm2
 *
 * using the equation:
 * \f[
 * \frac{Vector<T, Sz> v}{\sqrt{ \sum_{i=0}^{Sz-1}\,v[i]^2 }}
 * \f]
 */
template<class T, std::size_t Sz>
inline
XprVector<
  XprBinOp<
    Fcnl_div<T, T>,
    VectorConstReference<T, Sz>,
    XprLiteral< T >
  >,
  Sz
>
normalize(const Vector<T, Sz>& v) {
  typedef XprBinOp<
    Fcnl_div<T, T>,
    VectorConstReference<T, Sz>,
    XprLiteral< T >
  >							expr_type;
  return XprVector<expr_type, Sz>(
    expr_type(v.const_ref(), XprLiteral< T >(norm2(v))));
}


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * min/max unary functions
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/**
 * \fn maximum(const XprVector<E, Sz>& e)
 * \brief Find the maximum of a vector expression
 * \ingroup _unary_function
 */
template<class E, std::size_t Sz>
inline
Extremum<typename E::value_type, std::size_t, vector_tag>
maximum(const XprVector<E, Sz>& e) {
  typedef typename E::value_type 			value_type;

  value_type 						m_max(e(0));
  std::size_t 						m_idx(0);

  // this loop is faster than meta templates!
  for(std::size_t i = 1; i != Sz; ++i) {
    if(e(i) > m_max) {
      m_max = e(i);
      m_idx = i;
    }
  }

  return Extremum<value_type, std::size_t, vector_tag>(m_max, m_idx);
}


/**
 * \fn maximum(const Vector<T, Sz>& v)
 * \brief Find the maximum of a vector
 * \ingroup _unary_function
 */
template<class T, std::size_t Sz>
inline
Extremum<T, std::size_t, vector_tag>
maximum(const Vector<T, Sz>& v) { return maximum(v.as_expr()); }


/**
 * \fn minimum(const XprVector<E, Sz>& e)
 * \brief Find the minimum of a vector expression
 * \ingroup _unary_function
 */
template<class E, std::size_t Sz>
inline
Extremum<typename E::value_type, std::size_t, vector_tag>
minimum(const XprVector<E, Sz>& e) {
  typedef typename E::value_type 			value_type;

  value_type 						m_min(e(0));
  std::size_t 						m_idx(0);

  // this loop is faster than meta templates!
  for(std::size_t i = 1; i != Sz; ++i) {
    if(e(i) < m_min) {
      m_min = e(i);
      m_idx = i;
    }
  }

  return Extremum<value_type, std::size_t, vector_tag>(m_min, m_idx);
}


/**
 * \fn minimum(const Vector<T, Sz>& v)
 * \brief Find the minimum of a vector
 * \ingroup _unary_function
 */
template<class T, std::size_t Sz>
inline
Extremum<T, std::size_t, vector_tag>
minimum(const Vector<T, Sz>& v) { return minimum(v.as_expr()); }


/**
 * \fn max(const XprVector<E, Sz>& e)
 * \brief Find the maximum of a vector expression
 * \ingroup _unary_function
 */
template<class E, std::size_t Sz>
inline
typename E::value_type
max(const XprVector<E, Sz>& e) {
  typedef typename E::value_type 			value_type;

  value_type 						m_max(e(0));

  // this loop is faster than meta templates!
  for(std::size_t i = 1; i != Sz; ++i)
    if(e(i) > m_max)
      m_max = e(i);

  return m_max;
}


/**
 * \fn max(const Vector<T, Sz>& v)
 * \brief Find the maximum of a vector
 * \ingroup _unary_function
 */
template<class T, std::size_t Sz>
inline
T max(const Vector<T, Sz>& v) {
  typedef T			 			value_type;
  typedef typename Vector<T, Sz>::const_iterator	const_iterator;

  const_iterator					iter(v.begin());
  const_iterator					last(v.end());
  value_type 						temp(*iter);

  for( ; iter != last; ++iter)
    if(*iter > temp)
      temp = *iter;

  return temp;
}


/**
 * \fn min(const XprVector<E, Sz>& e)
 * \brief Find the minimum of a vector expression
 * \ingroup _unary_function
 */
template<class E, std::size_t Sz>
inline
typename E::value_type
min(const XprVector<E, Sz>& e) {
  typedef typename E::value_type 			value_type;

  value_type 						m_min(e(0));

  // this loop is faster than meta templates!
  for(std::size_t i = 1; i != Sz; ++i)
    if(e(i) < m_min)
      m_min = e(i);

  return m_min;
}


/**
 * \fn min(const Vector<T, Sz>& v)
 * \brief Find the minimum of a vector
 * \ingroup _unary_function
 */
template<class T, std::size_t Sz>
inline
T min(const Vector<T, Sz>& v) {
  typedef T			 			value_type;
  typedef typename Vector<T, Sz>::const_iterator	const_iterator;

  const_iterator					iter(v.begin());
  const_iterator					last(v.end());
  value_type 						temp(*iter);

  for( ; iter != last; ++iter)
    if(*iter < temp)
      temp = *iter;

  return temp;
}


/**
 * \fn cvector_ref(const T* mem)
 * \brief Creates an expression wrapper for a C like vector arrays.
 * \ingroup _unary_function
 *
 * This is like creating a vector of external data, as described
 * at \ref construct. With this function you wrap an expression
 * around a C style vector array and you can operate directly with it
 * as usual.
 *
 * \par Example:
 * \code
 * static float vertices[N][3] = {
 *   {-1,  0,  1}, { 1,  0,  1}, ...
 * };
 * ...
 * typedef Vector<float, 3>			vector_type;
 * ...
 * vector_type V( cross(cvector_ref<float, 3>(&vertices[0][0]),
 *                      cvector_ref<float, 3>(&vertices[1][0])) );
 * \endcode
 *
 * \since release 1.6.0
 */
template<class T, std::size_t Sz>
inline
XprVector<
  VectorConstReference<T, Sz>,
  Sz
>
cvector_ref(const T* mem) {
  typedef VectorConstReference<T, Sz>		expr_type;

  return XprVector<expr_type, Sz>(expr_type(mem));
}


} // namespace tvmet

#endif // TVMET_VECTOR_FUNCTIONS_H

// Local Variables:
// mode:C++
// tab-width:8
// End:

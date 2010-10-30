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
 * $Id: BinaryFunctionals.h,v 1.24 2007-06-23 15:58:58 opetzold Exp $
 */

#ifndef TVMET_BINARY_FUNCTIONAL_H
#define TVMET_BINARY_FUNCTIONAL_H

namespace tvmet {


/**
 * \class Fcnl_assign BinaryFunctionals.h "tvmet/BinaryFunctionals.h"
 * \brief Binary operator for assign operations.
 *
 * Unfortunally we have sometimes to cast on assign operations e.g.,
 * on assign on different POD. So we avoid warnings.
 */
template <class T1, class T2>
struct Fcnl_assign : public BinaryFunctional {
  static inline
  void apply_on(T1& _tvmet_restrict lhs, T2 rhs) {
    lhs = static_cast<T1>(rhs);
  }

  static
  void print_xpr(std::ostream& os, std::size_t l=0) {
    os << IndentLevel(l) << "fcnl_assign<T1="
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"
       << std::endl;
  }
};


/** \class Fcnl_add_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_sub_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_mul_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_div_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_mod_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_xor_eq		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_and_eq		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_or_eq		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_shl_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_shr_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
#define TVMET_IMPLEMENT_MACRO(NAME, OP)					\
template <class T1, class T2>						\
struct Fcnl_##NAME : public BinaryFunctional {				\
  typedef void						value_type;	\
									\
  static inline								\
  void apply_on(T1& _tvmet_restrict lhs, T2 rhs) {			\
    lhs OP rhs;								\
  }									\
									\
  static								\
  void print_xpr(std::ostream& os, std::size_t l=0) {			\
    os << IndentLevel(l)						\
       << "Fcnl_" << #NAME << "<T1="					\
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"	\
       << std::endl;							\
  }									\
};

TVMET_IMPLEMENT_MACRO(add_eq, +=)
TVMET_IMPLEMENT_MACRO(sub_eq, -=)
TVMET_IMPLEMENT_MACRO(mul_eq, *=)
TVMET_IMPLEMENT_MACRO(div_eq, /=)
TVMET_IMPLEMENT_MACRO(mod_eq, %=)
TVMET_IMPLEMENT_MACRO(xor_eq, ^=)
TVMET_IMPLEMENT_MACRO(and_eq, &=)
TVMET_IMPLEMENT_MACRO(or_eq, |=)
TVMET_IMPLEMENT_MACRO(shl_eq, <<=)
TVMET_IMPLEMENT_MACRO(shr_eq, >>=)

#undef TVMET_IMPLEMENT_MACRO


/** \class Fcnl_add 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_sub 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_mul 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_div 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_mod 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_bitxor		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_bitand		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_bitor		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_shl 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_shr 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
#define TVMET_IMPLEMENT_MACRO(NAME, OP)					\
template <class T1, class T2>						\
struct Fcnl_##NAME : public BinaryFunctional {				\
  typedef typename  PromoteTraits<T1, T2>::value_type	value_type;	\
  									\
  static inline 							\
  value_type apply_on(T1 lhs, T2 rhs) {					\
    return lhs OP rhs;							\
  }									\
  									\
  static 								\
  void print_xpr(std::ostream& os, std::size_t l=0) {			\
    os << IndentLevel(l)						\
       << "Fcnl_" << #NAME << "<T1="					\
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"	\
       << std::endl;							\
  }									\
};

TVMET_IMPLEMENT_MACRO(add, +)
TVMET_IMPLEMENT_MACRO(sub, -)
TVMET_IMPLEMENT_MACRO(mul, *)
TVMET_IMPLEMENT_MACRO(div, /)
TVMET_IMPLEMENT_MACRO(mod, %)
TVMET_IMPLEMENT_MACRO(bitxor, ^)
TVMET_IMPLEMENT_MACRO(bitand, &)
TVMET_IMPLEMENT_MACRO(bitor, |)
TVMET_IMPLEMENT_MACRO(shl, <<)
TVMET_IMPLEMENT_MACRO(shr, >>)

#undef TVMET_IMPLEMENT_MACRO


/** \class Fcnl_greater 	BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_greater_eq 	BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_less 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_less_eq 	BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_not_eq 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_and 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_or 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
#define TVMET_IMPLEMENT_MACRO(NAME, OP)					\
template <class T1, class T2>						\
struct Fcnl_##NAME : public BinaryFunctional {				\
  typedef bool						value_type;	\
  									\
  static inline								\
  bool apply_on(T1 lhs, T2 rhs) {					\
    return lhs OP rhs;							\
  }									\
  									\
  static 								\
  void print_xpr(std::ostream& os, std::size_t l=0) {			\
    os << IndentLevel(l)						\
       << "Fcnl_" << #NAME << "<T1="					\
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"	\
       << std::endl;							\
  }									\
};

TVMET_IMPLEMENT_MACRO(greater, >)
TVMET_IMPLEMENT_MACRO(less, <)
TVMET_IMPLEMENT_MACRO(greater_eq, >=)
TVMET_IMPLEMENT_MACRO(less_eq, <=)
TVMET_IMPLEMENT_MACRO(eq, ==)
TVMET_IMPLEMENT_MACRO(not_eq, !=)
TVMET_IMPLEMENT_MACRO(and, &&)
TVMET_IMPLEMENT_MACRO(or, ||)

#undef TVMET_IMPLEMENT_MACRO


/** \class Fcnl_atan2 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_fmod 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_pow 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
#define TVMET_IMPLEMENT_MACRO(NAME)					\
template <class T1, class T2>						\
struct Fcnl_##NAME : public BinaryFunctional {				\
  typedef typename PromoteTraits<T1, T2>::value_type	value_type;	\
									\
  static inline 							\
  value_type apply_on(T1 lhs, T2 rhs) {					\
    return TVMET_STD_SCOPE(NAME)(lhs, rhs);				\
  }									\
   									\
  static 								\
  void print_xpr(std::ostream& os, std::size_t l=0) {			\
    os << IndentLevel(l)						\
       << "Fcnl_" << #NAME << "<T1="					\
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"	\
       << std::endl;							\
  }									\
};

TVMET_IMPLEMENT_MACRO(atan2)
TVMET_IMPLEMENT_MACRO(fmod)
TVMET_IMPLEMENT_MACRO(pow)

#undef TVMET_IMPLEMENT_MACRO


/** \class Fcnl_drem 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_hypot 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_jn 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
/** \class Fcnl_yn 		BinaryFunctionals.h "tvmet/BinaryFunctionals.h" */
#define TVMET_IMPLEMENT_MACRO(NAME)					\
template <class T1, class T2>						\
struct Fcnl_##NAME : public BinaryFunctional {				\
  typedef typename PromoteTraits<T1, T2>::value_type	value_type;	\
									\
  static inline 							\
  value_type apply_on(T1 lhs, T2 rhs) {					\
    return TVMET_GLOBAL_SCOPE(NAME)(lhs, rhs);				\
  }									\
   									\
  static 								\
  void print_xpr(std::ostream& os, std::size_t l=0) {			\
    os << IndentLevel(l)						\
       << "Fcnl_" << #NAME << "<T1="					\
       << typeid(T1).name() << ", T2=" << typeid(T2).name() << ">,"	\
       << std::endl;							\
  }									\
};

TVMET_IMPLEMENT_MACRO(drem)
TVMET_IMPLEMENT_MACRO(hypot)
TVMET_IMPLEMENT_MACRO(jn)
TVMET_IMPLEMENT_MACRO(yn)

#undef TVMET_IMPLEMENT_MACRO


#if defined(TVMET_HAVE_COMPLEX)
/**
 * \class Fcnl_polar BinaryFunctionals.h "tvmet/BinaryFunctionals.h"
 * \brief %Functional for polar.
 */
template <class T1, class T2> struct Fcnl_polar : public BinaryFunctional { };


/**
 * \class Fcnl_polar<T,T> BinaryFunctionals.h "tvmet/BinaryFunctionals.h"
 * \brief %Functional for polar.
 * \note  This functional is partialy specialized due to the declaration
 *        of %polar in namespace std <tt>complex<T> polar(T, T)</tt>.
 *        This means especially that type promotion isn't avaible here.
 */
template <class T>
struct Fcnl_polar<T,T> : public BinaryFunctional {
  typedef std::complex<T>                               value_type;

  static inline
  value_type apply_on(T lhs, T rhs) {
    return std::polar(lhs, rhs);
  }

  static
  void print_xpr(std::ostream& os, std::size_t l=0) {
    os << IndentLevel(l) << "Fcnl_polar<T1="
       << typeid(T).name() << ", T2=" << typeid(T).name() << ">,"
       << std::endl;
  }
};
#endif // defined(TVMET_HAVE_COMPLEX)


/**
 * \class Fcnl_swap BinaryFunctionals.h "tvmet/BinaryFunctionals.h"
 * \brief Binary operator for swapping values using temporaries.
 */
template <class T1, class T2>
struct Fcnl_swap : public BinaryFunctional {
  static inline
  void apply_on(T1& _tvmet_restrict lhs, T2& _tvmet_restrict rhs) {
    typedef typename  PromoteTraits<T1, T2>::value_type	temp_type;

    temp_type 						temp(lhs);
    lhs = static_cast<T1>(rhs);
    rhs = static_cast<T2>(temp);
  }

  static
  void print_xpr(std::ostream& os, std::size_t l=0) {
    os << IndentLevel(l) << "Fcnl_swap<T1="
       << typeid(T1).name() << ", T2" << typeid(T2).name() << ">,"
       << std::endl;
  }
};


} // namespace tvmet

#endif // TVMET_BINARY_FUNCTIONAL_H

// Local Variables:
// mode:C++
// tab-width:8
// End:

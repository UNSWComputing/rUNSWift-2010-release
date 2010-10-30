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
 * $Id: MMProductTransposed.h,v 1.20 2007-06-23 15:58:59 opetzold Exp $
 */

#ifndef TVMET_XPR_MMPRODUCT_TRANSPOSED_H
#define TVMET_XPR_MMPRODUCT_TRANSPOSED_H

#include <tvmet/meta/Gemm.h>
#include <tvmet/loop/Gemm.h>

namespace tvmet {


/**
 * \class XprMMProductTransposed MMProductTransposed.h "tvmet/xpr/MMProductTransposed.h"
 * \brief Expression for transpose(matrix-matrix product).
 *        Using formula:
 *        \f[
 *        (M_1\,M_2)^T
 *        \f]
 * \note The Rows2 has to be  equal to Cols1.
 *       The result is a (Cols2 x Rows1) matrix.
 */
template<class E1, std::size_t Rows1, std::size_t Cols1,
	 class E2, std::size_t Cols2>
class XprMMProductTransposed
  : public TvmetBase< XprMMProductTransposed<E1, Rows1, Cols1, E2, Cols2> >
{
private:
  XprMMProductTransposed();
  XprMMProductTransposed& operator=(const XprMMProductTransposed&);

public:
  typedef typename PromoteTraits<
    typename E1::value_type,
    typename E2::value_type
  >::value_type							value_type;

public:
  /** Complexity counter. */
  enum {
    ops_lhs   = E1::ops,
    ops_rhs   = E2::ops,
    M = Rows1 * Cols1 * Cols2,
    N = Rows1 * (Cols1-1) * Cols2,
    ops_plus  = M * NumericTraits<value_type>::ops_plus,
    ops_muls  = N * NumericTraits<value_type>::ops_muls,
    ops       = ops_plus + ops_muls,
    use_meta  = Cols2*Rows1 < TVMET_COMPLEXITY_MM_TRIGGER ? true : false
  };

public:
  /** Constructor. */
  explicit XprMMProductTransposed(const E1& lhs, const E2& rhs)
    : m_lhs(lhs), m_rhs(rhs) { }

  /** Copy Constructor. Not explicit! */
#if defined(TVMET_OPTIMIZE_XPR_MANUAL_CCTOR)
  XprMMProductTransposed(const XprMMProductTransposed& e)
    : m_lhs(e.m_lhs), m_rhs(e.m_rhs)
  { }
#endif

private:
  /** Wrapper for meta gemm. */
  static inline
  value_type do_gemm(dispatch<true>, const E1& lhs, const E2& rhs, std::size_t i, std::size_t j) {
    return meta::gemm<Rows1, Cols1,
                      Cols2,
                      0>::prod(lhs, rhs, i, j);
  }

  /** Wrapper for loop gemm. */
  static inline
  value_type do_gemm(dispatch<false>, const E1& lhs, const E2& rhs, std::size_t i, std::size_t j) {
    return loop::gemm<Rows1, Cols1, Cols2>::prod(lhs, rhs, i, j);
  }

public:
  /** index operator for arrays/matrices */
  value_type operator()(std::size_t i, std::size_t j) const {
    TVMET_RT_CONDITION((i < Cols2) && (j < Rows1), "XprMMProductTransposed Bounce Violation")
    return do_gemm(dispatch<use_meta>(), m_lhs, m_rhs, j, i);
  }

public: // debugging Xpr parse tree
  void print_xpr(std::ostream& os, std::size_t l=0) const {
    os << IndentLevel(l++)
       << "XprMMProductTransposed["
       << (use_meta ? "M" :  "L") << ", O=" << ops
       << ", (O1=" << ops_lhs << ", O2=" << ops_rhs << ")]<"
       << std::endl;
    m_lhs.print_xpr(os, l);
    os << IndentLevel(l)
       << "R1=" << Rows1 << ", C1=" << Cols1 << ",\n";
    m_rhs.print_xpr(os, l);
    os << IndentLevel(l)
       << "C2=" << Cols2 << ",\n"
       << IndentLevel(l)
       << "\n"
       << IndentLevel(--l)
       << ">," << std::endl;
  }

private:
  const E1		 				m_lhs;
  const E2						m_rhs;
};


} // namespace tvmet

#endif // TVMET_XPR_MMPRODUCT_TRANSPOSED_H

// Local Variables:
// mode:C++
// tab-width:8
// End:

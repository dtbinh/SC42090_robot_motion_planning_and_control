/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            K.U. Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#ifndef CASADI_CUSTOM_FUNCTION_HPP
#define CASADI_CUSTOM_FUNCTION_HPP

#include "function.hpp"
#include <string>

namespace casadi {

/** \brief  Forward declaration of internal class */
class CustomFunctionInternal;

// Forward declaration
class CustomFunction;

/** \brief  Interface to a custom function

  \author Joel Andersson
  \date 2010
*/
class CASADI_EXPORT CustomFunction : public Function {

public:

/** \brief  default constructor */
  CustomFunction();

  ///@{
  /** \brief Constructor (new syntax, includes initialization) */
  CustomFunction(const std::string& name, const CustomEvaluate &c_fcn,
                 const std::vector<Sparsity>& inputscheme,
                 const std::vector<Sparsity>& outputscheme,
                 const Dict& opts=Dict());

  CustomFunction(const std::string& name, const CustomEvaluate &c_fcn,
                 const std::pair< SparsityDict, std::vector<std::string> >& inputscheme,
                 const std::vector<Sparsity>& outputscheme,
                 const Dict& opts=Dict());

  CustomFunction(const std::string& name, const CustomEvaluate &c_fcn,
                 const std::vector<Sparsity>& inputscheme,
                 const std::pair< SparsityDict, std::vector<std::string> >& outputscheme,
                 const Dict& opts=Dict());

  CustomFunction(const std::string& name, const CustomEvaluate &c_fcn,
                 const std::pair< SparsityDict, std::vector<std::string> >& inputscheme,
                 const std::pair< SparsityDict, std::vector<std::string> >& outputscheme,
                 const Dict& opts=Dict());
  ///@}

#ifdef WITH_DEPRECATED_FEATURES
  ///@{
  /** \brief [DEPRECATED] Constructor */
  CustomFunction(const CustomEvaluate &c_fcn,
                 const std::vector<Sparsity>& inputscheme,
                 const std::vector<Sparsity>& outputscheme);

  CustomFunction(const CustomEvaluate &c_fcn,
                 const std::pair< SparsityDict, std::vector<std::string> >& inputscheme,
                 const std::vector<Sparsity>& outputscheme);

  CustomFunction(const CustomEvaluate &c_fcn,
                 const std::vector<Sparsity>& inputscheme,
                 const std::pair< SparsityDict, std::vector<std::string> >& outputscheme);

  CustomFunction(const CustomEvaluate &c_fcn,
                 const std::pair< SparsityDict, std::vector<std::string> >& inputscheme,
                 const std::pair< SparsityDict, std::vector<std::string> >& outputscheme);
  ///@}
#endif // WITH_DEPRECATED_FEATURES

  /** \brief  Access functions of the node */
  CustomFunctionInternal* operator->();

  /** \brief  Const access functions of the node */
  const CustomFunctionInternal* operator->() const;

  /// Check if a particular cast is allowed
  static bool testCast(const SharedObjectNode* ptr);

}; // class CustomFunction



} // namespace casadi


#endif // CASADI_CUSTOM_FUNCTION_HPP

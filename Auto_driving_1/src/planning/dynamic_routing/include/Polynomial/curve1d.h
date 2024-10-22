/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
**/
#ifndef CURVE1D_H
#define CURVE1D_H

#include <string>

// Base type for various types of 1-dimensional curves

class Curve1d
{
public:
  Curve1d() = default;

  virtual ~Curve1d() = default;

  virtual double Evaluate(const std::uint32_t order, const double param) const = 0;

  virtual double ParamLength() const = 0;

  virtual std::string ToString() const = 0;
};
#endif
/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * Modified function input and used only some functions
 **/
#pragma once

#include <memory>
#include "qp_solver.h"
#include "piecewise_linear_constraint.h"
#include "piecewise_linear_kernel.h"

class PiecewiseLinearGenerator
{
public:
    // x = f(t)
    PiecewiseLinearGenerator(const uint32_t num_of_segments,
                             const double unit_segment);
    virtual ~PiecewiseLinearGenerator() = default;

    PiecewiseLinearConstraint *mutable_constraint();

    PiecewiseLinearKernel *mutable_kernel();

    // solve
    bool Solve();

    // results
    Eigen::MatrixXd params() const { return qp_solver_->params(); }

private:
    const uint32_t num_of_segments_;
    const double unit_segment_;
    const double total_t_;

    PiecewiseLinearConstraint constraint_;
    PiecewiseLinearKernel kernel_;

    std::unique_ptr<QpSolver> qp_solver_;
};
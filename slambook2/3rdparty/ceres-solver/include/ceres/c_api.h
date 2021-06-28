/* Ceres Solver - A fast non-linear least squares minimizer
 * Copyright 2015 Google Inc. All rights reserved.
 * http://ceres-solver.org/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Google Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: mierle@gmail.com (Keir Mierle)
 *
 * A minimal C API for Ceres. Not all functionality is included. This API is
 * not intended for clients of Ceres, but is instead intended for easing the
 * process of binding Ceres to other languages.
 *
 * Currently this is a work in progress.
 */

#ifndef CERES_PUBLIC_C_API_H_
#define CERES_PUBLIC_C_API_H_

#include "ceres/internal/port.h"
#include "ceres/internal/disable_warnings.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init the Ceres private data. Must be called before anything else. */
CERES_EXPORT void ceres_init();

/* Equivalent to CostFunction::Evaluate() in the C++ API.
 *
 * The user may keep private information inside the opaque user_data object.
 * The pointer here is the same one passed in the ceres_add_residual_block().
 */
typedef int (*ceres_cost_function_t)(void* user_data,
                                     double** parameters,
                                     double* residuals,
                                     double** jacobians);

/* Equivalent to LossFunction::Evaluate() from the C++ API. */
typedef void (*ceres_loss_function_t)(void* user_data,
                                      double squared_norm,
                                      double out[3]);

/* Create callback data for Ceres' stock loss functions.
 *
 * Ceres has several loss functions available by default, and these functions
 * expose those to the C API. To use the stock loss functions, call
 * ceres_create_*_loss_data(), which internally creates an instance of one of
 * the stock loss functions (for example ceres::CauchyLoss), and pass the
 * returned "loss_function_data" along with the ceres_stock_loss_function to
 * ceres_add_residual_block().
 *
 * For example:
 *
 *   void* cauchy_loss_function_data =
 *       ceres_create_cauchy_loss_function_data(1.2, 0.0);
 *   ceres_problem_add_residual_block(
 *       problem,
 *       my_cost_function,
 *       my_cost_function_data,
 *       ceres_stock_loss_function,
 *       cauchy_loss_function_data,
 *       1,
 *       2,
 *       parameter_sizes,
 *       parameter_pointers);
 *    ...
 *    ceres_free_stock_loss_function_data(cauchy_loss_function_data);
 *
 * See loss_function.h for the details of each loss function.
 */
CERES_EXPORT void* ceres_create_huber_loss_function_data(double a);
CERES_EXPORT void* ceres_create_softl1_loss_function_data(double a);
CERES_EXPORT void* ceres_create_cauchy_loss_function_data(double a);
CERES_EXPORT void* ceres_create_arctan_loss_function_data(double a);
CERES_EXPORT void* ceres_create_tolerant_loss_function_data(double a, double b);

/* Free the given stock loss function data. */
CERES_EXPORT void ceres_free_stock_loss_function_data(void* loss_function_data);

/* This is an implementation of ceres_loss_function_t contained within Ceres
 * itself, intended as a way to access the various stock Ceres loss functions
 * from the C API. This should be passed to ceres_add_residual() below, in
 * combination with a user_data pointer generated by
 * ceres_create_stock_loss_function() above. */
CERES_EXPORT void ceres_stock_loss_function(void* user_data,
                                            double squared_norm,
                                            double out[3]);

/* Equivalent to Problem from the C++ API. */
struct ceres_problem_s;
typedef struct ceres_problem_s ceres_problem_t;

struct ceres_residual_block_id_s;
typedef struct ceres_residual_block_id_s ceres_residual_block_id_t;

/* Create and destroy a problem */
/* TODO(keir): Add options for the problem. */
CERES_EXPORT ceres_problem_t* ceres_create_problem();
CERES_EXPORT void ceres_free_problem(ceres_problem_t* problem);

/* Add a residual block. */
CERES_EXPORT ceres_residual_block_id_t* ceres_problem_add_residual_block(
    ceres_problem_t* problem,
    ceres_cost_function_t cost_function,
    void* cost_function_data,
    ceres_loss_function_t loss_function,
    void* loss_function_data,
    int num_residuals,
    int num_parameter_blocks,
    int* parameter_block_sizes,
    double** parameters);

CERES_EXPORT void ceres_solve(ceres_problem_t* problem);

/* TODO(keir): Figure out a way to pass a config in. */

#ifdef __cplusplus
}
#endif

#include "ceres/internal/reenable_warnings.h"

#endif  /* CERES_PUBLIC_C_API_H_ */

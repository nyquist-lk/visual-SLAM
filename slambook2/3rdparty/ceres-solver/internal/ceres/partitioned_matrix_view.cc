// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2017 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Template specialization of PartitionedMatrixView.
//
// ========================================
// THIS FILE IS AUTOGENERATED. DO NOT EDIT.
// THIS FILE IS AUTOGENERATED. DO NOT EDIT.
// THIS FILE IS AUTOGENERATED. DO NOT EDIT.
// THIS FILE IS AUTOGENERATED. DO NOT EDIT.
//=========================================
//
// This file is generated using generate_template_specializations.py.

#include "ceres/linear_solver.h"
#include "ceres/partitioned_matrix_view.h"
#include "ceres/internal/eigen.h"

namespace ceres {
namespace internal {

PartitionedMatrixViewBase*
PartitionedMatrixViewBase::Create(const LinearSolver::Options& options,
                                  const BlockSparseMatrix& matrix) {
#ifndef CERES_RESTRICT_SCHUR_SPECIALIZATION
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 2) &&
     (options.f_block_size == 2)) {
   return new PartitionedMatrixView<2, 2, 2>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 2) &&
     (options.f_block_size == 3)) {
   return new PartitionedMatrixView<2, 2, 3>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 2) &&
     (options.f_block_size == 4)) {
   return new PartitionedMatrixView<2, 2, 4>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 2)) {
   return new PartitionedMatrixView<2, 2, Eigen::Dynamic>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 3) &&
     (options.f_block_size == 3)) {
   return new PartitionedMatrixView<2, 3, 3>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 3) &&
     (options.f_block_size == 4)) {
   return new PartitionedMatrixView<2, 3, 4>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 3) &&
     (options.f_block_size == 6)) {
   return new PartitionedMatrixView<2, 3, 6>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 3) &&
     (options.f_block_size == 9)) {
   return new PartitionedMatrixView<2, 3, 9>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 3)) {
   return new PartitionedMatrixView<2, 3, Eigen::Dynamic>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 3)) {
   return new PartitionedMatrixView<2, 4, 3>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 4)) {
   return new PartitionedMatrixView<2, 4, 4>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 6)) {
   return new PartitionedMatrixView<2, 4, 6>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 8)) {
   return new PartitionedMatrixView<2, 4, 8>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 9)) {
   return new PartitionedMatrixView<2, 4, 9>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 2) &&
     (options.e_block_size == 4)) {
   return new PartitionedMatrixView<2, 4, Eigen::Dynamic>(matrix, options.elimination_groups[0]);
 }
 if (options.row_block_size == 2){
   return new PartitionedMatrixView<2, Eigen::Dynamic, Eigen::Dynamic>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 4) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 2)) {
   return new PartitionedMatrixView<4, 4, 2>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 4) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 3)) {
   return new PartitionedMatrixView<4, 4, 3>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 4) &&
     (options.e_block_size == 4) &&
     (options.f_block_size == 4)) {
   return new PartitionedMatrixView<4, 4, 4>(matrix, options.elimination_groups[0]);
 }
 if ((options.row_block_size == 4) &&
     (options.e_block_size == 4)) {
   return new PartitionedMatrixView<4, 4, Eigen::Dynamic>(matrix, options.elimination_groups[0]);
 }

#endif
  VLOG(1) << "Template specializations not found for <"
          << options.row_block_size << ","
          << options.e_block_size << ","
          << options.f_block_size << ">";
  return new PartitionedMatrixView<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic>(
               matrix, options.elimination_groups[0]);
};

}  // namespace internal
}  // namespace ceres

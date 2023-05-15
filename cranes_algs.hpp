///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>

#include <math.h>

#include "cranes_types.hpp"

namespace cranes 
{
// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) 
{
  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);

  // Compute maximum path length, and check that it is legal.
  const size_t max_steps = setting.rows() + setting.columns() - 2;
  assert(max_steps < 64);

  path best(setting);

  for (size_t steps = 1; steps <= max_steps; ++steps) 
  {
    uint64_t Pos_Int = uint64_t(1) << steps;

    for (uint64_t MyBit = 0; MyBit < Pos_Int; ++MyBit) 
    {
      path candidate(setting);
      bool status = true;

      for (size_t i = 0; i < steps; ++i) 
      {
        // add a path not exceeding <steps> binary values to candidate using bit shiftting.
        size_t bit = (MyBit >> i) & 1; 
        if (bit == 1) 
        {
          if (candidate.is_step_valid(STEP_DIRECTION_EAST)) 
          {
            candidate.add_step(STEP_DIRECTION_EAST);
          }
          else status = false;
        }
        else 
        {
          if (candidate.is_step_valid(STEP_DIRECTION_SOUTH)) 
          {
            candidate.add_step(STEP_DIRECTION_SOUTH);
          }
          else status = false;
        }  
      }
      if (status && (candidate.total_cranes() > best.total_cranes())) 
      {
        best = candidate;
      }
    }
  }
  return best;
}










// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
path crane_unloading_dyn_prog(const grid& setting) 
{
  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);
  using cell_type = std::optional<path>;
  std::vector<std::vector<cell_type>> A(setting.rows(), std::vector<cell_type>(setting.columns()));
  A[0][0] = path(setting);
  assert(A[0][0].has_value());
  for (coordinate row = 0; row < setting.rows(); ++row) 
  {
    for (coordinate col = 0; col < setting.columns(); ++col) 
    {
      if (setting.get(row, col) != CELL_BUILDING) 
      {
        // set the value for A[row][col] as a path collecting most cranes
        // continue;
        cell_type Overhead;
        cell_type At_West;

        if (row > 0 && A[row-1][col].has_value()) 
        {
          Overhead = A[row-1][col];
          if (Overhead->is_step_valid(STEP_DIRECTION_SOUTH)) 
          {
            Overhead->add_step(STEP_DIRECTION_SOUTH);
          }
        }
        if (col > 0 && A[row][col-1].has_value()) 
        {
          At_West = A[row][col-1];
          if (At_West->is_step_valid(STEP_DIRECTION_EAST)) 
          {
            At_West->add_step(STEP_DIRECTION_EAST);
          }
        }
        // whichever of from_above and At_West is non-None and reaches more cranes; 
        // or None if both from_above and At_West are None
        if (Overhead.has_value() && At_West.has_value()) 
        {
          if (Overhead->total_cranes() > At_West->total_cranes()) 
          {
            A[row][col] = Overhead;
          }
          else 
          {
            A[row][col] = At_West;
          }
        }
        if (Overhead.has_value() && !(At_West.has_value())) 
        {
          A[row][col] = Overhead;
        }
        if (At_West.has_value() && !(Overhead.has_value())) 
        {
          A[row][col] = At_West;
        }
      }
    }
  }      
  cell_type* best = &(A[0][0]);
  assert(best->has_value());
  for (coordinate row = 0; row < setting.rows(); ++row) 
  {
    for (coordinate col = 0; col < setting.columns(); ++col) 
    {
      if (A[row][col].has_value() && A[row][col]->total_cranes() > (*best)->total_cranes()) 
      {
        best = &(A[row][col]);
      }
    }
  }
  assert(best->has_value());
  return **best;
}
}

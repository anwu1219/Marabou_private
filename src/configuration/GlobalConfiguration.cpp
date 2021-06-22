/*********************                                                        */
/*! \file GlobalConfiguration.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Parth Shah, Derek Huang
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#include "DivideStrategy.h"
#include "GlobalConfiguration.h"
#include "MString.h"
#include <cstdio>

const double GlobalConfiguration::SOFTMAX_BETA = 100;

const double GlobalConfiguration::EXPONENTIAL_MOVING_AVERAGE_ALPHA = 0.5;

// Use the polarity metrics to decide which branch to take first in a case split
// and how to repair a ReLU constraint.
const bool GlobalConfiguration::USE_POLARITY_BASED_DIRECTION_HEURISTICS = true;

const double GlobalConfiguration::DEFAULT_EPSILON_FOR_COMPARISONS = 1e-9;
const unsigned GlobalConfiguration::DEFAULT_DOUBLE_TO_STRING_PRECISION = 10;
const unsigned long long GlobalConfiguration::STATISTICS_PRINTING_FREQUENCY = 5;
const DivideStrategy GlobalConfiguration::SPLITTING_HEURISTICS = DivideStrategy::ReLUViolation;
const unsigned GlobalConfiguration::INTERVAL_SPLITTING_FREQUENCY = 1;
const unsigned GlobalConfiguration::INTERVAL_SPLITTING_THRESHOLD = 10;

const unsigned GlobalConfiguration::ROW_BOUND_TIGHTENER_SATURATION_ITERATIONS = 20;

const bool GlobalConfiguration::PREPROCESS_INPUT_QUERY = true;
const bool GlobalConfiguration::PREPROCESSOR_ELIMINATE_VARIABLES = true;
const bool GlobalConfiguration::PREPROCESSOR_PL_CONSTRAINTS_ADD_AUX_EQUATIONS = true;
const double GlobalConfiguration::PREPROCESSOR_ALMOST_FIXED_THRESHOLD = 1e-9;
const bool GlobalConfiguration::PREPROCESSOR_MERGE_CONSECUTIVE_WEIGHTED_SUMS = false;

const bool GlobalConfiguration::WARM_START = false;

const double GlobalConfiguration::RELU_CONSTRAINT_COMPARISON_TOLERANCE = 1e-8;
const double GlobalConfiguration::ABS_CONSTRAINT_COMPARISON_TOLERANCE = 1e-8;

const bool GlobalConfiguration::ONLY_AUX_INITIAL_BASIS = false;

const unsigned GlobalConfiguration::EXPLICIT_BOUND_TIGHTENING_DEPTH_THRESHOLD = 0;
const GlobalConfiguration::ExplicitBasisBoundTighteningType GlobalConfiguration::EXPLICIT_BASIS_BOUND_TIGHTENING_TYPE =
    GlobalConfiguration::COMPUTE_INVERTED_BASIS_MATRIX;
const bool GlobalConfiguration::EXPLICIT_BOUND_TIGHTENING_UNTIL_SATURATION = false;

const unsigned GlobalConfiguration::REFACTORIZATION_THRESHOLD = 100;
const GlobalConfiguration::BasisFactorizationType GlobalConfiguration::BASIS_FACTORIZATION_TYPE =
    GlobalConfiguration::SPARSE_FORREST_TOMLIN_FACTORIZATION;

const unsigned GlobalConfiguration::POLARITY_CANDIDATES_THRESHOLD = 5;

const unsigned GlobalConfiguration::DNC_DEPTH_THRESHOLD = 5;

// Not in use
const double GlobalConfiguration::SPARSE_FORREST_TOMLIN_DIAGONAL_ELEMENT_TOLERANCE = 0.00001;
const double GlobalConfiguration::GAUSSIAN_ELIMINATION_PIVOT_SCALE_THRESHOLD = 0.1;

#ifdef ENABLE_GUROBI
const unsigned GlobalConfiguration::GUROBI_NUMBER_OF_THREADS = 1;
const bool GlobalConfiguration::GUROBI_LOGGING = false;
#endif // ENABLE_GUROBI

// Logging - note that it is enabled only in Debug mode
const bool GlobalConfiguration::DNC_MANAGER_LOGGING = false;
const bool GlobalConfiguration::ENGINE_LOGGING = true;
const bool GlobalConfiguration::TABLEAU_LOGGING = false;
const bool GlobalConfiguration::SMT_CORE_LOGGING = true;
const bool GlobalConfiguration::BASIS_FACTORIZATION_LOGGING = false;
const bool GlobalConfiguration::PREPROCESSOR_LOGGING = true;
const bool GlobalConfiguration::INPUT_QUERY_LOGGING = false;
const bool GlobalConfiguration::GAUSSIAN_ELIMINATION_LOGGING = false;
const bool GlobalConfiguration::QUERY_LOADER_LOGGING = false;
const bool GlobalConfiguration::NETWORK_LEVEL_REASONER_LOGGING = false;
const bool GlobalConfiguration::PLCONSTRAINT_LOGGING= false;
const bool GlobalConfiguration::HEURISTIC_COST_MANAGER_LOGGING= false;
const bool GlobalConfiguration::PSEUDO_COST_TRACKER_LOGGING= false;

const bool GlobalConfiguration::USE_SMART_FIX = false;
const bool GlobalConfiguration::USE_LEAST_FIX = false;

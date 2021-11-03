/*********************                                                        */
/*! \file HeuristicCostManager.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Parth Shah
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

**/

#ifndef __HeuristicCostManager_h__
#define __HeuristicCostManager_h__

#include "GlobalConfiguration.h"
#include "IEngine.h"
#include "List.h"
#include "LPSolver.h"
#include "NetworkLevelReasoner.h"
#include "PiecewiseLinearConstraint.h"
#include "PseudoCostTracker.h"
#include "Statistics.h"
#include "Vector.h"

#include <memory>
#include <random>

#define COST_LOG( x, ... ) LOG( GlobalConfiguration::HEURISTIC_COST_MANAGER_LOGGING, "HeuristicCostManager: %s\n", x )

class HeuristicCostManager
{
public:

    HeuristicCostManager( IEngine *engine );

    /*
      Reset the HeuristicCostManager
    */
    void reset();

    inline Map<unsigned, double> &getHeuristicCost()
    {
        return _heuristicCost;
    }

    /*
      Create the initial cost function for local search
    */
    void initiateCostFunctionForLocalSearch();

    /*
      Called when local optima is reached but not all PLConstraint is satisfied.

      Return whether after the update, the heuristic cost is guaranteed to descend
    */
    PiecewiseLinearConstraint *updateHeuristicCost();

    void undoLastHeuristicCostUpdate();

    // Go through the cost term for each PLConstraint, check whether it is satisfied.
    // If it is satisfied but the cost term is not zero, flip the cost term so that
    // the cost term is zero.
    void updateCostTermsForSatisfiedPLConstraints();

    void removeCostComponentFromHeuristicCost( PiecewiseLinearConstraint *constraint );

    bool acceptProposedUpdate( double previousCost, double currentCost );

    double computeHeuristicCost();

    void setStatistics( Statistics *statistics );

    void setPLConstraints( List<PiecewiseLinearConstraint *> &plConstraints );

    void setNetworkLevelReasoner( NLR::NetworkLevelReasoner *networkLevelReasoner );

    void setGurobi( LPSolver *gurobi );

    void dumpHeuristicCost();

private:

    IEngine *_engine;
    NLR::NetworkLevelReasoner *_networkLevelReasoner;
    LPSolver *_gurobi;
    Statistics *_statistics;

    List<PiecewiseLinearConstraint *> _plConstraints;

    /*
      The probability to use a noise strategy in local search
    */
    float _noiseParameter;

    /*
      The strategies of local search
    */
    String _initializationStrategy;
    String _flippingStrategy;

    Map<unsigned, double> _heuristicCost;
    Map<PiecewiseLinearConstraint *, PhaseStatus> _previousHeuristicCost;
    Vector<PiecewiseLinearConstraint *> _plConstraintsInHeuristicCost;

    double _probabilityDensityParameter;

    /*
      Based on current assignment
    */
    void initiateCostFunctionForLocalSearchBasedOnCurrentAssignment
    ( const List<PiecewiseLinearConstraint *> &plConstraintsToAdd );

    /*
      Based on input assignment
    */
    void initiateCostFunctionForLocalSearchBasedOnInputAssignment
    ( const List<PiecewiseLinearConstraint *> &plConstraintsToAdd );

    /*
      Pick a phase at uniform random
    */
    void initiateCostFunctionForLocalSearchRandomly
    ( const List<PiecewiseLinearConstraint *> &plConstraintsToAdd );

    /*
      Heuristic to flip the cost component of a PLConstraint:
      following the heuristics from
      https://www.researchgate.net/publication/2637561_Noise_Strategies_for_Improving_Local_Search
      with probability p, flip the cost term of a randomly chosen PLConstraint
      with probability 1 - p, flip the cost term of the PLConstraint that reduces in the greatest decline in the cost
    */
    PiecewiseLinearConstraint *updateHeuristicCostGWSAT();

    PiecewiseLinearConstraint *updateHeuristicCostGWSAT2();

    PiecewiseLinearConstraint *updateHeuristicCostRandomly();
};

#endif // __HeuristicCostManager_h__

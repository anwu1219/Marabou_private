/*********************                                                        */
/*! \file ReluConstraint.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Duligur Ibeling, Christopher Lazarus
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#ifndef __ReluConstraint_h__
#define __ReluConstraint_h__

#include "context/context.h"
#include "context/cdo.h"
#include "Map.h"
#include "PiecewiseLinearConstraint.h"
#include <cmath>

class ReluConstraint : public PiecewiseLinearConstraint
{
public:
    /*
      The f variable is the relu output on the b variable:
      f = relu( b )
    */
    ReluConstraint( unsigned b, unsigned f );
    ReluConstraint( const String &serializedRelu );

    /*
      Get the type of this constraint.
    */
    PiecewiseLinearFunctionType getType() const;

    /*
      Return a clone of the constraint.
    */
    PiecewiseLinearConstraint *duplicateConstraint() const;

    /*
      Register/unregister the constraint with a talbeau.
     */
    void registerAsWatcher( ITableau *tableau );
    void unregisterAsWatcher( ITableau *tableau );

    /*
      These callbacks are invoked when a watched variable's value
      changes, or when its bounds change.
    */
    void notifyLowerBound( unsigned variable, double bound );
    void notifyUpperBound( unsigned variable, double bound );

    /*
      Returns true iff the variable participates in this piecewise
      linear constraint
    */
    bool participatingVariable( unsigned variable ) const;

    /*
      Get the list of variables participating in this constraint.
    */
    List<unsigned> getParticipatingVariables() const;

    /*
      Returns true iff the assignment satisfies the constraint
    */
    bool satisfied() const;

    /*
      Returns the list of case splits that this piecewise linear
      constraint breaks into. These splits need to complementary,
      i.e. if the list is {l1, l2, ..., ln-1, ln},
      then ~l1 /\ ~l2 /\ ... /\ ~ln-1 --> ln.
     */
    List<PiecewiseLinearCaseSplit> getCaseSplits() const;

    /*
      Check if the constraint's phase has been fixed.
    */
    inline bool phaseFixed() const
    {
        DEBUG({

                if ( *_phaseStatus == RELU_PHASE_ACTIVE && _boundManager )
                    {
                        if ( FloatUtils::isNegative( _boundManager->getLowerBound( _b ) ) )
                            {
                                printf( "x%u >= %f\n", _b, _boundManager->getLowerBound( _b ) );
                                ASSERT( false );
                            }
                    }
                if ( *_phaseStatus == RELU_PHASE_INACTIVE && _boundManager )
                    {
                        if ( FloatUtils::isPositive( _boundManager->getUpperBound( _b ) ) )
                            {
                                printf( "x%u <= %f\n", _b, _boundManager->getUpperBound( _b ) );
                                ASSERT( false );
                            }
                    }
            });
        return *_phaseStatus != PHASE_NOT_FIXED;
    }

    /*
      If the constraint's phase has been fixed, get the (valid) case split.
    */
    PiecewiseLinearCaseSplit getValidCaseSplit() const;

    /*
      Preprocessing related functions, to inform that a variable has
      been eliminated completely because it was fixed to some value,
      or that a variable's index has changed (e.g., x4 is now called
      x2). constraintObsolete() returns true iff and the constraint
      has become obsolote as a result of variable eliminations.
    */
    void eliminateVariable( unsigned variable, double fixedValue );
    void updateVariableIndex( unsigned oldIndex, unsigned newIndex );
    bool constraintObsolete() const;

    /*
      Get the tightenings entailed by the constraint.
    */
    void getEntailedTightenings( List<Tightening> &tightenings ) const;

    /*
      Dump the current state of the constraint.
    */
    void dump( String &output ) const;

    /*
      For preprocessing: get any auxiliary equations that this
      constraint would like to add to the equation pool. In the ReLU
      case, this is an equation of the form aux = f - b, where aux is
      non-negative.
    */
    void addAuxiliaryEquations( InputQuery &inputQuery );

    /*
      Ask the piecewise linear constraint to contribute a component to the cost
      function. If implemented, this component should be empty when the constraint is
      satisfied or inactive, and should be non-empty otherwise. Minimizing the returned
      equation should then lead to the constraint being "closer to satisfied".
    */
    virtual void getCostFunctionComponent( Map<unsigned, double> &cost ) const;

    /*
      Returns string with shape: relu, _f, _b
    */
    String serializeToString() const;

    /*
      Get the index of the B and F variables.
    */
    unsigned getB() const;
    unsigned getF() const;

    /*
      Check if the aux variable is in use and retrieve it
    */
    bool auxVariableInUse() const;
    unsigned getAux() const;

    bool dynamicConstraintsInUse() const;

    bool supportPolarity() const;

    /*
      Return the polarity of this ReLU, which computes how symmetric
      the bound of the input to this ReLU is with respect to 0.
      Let LB be the lowerbound, and UB be the upperbound.
      If LB >= 0, polarity is 1.
      If UB <= 0, polarity is -1.
      If LB < 0, and UB > 0, polarity is ( LB + UB ) / (UB - LB).

      We divide the sum by the width of the interval so that the polarity is
      always between -1 and 1. The closer it is to 0, the more symmetric the
      bound is.
    */
    double computePolarity() const;

    void updateScoreBasedOnPolarity();

    void addCostFunctionComponent( Map<unsigned, double> &costTerms, PhaseStatus phaseStatus );

    void addCostFunctionComponent( Map<unsigned, double> &costTerms );

    void addCostFunctionComponentByOutputValue( Map<unsigned, double> &costTerms, double value );

    void getReducedHeuristicCost( double &reducedCost, PhaseStatus &phaseStatusOfReducedCost );

    Vector<PhaseStatus> getAlternativeHeuristicPhaseStatus();

    void removeCostFunctionComponent( Map<unsigned, double> &cost );

private:
    unsigned _b, _f;
    bool _auxVarInUse;
    unsigned _aux;

    PiecewiseLinearCaseSplit getInactiveSplit() const;
    PiecewiseLinearCaseSplit getActiveSplit() const;

    bool _haveEliminatedVariables;

    static String phaseToString( PhaseStatus phase );

    /*
      Return true iff b or f are out of bounds.
    */
    bool haveOutOfBoundVariables() const;
};

#endif // __ReluConstraint_h__

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

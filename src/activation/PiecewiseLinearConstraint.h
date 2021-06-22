/*********************                                                        */
/*! \file PiecewiseLinearConstraint.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Derek Huang, Duligur Ibeling
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

**/

#ifndef __PiecewiseLinearConstraint_h__
#define __PiecewiseLinearConstraint_h__

#include "BoundManager.h"
#include "FloatUtils.h"
#include "LPSolver.h"
#include "ITableau.h"
#include "List.h"
#include "Map.h"
#include "PiecewiseLinearCaseSplit.h"
#include "PiecewiseLinearFunctionType.h"
#include "Queue.h"
#include "Tightening.h"
#include "Vector.h"

#include "context/context.h"
#include "context/cdo.h"
#include "context/cdlist.h"

class Equation;
class ITableau;
class InputQuery;
class String;

#define PLConstraint_LOG(x, ...) LOG(GlobalConfiguration::PLCONSTRAINT_LOGGING, "PLConstraint: %s\n", x)

enum PhaseStatus : unsigned {
    PHASE_NOT_FIXED = 0,
    RELU_PHASE_ACTIVE = 1,
    RELU_PHASE_INACTIVE = 2,
    ABS_PHASE_POSITIVE = 3,
    ABS_PHASE_NEGATIVE = 4,
    SIGN_PHASE_POSITIVE = 5,
    SIGN_PHASE_NEGATIVE = 6
};

class PiecewiseLinearConstraint : public ITableau::VariableWatcher
{
public:
    /*
      A possible fix for a violated piecewise linear constraint: a
      variable whose value should be changed.
    */
    struct Fix
    {
    public:
        Fix( unsigned variable, double value )
            : _variable( variable )
            , _value( value )
        {
        }

        bool operator==( const Fix &other ) const
        {
            return
                _variable == other._variable &&
                FloatUtils::areEqual( _value, other._value );
        }

        unsigned _variable;
        double _value;
    };

    PiecewiseLinearConstraint();
    virtual ~PiecewiseLinearConstraint()
    {
        if ( _constraintActive )
            _constraintActive->deleteSelf();
        if ( _phaseStatus )
            _phaseStatus->deleteSelf();
    }

    bool operator<( const PiecewiseLinearConstraint &other ) const
    {
        return _score < other._score;
    }

    /*
      Get the type of this constraint.
    */
    virtual PiecewiseLinearFunctionType getType() const = 0;

    /*
      Return a clone of the constraint.
    */
    virtual PiecewiseLinearConstraint *duplicateConstraint() const = 0;

    /*
      Register/unregister the constraint with a talbeau.
    */
    virtual void registerAsWatcher( ITableau *tableau ) = 0;
    virtual void unregisterAsWatcher( ITableau *tableau ) = 0;

    virtual void notifyLowerBound( unsigned /* variable */, double /* bound */ ) {}
    virtual void notifyUpperBound( unsigned /* variable */, double /* bound */ ) {}

    /*
      Turn the constraint on/off.
    */
    virtual void setActiveConstraint( bool active )
    {
        ASSERT( _constraintActive )
        *_constraintActive = active;
    }

    virtual bool isActive() const
    {
        return *_constraintActive;
    }

    /*
      Returns true iff the variable participates in this piecewise
      linear constraint.
    */
    virtual bool participatingVariable( unsigned variable ) const = 0;

    /*
      Get the list of variables participating in this constraint.
    */
    virtual List<unsigned> getParticipatingVariables() const = 0;

    /*
      Returns true iff the assignment satisfies the constraint.
    */
    virtual bool satisfied() const = 0;

    /*
      Returns the list of case splits that this piecewise linear
      constraint breaks into. These splits need to complementary,
      i.e. if the list is {l1, l2, ..., ln-1, ln},
      then ~l1 /\ ~l2 /\ ... /\ ~ln-1 --> ln.
    */
    virtual List<PiecewiseLinearCaseSplit> getCaseSplits() const = 0;

    /*
      Check if the constraint's phase has been fixed.
    */
    virtual bool phaseFixed() const = 0;

    /*
      If the constraint's phase has been fixed, get the (valid) case split.
    */
    virtual PiecewiseLinearCaseSplit getValidCaseSplit() const = 0;

    /*
      Dump the current state of the constraint.
    */
    virtual void dump( String & ) const {};

    /*
      Preprocessing related functions, to inform that a variable has been eliminated completely
      because it was fixed to some value, or that a variable's index has changed (e.g., x4 is now
      called x2). constraintObsolete() returns true iff and the constraint has become obsolote
      as a result of variable eliminations.
    */
    virtual void eliminateVariable( unsigned variable, double fixedValue ) = 0;
    virtual void updateVariableIndex( unsigned oldIndex, unsigned newIndex ) = 0;
    virtual bool constraintObsolete() const = 0;

    /*
      Get the tightenings entailed by the constraint.
    */
    virtual void getEntailedTightenings( List<Tightening> &tightenings ) const = 0;

    void setStatistics( Statistics *statistics );

    /*
      For preprocessing: get any auxiliary equations that this constraint would
      like to add to the equation pool.
    */
    virtual void addAuxiliaryEquations( InputQuery &/* inputQuery */ ) {}

    /*
      Produce string representation of the piecewise linear constraint.
      This representation contains only the information necessary to reproduce it
      but does not account for state or change in state during execution. Additionally
      the first string before a comma has the contraint type identifier
      (ie. "relu", "max", etc)
    */
    virtual String serializeToString() const = 0;

    /*
      Return true if and only if this piecewise linear constraint supports
      the polarity metric
    */
    virtual bool supportPolarity() const
    {
        return false;
    }

    double getScore() const
    {
        return _score;
    }

    virtual void updateScoreBasedOnPolarity()
    {
    }

    /*
      Update _score with score
    */
    void setScore( double score )
    {
        _score = score;
    }

    /*
      Retrieve the current lower and upper bounds
    */
    double getLowerBound( unsigned i ) const
    {
        return _lowerBounds[i];
    }

    double getUpperBound( unsigned i ) const
    {
        return _upperBounds[i];
    }

    inline void setAddedHeuristicCost( PhaseStatus phaseStatus )
    {
        _phaseOfHeuristicCost = phaseStatus;
    }

    inline void resetCostFunctionComponent()
    {
        _phaseOfHeuristicCost = PHASE_NOT_FIXED;
    }

    virtual void addCostFunctionComponent( Map<unsigned, double> &, PhaseStatus ) {};

    /*
      Ask the piecewise linear constraint to contribute a component to the cost
      function. If implemented, this component should be empty when the constraint is
      fixed or inactive, and should be non-empty otherwise. Minimizing the returned
      equation should then lead to the constraint being "closer to satisfied".
    */

    virtual void addCostFunctionComponent( Map<unsigned, double> & ) {};

    virtual void addCostFunctionComponentByOutputValue( Map<unsigned, double> &, double ) {};

    /*
      See if picking another phaseStatus as the heuristic cost can reduce the
      cost. Store the amount by which the cost will be reduced in the first argument,
      and store the alternative phaseStatus in the second argument.
    */
    virtual void getReducedHeuristicCost( double &, PhaseStatus & ) {};

    virtual Vector<PhaseStatus> getAlternativeHeuristicPhaseStatus()
    {
        return Vector<PhaseStatus>();
    }

    inline PhaseStatus getPhaseOfHeuristicCost() const
    {
        return _phaseOfHeuristicCost;
    }

    virtual void removeCostFunctionComponent( Map<unsigned, double> & ) {};

    virtual void extractVariableValueFromGurobi( LPSolver & ){};

    void registerBoundManager( BoundManager *boundManager )
    {
        _boundManager = boundManager;
    }

    void registerGurobi( LPSolver *gurobi )
    {
        _gurobi = gurobi;
    }

    void initializeCDOs( CVC4::context::Context *context )
    {
        if ( _context == NULL )
        {
            ASSERT( NULL == _context );
            ASSERT( NULL == _constraintActive );
            ASSERT( NULL == _phaseStatus );
            _context = context;
            _constraintActive = new (true) CVC4::context::CDO<bool>( _context, true );
            _phaseStatus = new (true) CVC4::context::CDO<PhaseStatus>( _context, PHASE_NOT_FIXED );
        }
        else
        {
            _context = context;
            bool constraintActive = *_constraintActive;
            _constraintActive->deleteSelf();
            _constraintActive = new (true) CVC4::context::CDO<bool>( _context, constraintActive );
            PhaseStatus phaseStatus = *_phaseStatus;
            _phaseStatus->deleteSelf();
            _phaseStatus = new (true) CVC4::context::CDO<PhaseStatus>( _context, phaseStatus );
        }
    }

    PhaseStatus getPhaseStatus() const
    {
        return *_phaseStatus;
    };

protected:
    BoundManager *_boundManager;
    LPSolver *_gurobi;

    CVC4::context::Context *_context;
    CVC4::context::CDO<bool> *_constraintActive;
    CVC4::context::CDO<PhaseStatus> *_phaseStatus;

    Map<unsigned, double> _lowerBounds;
    Map<unsigned, double> _upperBounds;

    /*
      The score denotes priority for splitting. When score is negative, the PL constraint
      is not being considered for splitting.
      We pick the PL constraint with the highest score to branch.
     */
    double _score;

    /*
      Statistics collection
    */
    Statistics *_statistics;

    PhaseStatus _phaseOfHeuristicCost;

    /*
      Set the phase status of the constraint. Uses the global PhaseStatus
      enumeration and is initialized to PHASE_NOT_FIXED for all constraints.
     */
    void setPhaseStatus( PhaseStatus phase )
    {
        ASSERT( _phaseStatus );
        *_phaseStatus = phase;
    };

    void reinitializeCDOs()
    {
        if ( _context == nullptr )
        {
            return;
        }
        ASSERT( nullptr != _context );
        ASSERT( nullptr != _constraintActive );
        ASSERT( nullptr != _phaseStatus );

        bool constraintActive = *_constraintActive;
        _constraintActive = new (true) CVC4::context::CDO<bool>( _context, constraintActive );
        PhaseStatus phaseStatus = *_phaseStatus;
        _phaseStatus = new (true) CVC4::context::CDO<PhaseStatus>( _context, phaseStatus );
    };
};

#endif // __PiecewiseLinearConstraint_h__

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

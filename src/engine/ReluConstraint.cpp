/*********************                                                        */
/*! \file ReluConstraint.cpp
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

#include "Debug.h"
#include "DivideStrategy.h"
#include "FloatUtils.h"
#include "GlobalConfiguration.h"
#include "ITableau.h"
#include "InputQuery.h"
#include "MStringf.h"
#include "PiecewiseLinearCaseSplit.h"
#include "ReluConstraint.h"
#include "MarabouError.h"
#include "Statistics.h"
#include "TableauRow.h"

#include "context/cdlist.h"
#include "context/cdo.h"
#include "context/context.h"

#ifdef _WIN32
#define __attribute__(x)
#endif

ReluConstraint::ReluConstraint( unsigned b, unsigned f )
    : _b( b )
    , _f( f )
    , _auxVarInUse( false )
    , _haveEliminatedVariables( false )
{
}

ReluConstraint::ReluConstraint( const String &serializedRelu )
    : _haveEliminatedVariables( false )
{
    String constraintType = serializedRelu.substring( 0, 4 );
    ASSERT( constraintType == String( "relu" ) );

    // Remove the constraint type in serialized form
    String serializedValues = serializedRelu.substring( 5, serializedRelu.length() - 5 );
    List<String> values = serializedValues.tokenize( "," );

    ASSERT( values.size() >= 2 && values.size() <= 3 );

    if ( values.size() == 2 )
    {
        auto var = values.begin();
        _f = atoi( var->ascii() );
        ++var;
        _b = atoi( var->ascii() );

        _auxVarInUse = false;
    }
    else
    {
        auto var = values.begin();
        _f = atoi( var->ascii() );
        ++var;
        _b = atoi( var->ascii() );
        ++var;
        _aux = atoi( var->ascii() );

        _auxVarInUse = true;
    }
}

PiecewiseLinearFunctionType ReluConstraint::getType() const
{
    return PiecewiseLinearFunctionType::RELU;
}

PiecewiseLinearConstraint *ReluConstraint::duplicateConstraint() const
{
    ReluConstraint *clone = new ReluConstraint( _b, _f );
    *clone = *this;
    clone->reinitializeCDOs();
    return clone;
}

void ReluConstraint::registerAsWatcher( ITableau *tableau )
{
    tableau->registerToWatchVariable( this, _b );
    tableau->registerToWatchVariable( this, _f );

    if ( _auxVarInUse )
        tableau->registerToWatchVariable( this, _aux );
}

void ReluConstraint::unregisterAsWatcher( ITableau *tableau )
{
    tableau->unregisterToWatchVariable( this, _b );
    tableau->unregisterToWatchVariable( this, _f );

    if ( _auxVarInUse )
        tableau->unregisterToWatchVariable( this, _aux );
}

void ReluConstraint::notifyLowerBound( unsigned variable, double bound )
{
    if ( !_boundManager )
    {
        if ( _lowerBounds.exists( variable ) && !FloatUtils::gt( bound, _lowerBounds[variable] ) )
            return;
        else
            _lowerBounds[variable] = bound;
    }

    if ( variable == _f && FloatUtils::isPositive( bound ) )
        setPhaseStatus( RELU_PHASE_ACTIVE );
    else if ( variable == _b && !FloatUtils::isNegative( bound ) )
        setPhaseStatus( RELU_PHASE_ACTIVE );
    else if ( variable == _aux && FloatUtils::isPositive( bound ) )
        setPhaseStatus( RELU_PHASE_INACTIVE );

    if ( isActive() && _boundManager )
    {
        // A positive lower bound is always propagated between f and b
        if ( ( variable == _f || variable == _b ) && bound > 0 )
        {
            unsigned partner = ( variable == _f ) ? _b : _f;
            _boundManager->tightenLowerBound( partner, bound );

            // If we're in the active phase, aux should be 0
            if ( _auxVarInUse )
                _boundManager->tightenUpperBound( _aux, 0 );
        }

        // If b is non-negative, we're in the active phase
        else if ( _auxVarInUse && variable == _b && FloatUtils::isZero( bound ) )
        {
            _boundManager->tightenUpperBound( _aux, 0 );
        }

        // A positive lower bound for aux means we're inactive: f is 0, b is non-positive
        // When inactive, b = -aux
        else if ( _auxVarInUse && variable == _aux && bound > 0 )
        {
            _boundManager->tightenUpperBound( _b, -bound );
            _boundManager->tightenUpperBound( _f, 0 );
        }

        // A negative lower bound for b could tighten aux's upper bound
        else if ( _auxVarInUse && variable == _b && bound < 0 )
        {
            _boundManager->tightenUpperBound( _aux, -bound );
        }

        // Also, if for some reason we only know a negative lower bound for f,
        // we attempt to tighten it to 0
        else if ( bound < 0 && variable == _f )
        {
            _boundManager->tightenLowerBound( _f, 0 );
        }
    }
}

void ReluConstraint::notifyUpperBound( unsigned variable, double bound )
{
    if ( !_boundManager )
    {
        if ( _upperBounds.exists( variable ) && !FloatUtils::lt( bound, _upperBounds[variable] ) )
            return;
        else
            _upperBounds[variable] = bound;
    }

    if ( ( variable == _f || variable == _b ) && !FloatUtils::isPositive( bound ) )
        setPhaseStatus( RELU_PHASE_INACTIVE );

    if ( _auxVarInUse && variable == _aux && FloatUtils::isZero( bound ) )
        setPhaseStatus( RELU_PHASE_ACTIVE );

    if ( isActive() && _boundManager )
    {
        if ( variable == _f )
        {
            // Any bound that we learned of f should be propagated to b
            _boundManager->tightenUpperBound( _b, bound );
        }
        else if ( variable == _b )
        {
            if ( !FloatUtils::isPositive( bound ) )
            {
                // If b has a non-positive upper bound, f's upper bound is 0
                _boundManager->tightenUpperBound( _f, 0 );

                if ( _auxVarInUse )
                {
                    // Aux's range is minus the range of b
                    _boundManager->tightenLowerBound( _aux, -bound );
                }
            }
            else
            {
                // b has a positive upper bound, propagate to f
                _boundManager->tightenUpperBound( _f, bound );
            }
        }
        else if ( _auxVarInUse && variable == _aux )
        {
            _boundManager->tightenLowerBound( _b, -bound );
        }
    }
}

bool ReluConstraint::participatingVariable( unsigned variable ) const
{
    return ( variable == _b ) || ( variable == _f ) || ( _auxVarInUse && variable == _aux );
}

List<unsigned> ReluConstraint::getParticipatingVariables() const
{
    return _auxVarInUse?
        List<unsigned>( { _b, _f, _aux } ) :
        List<unsigned>( { _b, _f } );
}

bool ReluConstraint::satisfied() const
{
    if ( !_gurobi )
        throw MarabouError( MarabouError::GUROBI_NOT_AVAILABLE );

    double bValue = _gurobi->getValue( _b );
    double fValue = _gurobi->getValue( _f );

    if ( FloatUtils::isNegative( fValue ) )
        return false;

    if ( FloatUtils::isPositive( fValue ) )
        return FloatUtils::areEqual( bValue, fValue, GlobalConfiguration::RELU_CONSTRAINT_COMPARISON_TOLERANCE );
    else
        return !FloatUtils::isPositive( bValue );
}

List<PiecewiseLinearCaseSplit> ReluConstraint::getCaseSplits() const
{
    if ( *_phaseStatus != PHASE_NOT_FIXED )
        throw MarabouError( MarabouError::REQUESTED_CASE_SPLITS_FROM_FIXED_CONSTRAINT );

    List<PiecewiseLinearCaseSplit> splits;

    if ( _phaseOfHeuristicCost == RELU_PHASE_ACTIVE )
    {
        splits.append( getActiveSplit() );
        splits.append( getInactiveSplit() );
        return splits;
    }
    else
    {
        splits.append( getInactiveSplit() );
        splits.append( getActiveSplit() );
        return splits;
    }
}

PiecewiseLinearCaseSplit ReluConstraint::getInactiveSplit() const
{
    // Inactive phase: b <= 0, f = 0
    PiecewiseLinearCaseSplit inactivePhase;
    inactivePhase.storeBoundTightening( Tightening( _b, 0.0, Tightening::UB ) );
    inactivePhase.storeBoundTightening( Tightening( _f, 0.0, Tightening::UB ) );
    return inactivePhase;
}

PiecewiseLinearCaseSplit ReluConstraint::getActiveSplit() const
{
    // Active phase: b >= 0, b - f = 0
    PiecewiseLinearCaseSplit activePhase;
    activePhase.storeBoundTightening( Tightening( _b, 0.0, Tightening::LB ) );

    if ( _auxVarInUse )
    {
        // Special case: aux var in use.
        // Because aux = f - b and aux >= 0, we just add that aux <= 0.
        activePhase.storeBoundTightening( Tightening( _aux, 0.0, Tightening::UB ) );
    }
    else
    {
        Equation activeEquation( Equation::EQ );
        activeEquation.addAddend( 1, _b );
        activeEquation.addAddend( -1, _f );
        activeEquation.setScalar( 0 );
        activePhase.addEquation( activeEquation );
    }

    return activePhase;
}

PiecewiseLinearCaseSplit ReluConstraint::getValidCaseSplit() const
{
    ASSERT( *_phaseStatus != PHASE_NOT_FIXED );

    if ( *_phaseStatus == RELU_PHASE_ACTIVE )
        return getActiveSplit();

    return getInactiveSplit();
}

void ReluConstraint::dump( String &output ) const
{
    PhaseStatus phase = *_phaseStatus;
    output = Stringf( "ReluConstraint: x%u = ReLU( x%u ). Active? %s. PhaseStatus = %u (%s).\n",
                      _f, _b,
                      *_constraintActive ? "Yes" : "No",
                      phase, phaseToString( phase ).ascii()
                      );

    output += Stringf( "b in [%s, %s], ",
                       Stringf( "%lf", _boundManager->getLowerBound( _b ) ).ascii(),
                       Stringf( "%lf", _boundManager->getUpperBound( _b ) ).ascii() );

    output += Stringf( "f in [%s, %s]",
                       Stringf( "%lf", _boundManager->getLowerBound( _f ) ).ascii(),
                       Stringf( "%lf", _boundManager->getUpperBound( _f ) ).ascii() );

    if ( _auxVarInUse )
    {
        output += Stringf( ". Aux var: %u. Range: [%s, %s]\n",
                           _aux,
                           Stringf( "%lf", _boundManager->getLowerBound( _aux ) ).ascii(),
                           Stringf( "%lf", _boundManager->getUpperBound( _aux ) ).ascii() );
    }
}

void ReluConstraint::updateVariableIndex( unsigned oldIndex, unsigned newIndex )
{
	ASSERT( oldIndex == _b || oldIndex == _f || ( _auxVarInUse && oldIndex == _aux ) );
    ASSERT( !_lowerBounds.exists( newIndex ) &&
            !_upperBounds.exists( newIndex ) &&
            newIndex != _b && newIndex != _f && ( !_auxVarInUse || newIndex != _aux ) );

    if ( _lowerBounds.exists( oldIndex ) )
    {
        _lowerBounds[newIndex] = _lowerBounds.get( oldIndex );
        _lowerBounds.erase( oldIndex );
    }

    if ( _upperBounds.exists( oldIndex ) )
    {
        _upperBounds[newIndex] = _upperBounds.get( oldIndex );
        _upperBounds.erase( oldIndex );
    }

    if ( oldIndex == _b )
        _b = newIndex;
    else if ( oldIndex == _f )
        _f = newIndex;
    else
        _aux = newIndex;
}

void ReluConstraint::eliminateVariable( __attribute__((unused)) unsigned variable,
                                        __attribute__((unused)) double fixedValue )
{
    ASSERT( variable == _b || variable == _f || ( _auxVarInUse && variable == _aux ) );

    DEBUG({
            if ( variable == _f )
            {
                ASSERT( FloatUtils::gte( fixedValue, 0.0 ) );
            }

            if ( variable == _f || variable == _b )
            {
                if ( FloatUtils::gt( fixedValue, 0 ) )
                {
                    ASSERT( *_phaseStatus != RELU_PHASE_INACTIVE );
                }
                else if ( FloatUtils::lt( fixedValue, 0 ) )
                {
                    ASSERT( *_phaseStatus != RELU_PHASE_ACTIVE );
                }
            }
            else
            {
                // This is the aux variable
                if ( FloatUtils::isPositive( fixedValue ) )
                {
                    ASSERT( *_phaseStatus != RELU_PHASE_ACTIVE );
                }
            }
        });

    // In a ReLU constraint, if a variable is removed the entire constraint can be discarded.
    _haveEliminatedVariables = true;
}

bool ReluConstraint::constraintObsolete() const
{
    return _haveEliminatedVariables;
}

void ReluConstraint::getEntailedTightenings( List<Tightening> &tightenings ) const
{
    ASSERT( !_gurobi );
    ASSERT( _lowerBounds.exists( _b ) && _lowerBounds.exists( _f ) &&
            _upperBounds.exists( _b ) && _upperBounds.exists( _f ) );

    ASSERT( !_auxVarInUse || ( _lowerBounds.exists( _aux ) && _upperBounds.exists( _aux ) ) );

    double bLowerBound = _lowerBounds[_b];
    double fLowerBound = _lowerBounds[_f];

    double bUpperBound = _upperBounds[_b];
    double fUpperBound = _upperBounds[_f];

    double auxLowerBound = 0;
    double auxUpperBound = 0;

    if ( _auxVarInUse )
    {
        auxLowerBound = _lowerBounds[_aux];
        auxUpperBound = _upperBounds[_aux];
    }

    // Determine if we are in the active phase, inactive phase or unknown phase
    if ( !FloatUtils::isNegative( bLowerBound ) ||
         FloatUtils::isPositive( fLowerBound ) ||
         ( _auxVarInUse && FloatUtils::isZero( auxUpperBound ) ) )
    {
        // Active case;

        // All bounds are propagated between b and f
        tightenings.append( Tightening( _b, fLowerBound, Tightening::LB ) );
        tightenings.append( Tightening( _f, bLowerBound, Tightening::LB ) );

        tightenings.append( Tightening( _b, fUpperBound, Tightening::UB ) );
        tightenings.append( Tightening( _f, bUpperBound, Tightening::UB ) );

        // Aux is zero
        if ( _auxVarInUse )
        {
            tightenings.append( Tightening( _aux, 0, Tightening::LB ) );
            tightenings.append( Tightening( _aux, 0, Tightening::UB ) );
        }

        tightenings.append( Tightening( _b, 0, Tightening::LB ) );
        tightenings.append( Tightening( _f, 0, Tightening::LB ) );
    }
    else if ( FloatUtils::isNegative( bUpperBound ) ||
              FloatUtils::isZero( fUpperBound ) ||
              ( _auxVarInUse && FloatUtils::isPositive( auxLowerBound ) ) )
    {
        // Inactive case

        // f is zero
        tightenings.append( Tightening( _f, 0, Tightening::LB ) );
        tightenings.append( Tightening( _f, 0, Tightening::UB ) );

        // b is non-positive
        tightenings.append( Tightening( _b, 0, Tightening::UB ) );

        // aux = -b, aux is non-negative
        if ( _auxVarInUse )
        {
            tightenings.append( Tightening( _aux, -bLowerBound, Tightening::UB ) );
            tightenings.append( Tightening( _aux, -bUpperBound, Tightening::LB ) );

            tightenings.append( Tightening( _b, -auxLowerBound, Tightening::UB ) );
            tightenings.append( Tightening( _b, -auxUpperBound, Tightening::LB ) );

            tightenings.append( Tightening( _aux, 0, Tightening::LB ) );
        }
    }
    else
    {
        // Unknown case

        // b and f share upper bounds
        tightenings.append( Tightening( _b, fUpperBound, Tightening::UB ) );
        tightenings.append( Tightening( _f, bUpperBound, Tightening::UB ) );

        // aux upper bound is -b lower bound
        if ( _auxVarInUse )
        {
            tightenings.append( Tightening( _b, -auxUpperBound, Tightening::LB ) );
            tightenings.append( Tightening( _aux, -bLowerBound, Tightening::UB ) );
        }

        // f and aux are always non negative
        tightenings.append( Tightening( _f, 0, Tightening::LB ) );
        if ( _auxVarInUse )
            tightenings.append( Tightening( _aux, 0, Tightening::LB ) );
    }
}

String ReluConstraint::phaseToString( PhaseStatus phase )
{
    switch ( phase )
    {
    case PHASE_NOT_FIXED:
        return "PHASE_NOT_FIXED";

    case RELU_PHASE_ACTIVE:
        return "RELU_PHASE_ACTIVE";

    case RELU_PHASE_INACTIVE:
        return "RELU_PHASE_INACTIVE";

    default:
        return "UNKNOWN";
    }
};

void ReluConstraint::addAuxiliaryEquations( InputQuery &inputQuery )
{
    /*
      We want to add the equation

          f >= b

      Which actually becomes

          f - b - aux = 0

      Lower bound: always non-negative
      Upper bound: when f = 0 and b is minimal, i.e. -b.lb
    */
    ASSERT( _gurobi == NULL );

    // Create the aux variable
    _aux = inputQuery.getNumberOfVariables();
    inputQuery.setNumberOfVariables( _aux + 1 );

    // Create and add the equation
    Equation equation( Equation::EQ );
    equation.addAddend( 1.0, _f );
    equation.addAddend( -1.0, _b );
    equation.addAddend( -1.0, _aux );
    equation.setScalar( 0 );
    inputQuery.addEquation( equation );

    // Adjust the bounds for the new variable
    ASSERT( _lowerBounds.exists( _b ) );
    inputQuery.setLowerBound( _aux, 0 );

    // Generally, aux.ub = -b.lb. However, if b.lb is positive (active
    // phase), then aux.ub needs to be 0
    double auxUpperBound =
        _lowerBounds[_b] > 0 ? 0 : -_lowerBounds[_b];
    inputQuery.setUpperBound( _aux, auxUpperBound );

    // We now care about the auxiliary variable, as well
    _auxVarInUse = true;

}

void ReluConstraint::getCostFunctionComponent( Map<unsigned, double> &cost ) const
{
    // This should not be called for inactive constraints
    ASSERT( isActive() );

    // If the constraint is satisfied, fixed or has OOB components,
    // it contributes nothing
    if ( satisfied() || phaseFixed() || haveOutOfBoundVariables() )
        return;

    // Both variables are within bounds and the constraint is not
    // satisfied or fixed.
    double bValue = _gurobi->getValue( _b );
    double fValue = _gurobi->getValue( _f );

    if ( !cost.exists( _f ) )
        cost[_f] = 0;

    // Case 1: b is non-positive, f is not zero. Cost: f
    if ( !FloatUtils::isPositive( bValue ) )
    {
        ASSERT( !FloatUtils::isZero( fValue ) );
        cost[_f] = cost[_f] + 1;
        return;
    }

    ASSERT( !FloatUtils::isNegative( bValue ) );
    ASSERT( !FloatUtils::isNegative( fValue ) );

    if ( !cost.exists( _b ) )
        cost[_b] = 0;

    // Case 2: both non-negative, not equal, b > f. Cost: b - f
    if ( FloatUtils::gt( bValue, fValue ) )
    {
        cost[_b] = cost[_b] + 1;
        cost[_f] = cost[_f] - 1;
        return;
    }

    // Case 3: both non-negative, not equal, f > b. Cost: f - b
    cost[_b] = cost[_b] - 1;
    cost[_f] = cost[_f] + 1;
    return;
}

bool ReluConstraint::haveOutOfBoundVariables() const
{
    double bValue = _gurobi->getValue( _b );
    double fValue = _gurobi->getValue( _f );

    if ( FloatUtils::gt( _boundManager->getLowerBound( _b ), bValue ) || FloatUtils::lt( _boundManager->getUpperBound( _b ), bValue ) )
        return true;

    if ( FloatUtils::gt( _boundManager->getLowerBound( _f ), fValue ) || FloatUtils::lt( _boundManager->getUpperBound( _f ), fValue ) )
        return true;

    return false;
}

String ReluConstraint::serializeToString() const
{
    // Output format is: relu,f,b,aux
    if ( _auxVarInUse )
        return Stringf( "relu,%u,%u,%u", _f, _b, _aux );

    return Stringf( "relu,%u,%u", _f, _b );
}

unsigned ReluConstraint::getB() const
{
    return _b;
}

unsigned ReluConstraint::getF() const
{
    return _f;
}

bool ReluConstraint::supportPolarity() const
{
    return true;
}

bool ReluConstraint::auxVariableInUse() const
{
    return _auxVarInUse;
}

unsigned ReluConstraint::getAux() const
{
    return _aux;
}

double ReluConstraint::computePolarity() const
{
    double currentLb = _boundManager->getLowerBound( _b );
    double currentUb = _boundManager->getUpperBound( _b );
    if ( currentLb >= 0 ) return 1;
    if ( currentUb <= 0 ) return -1;
    double width = currentUb - currentLb;
    double sum = currentUb + currentLb;
    return sum / width;
}

void ReluConstraint::updateScoreBasedOnPolarity()
{
    _score = std::abs( computePolarity() );
}

void ReluConstraint::addCostFunctionComponent( Map<unsigned, double> &cost,
                                               PhaseStatus phaseStatus )
{
    ASSERT( phaseStatus == RELU_PHASE_ACTIVE ||
            phaseStatus == RELU_PHASE_INACTIVE );

    if ( _phaseOfHeuristicCost == phaseStatus )
        return;

    if ( phaseStatus == RELU_PHASE_INACTIVE )
    {
        PLConstraint_LOG( Stringf( "Cost component: x%u", _f ).ascii() );
        if ( _phaseOfHeuristicCost == RELU_PHASE_ACTIVE )
        {
            ASSERT( cost.exists( _b ) );
            cost[_b] = cost[_b] + 1;
            setAddedHeuristicCost( RELU_PHASE_INACTIVE );

        }
        else
        {
            // To force ReLU phase to be inactive, we add cost _f
            if ( !cost.exists( _f ) )
                cost[_f] = 0;
            cost[_f] = cost[_f] + 1;
            setAddedHeuristicCost( RELU_PHASE_INACTIVE );
        }
    }
    else if ( phaseStatus == RELU_PHASE_ACTIVE )
    {
        PLConstraint_LOG( Stringf( "Cost component: x%u - x%u", _f, _b ).ascii() );
        if ( _phaseOfHeuristicCost == RELU_PHASE_INACTIVE )
        {
            if ( !cost.exists( _b ) )
                cost[_b] = 0;
            cost[_b] = cost[_b] - 1;
            setAddedHeuristicCost( RELU_PHASE_ACTIVE );
        }
        else
        {
            // To force ReLU phase to be active, we add cost _f - _b
            if ( !cost.exists( _f ) )
                cost[_f] = 0;
            if ( !cost.exists( _b ) )
                cost[_b] = 0;
            cost[_f] = cost[_f] + 1;
            cost[_b] = cost[_b] - 1;
            setAddedHeuristicCost( RELU_PHASE_ACTIVE );
        }
    }
}

void ReluConstraint::addCostFunctionComponent( Map<unsigned, double> &cost )
{
    double bValue = _gurobi->getValue( _b );

    PLConstraint_LOG( Stringf( "Relu constraint. b: %u, bValue: %.2lf. blb: %.2lf, bub: %.2lf f: %u, fValue: %.2lf. ",
                               _b, bValue, _boundManager->getLowerBound( _b ),
                               _boundManager->getUpperBound( _b ), _f, _gurobi->getValue( _f ) ).ascii() );

    // If the constraint is not active or is fixed, it contributes nothing
    ASSERT( isActive() && !phaseFixed() );

    // This should not be called when the linear part
    // has not been satisfied
    ASSERT( !haveOutOfBoundVariables() );

    // Use a simple heuristic to decide which cost term to add.
    if ( !FloatUtils::isPositive( bValue ) )
    {
        // Case 1: b is non-positive. Cost: f
        addCostFunctionComponent( cost, RELU_PHASE_INACTIVE );
        return;
    }
    else
    {
        // Case 2: b is positive. Cost: f - b
        addCostFunctionComponent( cost, RELU_PHASE_ACTIVE );
        return;
    }
}

void ReluConstraint::addCostFunctionComponentByOutputValue( Map<unsigned, double> &cost, double fValue )
{
    PLConstraint_LOG( Stringf( "Relu constraint. b: %u, bValue: %.2lf. blb: %.2lf, bub: %.2lf f: %u, "
                               "currentfValue: %.2lf, fValue: %.2lf. ",
                               _b, _gurobi->getValue( _b ), _boundManager->getLowerBound( _b ),
                               _boundManager->getUpperBound( _b ), _f, _gurobi->getValue( _f ), fValue ).ascii() );

    // If the constraint is not active or is fixed, it contributes nothing
    if( !isActive() || phaseFixed() )
        return;

    // This should not be called when the linear part
    // has not been satisfied
    ASSERT( !haveOutOfBoundVariables() );

    // Use a simple heuristic to decide which cost term to add.
    if ( !FloatUtils::isPositive( fValue ) )
    {
        // Case 1: b is non-positive. Cost: f
        addCostFunctionComponent( cost, RELU_PHASE_INACTIVE );
        return;
    }
    else
    {
        // Case 2: b is positive. Cost: f - b
        addCostFunctionComponent( cost, RELU_PHASE_ACTIVE );
        return;
    }
}

void ReluConstraint::getReducedHeuristicCost( double &reducedCost,
                                              PhaseStatus &phaseStatusOfReducedCost )
{
    PLConstraint_LOG( Stringf( "Getting reduced heuristic cost for relu constraint. b: %u, "
                               "bValue: %.2lf. blb: %.2lf, bub: %.2lf f: %u, fValue: %.2lf. ",
                               _b, _gurobi->getValue( _b ), _boundManager->getLowerBound( _b ),
                               _boundManager->getUpperBound( _b ), _f, _gurobi->getValue( _f ) ).ascii() );

    ASSERT( _phaseOfHeuristicCost != PHASE_NOT_FIXED );
    double bValue = _gurobi->getValue( _b );

    // Current heuristic cost is f - b, see if the heuristic cost f is better
    if ( _phaseOfHeuristicCost == RELU_PHASE_ACTIVE )
    {
        reducedCost = -bValue;
        phaseStatusOfReducedCost = RELU_PHASE_INACTIVE;
    }
    else
    {
        ASSERT( _phaseOfHeuristicCost == RELU_PHASE_INACTIVE );
        reducedCost = bValue;
        phaseStatusOfReducedCost = RELU_PHASE_ACTIVE;
        return;
    }
}

void ReluConstraint::removeCostFunctionComponent( Map<unsigned, double> &cost )
{
    ASSERT( _phaseOfHeuristicCost != PHASE_NOT_FIXED );
    if ( _phaseOfHeuristicCost == RELU_PHASE_ACTIVE )
    {
        if ( !cost.exists( _f ) )
            cost[_f] = 0;
        if ( !cost.exists( _b ) )
            cost[_b] = 0;
        cost[_b] += 1;
        cost[_f] -= 1;
    }
    else
    {
        ASSERT( _phaseOfHeuristicCost == RELU_PHASE_INACTIVE );
        if ( !cost.exists( _f ) )
            cost[_f] = 0;
        cost[_f] -= 1;
    }
    if ( cost.exists( _f ) && cost[_f] == 0 )
        cost.erase( _f );
    if ( cost.exists( _b ) && cost[_b] == 0 )
        cost.erase( _b );
    _phaseOfHeuristicCost = PHASE_NOT_FIXED;
}

Vector<PhaseStatus> ReluConstraint::getAlternativeHeuristicPhaseStatus()
{
    Vector<PhaseStatus> alternatives;
    if ( _phaseOfHeuristicCost == PHASE_NOT_FIXED )
    {
        alternatives.append( RELU_PHASE_INACTIVE );
        alternatives.append( RELU_PHASE_ACTIVE );
    }
    else if ( _phaseOfHeuristicCost == RELU_PHASE_ACTIVE )
        alternatives.append( RELU_PHASE_INACTIVE );
    else
        alternatives.append( RELU_PHASE_ACTIVE );
    return alternatives;
}

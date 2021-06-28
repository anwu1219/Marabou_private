/*********************                                                        */
/*! \file MILPEncoder.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Andrew Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** \brief [[ Add one-line brief description here ]]
 **
 ** [[ Add lengthier description here ]]
 **/

#include "FloatUtils.h"
#include "MILPEncoder.h"
#include "Options.h"

MILPEncoder::MILPEncoder( BoundManager &boundManager, bool relax )
    : _boundManager( boundManager )
    , _relax( relax )
{}

void MILPEncoder::encodeInputQuery( LPSolver &gurobi,
                                    const InputQuery &inputQuery )
{
    gurobi.reset();
    // Add variables
    for ( unsigned var = 0; var < inputQuery.getNumberOfVariables(); var++ )
    {
        double lb = _boundManager.getLowerBound( var );
        double ub = _boundManager.getUpperBound( var );
        String varName = Stringf( "x%u", var );
        gurobi.addVariable( varName, lb, ub );
        _variableToVariableName[var] = varName;
    }

    // Add equations
    for ( const auto &equation : inputQuery.getEquations() )
    {
        encodeEquation( gurobi, equation );
    }

    // Add Piecewise-linear Constraints
    for ( const auto &plConstraint : inputQuery.getPiecewiseLinearConstraints() )
    {
        switch ( plConstraint->getType() )
        {
        case PiecewiseLinearFunctionType::RELU:
            encodeReLUConstraint( gurobi, (ReluConstraint *)plConstraint );
            break;
        case PiecewiseLinearFunctionType::MAX:
            encodeMaxConstraint( gurobi, (MaxConstraint *)plConstraint );
            break;
        case PiecewiseLinearFunctionType::DISJUNCTION:
            break;

        default:
            throw MarabouError( MarabouError::UNSUPPORTED_PIECEWISE_LINEAR_CONSTRAINT,
                                "LPSolver::encodeInputQuery: "
                                "Only ReLU and Max are supported\n" );
        }
    }
    gurobi.updateModel();
}

String MILPEncoder::getVariableNameFromVariable( unsigned variable )
{
    if ( !_variableToVariableName.exists( variable ) )
        throw CommonError( CommonError::KEY_DOESNT_EXIST_IN_MAP );
    return _variableToVariableName[variable];
}

void MILPEncoder::encodeEquation( LPSolver &gurobi, const Equation &equation )
{
    List<LPSolver::Term> terms;
    double scalar = equation._scalar;
    for ( const auto &term : equation._addends )
        terms.append( LPSolver::Term
                      ( term._coefficient,
                        Stringf( "x%u", term._variable ) ) );
    switch ( equation._type )
    {
    case Equation::EQ:
        gurobi.addEqConstraint( terms, scalar );
        break;
    case Equation::LE:
        gurobi.addLeqConstraint( terms, scalar );
        break;
    case Equation::GE:
        gurobi.addGeqConstraint( terms, scalar );
        break;
    default:
        break;
    }
}

void MILPEncoder::encodeReLUConstraint( LPSolver &gurobi, ReluConstraint *relu )
{

    if ( !relu->isActive() || relu->phaseFixed() )
    {
        ASSERT( relu->auxVariableInUse() );
        ASSERT( ( FloatUtils::gte( _boundManager.getLowerBound( relu->getB() ),  0 ) &&
                  FloatUtils::lte( _boundManager.getLowerBound( relu->getAux() ), 0 ) )
                ||
                ( FloatUtils::lte( _boundManager.getUpperBound( relu->getB() ), 0 ) &&
                  FloatUtils::lte( _boundManager.getUpperBound( relu->getF() ), 0 ) ) );
        return;
    }

    if ( !_relax )
    {
        gurobi.addVariable( Stringf( "a%u", _binVarIndex ),
                            0,
                            1,
                            LPSolver::BINARY );

        unsigned sourceVariable = relu->getB();
        unsigned targetVariable = relu->getF();
        double sourceLb = _boundManager.getLowerBound( sourceVariable );
        double sourceUb = _boundManager.getUpperBound( sourceVariable );

        List<LPSolver::Term> terms;
        terms.append( LPSolver::Term( 1, Stringf( "x%u", targetVariable ) ) );
        terms.append( LPSolver::Term( -1, Stringf( "x%u", sourceVariable ) ) );
        terms.append( LPSolver::Term( -sourceLb, Stringf( "a%u", _binVarIndex ) ) );
        gurobi.addLeqConstraint( terms, -sourceLb );

        terms.clear();
        terms.append( LPSolver::Term( 1, Stringf( "x%u", targetVariable ) ) );
        terms.append( LPSolver::Term( -sourceUb, Stringf( "a%u", _binVarIndex++ ) ) );
        gurobi.addLeqConstraint( terms, 0 );
    }
    else
    {
        if ( Options::get()->getString( Options::LP_ENCODING ) == "lp" )
        {
            unsigned sourceVariable = relu->getB();
            unsigned targetVariable = relu->getF();
            double sourceLb = _boundManager.getLowerBound( sourceVariable );
            double sourceUb = _boundManager.getUpperBound( sourceVariable );

            List<LPSolver::Term> terms;
            terms.append( LPSolver::Term( 1, Stringf( "x%u", targetVariable ) ) );
            terms.append( LPSolver::Term( -sourceUb / ( sourceUb - sourceLb ), Stringf( "x%u", sourceVariable ) ) );
            gurobi.addLeqConstraint( terms, ( -sourceUb * sourceLb ) / ( sourceUb - sourceLb ) );
        }
        else
        {
            gurobi.addVariable( Stringf( "a%u", _binVarIndex ), 0, 1 );

            unsigned sourceVariable = relu->getB();
            unsigned targetVariable = relu->getF();
            double sourceLb = _boundManager.getLowerBound( sourceVariable );
            double sourceUb = _boundManager.getUpperBound( sourceVariable );

            List<LPSolver::Term> terms;
            terms.append( LPSolver::Term( 1, Stringf( "x%u", targetVariable ) ) );
            terms.append( LPSolver::Term( -1, Stringf( "x%u", sourceVariable ) ) );
            terms.append( LPSolver::Term( -sourceLb, Stringf( "a%u", _binVarIndex ) ) );
            gurobi.addLeqConstraint( terms, -sourceLb );

            terms.clear();
            terms.append( LPSolver::Term( 1, Stringf( "x%u", targetVariable ) ) );
            terms.append( LPSolver::Term( -sourceUb, Stringf( "a%u", _binVarIndex++ ) ) );
            gurobi.addLeqConstraint( terms, 0 );
        }
    }
}

void MILPEncoder::encodeMaxConstraint( LPSolver &gurobi, MaxConstraint *max )
{
    if ( !max->isActive() )
        return;

    if ( _relax )
        return;

    // y = max(x_1, x_2, ... , x_m)
    unsigned y = max->getF();

    // xs = [x_1, x_2, ... , x_m]
    List<unsigned> xs = max->getElements();

    // upper bounds of each x_i
    using qtype = std::pair<double, unsigned>;
    auto cmp = []( qtype l, qtype r) { return l.first <= r.first; };
    std::priority_queue<qtype, std::vector<qtype>, decltype( cmp )> ubq( cmp );

    // terms for Gurobi
    List<LPSolver::Term> terms;

    for ( const auto &x : xs ) 
    {
        // add binary variable
        // Nameing rule is `a{_binVarIndex}_{x}` to clarify
        // which x binary variable is for. 
        gurobi.addVariable( Stringf( "a%u_%u", _binVarIndex, x ),
                            0,
                            1,
                            LPSolver::BINARY );

        terms.append( LPSolver::Term( 1, Stringf( "a%u_%u", _binVarIndex, x ) ) );
        ubq.push( { _boundManager.getUpperBound( x ), x } );
    }

    // add constraint: a_1 + a_2 + ... + a_m = 1
    gurobi.addEqConstraint( terms, 1 );

    // extract the pairs of the maximum upper bound and the second.
    auto ubMax1 = ubq.top();
    ubq.pop();
    auto ubMax2 = ubq.top();

    terms.clear();

    double umax = 0;
    for ( const auto &x : xs ) 
    {
        // add constraint: y <= x_i + (1 - a_i) * (umax - l)
        if ( ubMax1.second != x )
            umax = ubMax1.first;
        else
            umax = ubMax2.first;
        terms.append( LPSolver::Term( 1, Stringf( "x%u", y ) ) );
        terms.append( LPSolver::Term( -1, Stringf( "x%u", x ) ) );
        terms.append( LPSolver::Term( umax - _boundManager.getLowerBound( x ), Stringf( "a%u_%u", _binVarIndex, x ) ) );
        gurobi.addLeqConstraint( terms, umax - _boundManager.getLowerBound( x ) );

        terms.clear();
    }
    _binVarIndex++;
}

void MILPEncoder::encodeDisjConstraint( LPSolver &gurobi, DisjunctionConstraint *disj )
{
    if ( !disj->isActive() )
        return;

    if ( _relax )
        return;

    List<String> cases;
    for ( const auto &split : disj->getCaseSplits() )
    {
        gurobi.addVariable( Stringf( "a%u", _binVarIndex ),
                            0,
                            1,
                            LPSolver::BINARY );

        for ( const auto &bound : split.getBoundTightenings() )
            gurobi.addGenConstrIndicator( Stringf( "a%u", _binVarIndex ), 1, bound );

        cases.append(Stringf( "a%u", _binVarIndex ));
        _binVarIndex++;
    }
    List<LPSolver::Term> terms;
    for ( const auto &varName : cases )
        terms.append( LPSolver::Term( 1, varName ) );
    gurobi.addEqConstraint( terms, 1 );
}

/*********************                                                        */
/*! \file GurobiWrapper.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Haoze Andrew Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#ifdef ENABLE_GUROBI

#include "Debug.h"
#include "FloatUtils.h"
#include "GlobalConfiguration.h"
#include "GurobiWrapper.h"
#include "MStringf.h"
#include "Options.h"
#include "gurobi_c.h"

#include <iostream>

using namespace std;

GurobiWrapper::GurobiWrapper()
    : _environment( NULL )
    , _model( NULL )
    , _timeoutInSeconds( Options::get()->getFloat( Options::MILP_SOLVER_TIMEOUT ) )
{
    _environment = new GRBEnv;
    resetModel();
}

GurobiWrapper::~GurobiWrapper()
{
    freeMemoryIfNeeded();
}

void GurobiWrapper::freeModelIfNeeded()
{
    for ( auto &entry : _nameToVariable )
    {
        delete entry.second;
        entry.second = NULL;
    }
    _nameToVariable.clear();

    if ( _model )
    {
        delete _model;
        _model = NULL;
    }
}

void GurobiWrapper::freeMemoryIfNeeded()
{
    freeModelIfNeeded();

    if ( _environment )
    {
        delete _environment;
        _environment = NULL;
    }
}

void GurobiWrapper::resetModel()
{
    freeModelIfNeeded();
    _model = new GRBModel( *_environment );

    // Suppress printing
    _model->getEnv().set( GRB_IntParam_OutputFlag, 0 );

    // Thread number
    _model->getEnv().set( GRB_IntParam_Threads,
                          GlobalConfiguration::GUROBI_NUMBER_OF_THREADS );

    // Thread precision
    _model->getEnv().set( GRB_DoubleParam_OptimalityTol,
                          1e-5 );
    _model->getEnv().set( GRB_DoubleParam_FeasibilityTol,
                          1e-4 );
    _model->getEnv().set( GRB_DoubleParam_IntFeasTol,
                          1e-4 );

    // Timeout
    //setTimeLimit( _timeoutInSeconds );
}

void GurobiWrapper::setVerbosity( unsigned verbosity )
{
    if ( verbosity > 0 )
        _model->getEnv().set( GRB_IntParam_OutputFlag, 1 );
    else
        _model->getEnv().set( GRB_IntParam_OutputFlag, 0 );
}

void GurobiWrapper::reset()
{
    _model->reset();
}

void GurobiWrapper::addVariable( String name, double lb, double ub, VariableType type )
{
    ASSERT( !_nameToVariable.exists( name ) );

    char variableType = GRB_CONTINUOUS;
    switch ( type )
    {
    case CONTINUOUS:
        variableType = GRB_CONTINUOUS;
        break;

    case BINARY:
        variableType = GRB_BINARY;
        break;

    default:
        break;
    }

    try
    {
        GRBVar *newVar = new GRBVar;
        double objectiveValue = 0;
        *newVar = _model->addVar( lb,
                                  ub,
                                  objectiveValue,
                                  variableType,
                                  name.ascii() );

        _nameToVariable[name] = newVar;
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

void GurobiWrapper::setLowerBound( String name, double lb )
{
    GRBVar var = _model->getVarByName( name.ascii() );
    var.set( GRB_DoubleAttr_LB, lb );
}

void GurobiWrapper::setUpperBound( String name, double ub )
{
    GRBVar var = _model->getVarByName( name.ascii() );
    var.set( GRB_DoubleAttr_UB, ub );
}

void GurobiWrapper::setCutoff( double cutoff )
{
    _model->set( GRB_DoubleParam_Cutoff, cutoff );
}

void GurobiWrapper::addLeqConstraint( const List<Term> &terms, double scalar, String name )
{
    addConstraint( terms, scalar, GRB_LESS_EQUAL, name );
}

void GurobiWrapper::addGeqConstraint( const List<Term> &terms, double scalar, String name )
{
    addConstraint( terms, scalar, GRB_GREATER_EQUAL, name );
}

void GurobiWrapper::addEqConstraint( const List<Term> &terms, double scalar, String name )
{
    addConstraint( terms, scalar, GRB_EQUAL, name );
}

void GurobiWrapper::addConstraint( const List<Term> &terms, double scalar, char sense, String name )
{
    try
    {
        log( Stringf( "Adding constraint (name: %s).", name.ascii() ).ascii() );

        GRBLinExpr constraint;

        for ( const auto &term : terms )
        {
            ASSERT( _nameToVariable.exists( term._variable ) );
            constraint += GRBLinExpr( *_nameToVariable[term._variable], term._coefficient );
        }

        _model->addConstr( constraint, sense, scalar, name.ascii() );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

void GurobiWrapper::removeConstraint( String constraintName )
{
    try
    {
        log( Stringf( "Removing constraint (name: %s).", constraintName.ascii() ).ascii() );

        _model->remove( _model->getConstrByName( constraintName.ascii() ) );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}


void GurobiWrapper::setCost( const List<Term> &terms )
{
    try
    {
        GRBLinExpr cost;

        for ( const auto &term : terms )
        {
            ASSERT( _nameToVariable.exists( term._variable ) );
            cost += GRBLinExpr( *_nameToVariable[term._variable], term._coefficient );
        }

        _model->setObjective( cost, GRB_MINIMIZE );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

void GurobiWrapper::setObjective( const List<Term> &terms )
{
    try
    {
        GRBLinExpr cost;

        for ( const auto &term : terms )
        {
            ASSERT( _nameToVariable.exists( term._variable ) );
            cost += GRBLinExpr( *_nameToVariable[term._variable], term._coefficient );
        }

        _model->setObjective( cost, GRB_MAXIMIZE );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

void GurobiWrapper::setTimeLimit( double seconds )
{
    _model->set( GRB_DoubleParam_TimeLimit, seconds );
}

void GurobiWrapper::solve()
{
    try
    {
        /*
        DEBUG({
                if ( Options::get()->getInt( Options::VERBOSITY ) == 2 )
                {
                    _model->update();
                    printf( "Number of constraints: %u\n", _model->get( GRB_IntAttr_NumConstrs ) );
                    printf( "Number of variables: %u\n", _model->get( GRB_IntAttr_NumVars ) );
                    printf( "Number of non-zeros: %u\n", _model->get( GRB_IntAttr_NumNZs ) );
                }
            });
        */
        _model->optimize();
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

double GurobiWrapper::getObjective()
{
    try
    {
        return _model->get( GRB_DoubleAttr_ObjVal );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}


void GurobiWrapper::extractSolution( Map<String, double> &values, double &costOrObjective )
{
    try
    {
        values.clear();

        for ( const auto &variable : _nameToVariable )
            values[variable.first] = variable.second->get( GRB_DoubleAttr_X );

        costOrObjective = _model->get( GRB_DoubleAttr_ObjVal );
    }
    catch ( GRBException e )
    {
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

double GurobiWrapper::getObjectiveBound()
{
    try
    {
        return _model->get( GRB_DoubleAttr_ObjBound );
    }
    catch ( GRBException e )
    {
        log( "Failed to get objective bound from Gurobi." );
        if ( e.getErrorCode() == GRB_ERROR_DATA_NOT_AVAILABLE )
        {
            // From https://www.gurobi.com/documentation/9.0/refman/py_model_setattr.html
            // due to our lazy update approach, the change won't actually take effect until you
            // update the model (using Model.update), optimize the model (using Model.optimize),
            // or write the model to disk (using Model.write).
            _model->update();

            if ( _model->get( GRB_IntAttr_ModelSense ) == 1 )
                // case minimize
                return FloatUtils::negativeInfinity();
            else
                // case maximize
                return FloatUtils::infinity();
        }
        throw CommonError( CommonError::GUROBI_EXCEPTION,
                           Stringf( "Gurobi exception. Gurobi Code: %u, message: %s\n",
                                    e.getErrorCode(),
                                    e.getMessage().c_str() ).ascii() );
    }
}

void GurobiWrapper::dumpModel( String name )
{
    _model->write( name.ascii() );
}

unsigned GurobiWrapper::getNumberOfSimplexIterations()
{
    return _model->get( GRB_DoubleAttr_IterCount );
}

unsigned GurobiWrapper::getNumberOfNodes()
{
    return _model->get( GRB_DoubleAttr_NodeCount );
}

void GurobiWrapper::log( const String &message )
{
    if ( GlobalConfiguration::GUROBI_LOGGING )
        printf( "GurobiWrapper: %s\n", message.ascii() );
}

#endif // ENABLE_GUROBI

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

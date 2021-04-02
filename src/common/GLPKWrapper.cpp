/*********************                                                        */
/*! \file GLPKWrapper.cpp
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

#ifdef ENABLE_GLPK

#include "Debug.h"
#include "FloatUtils.h"
#include "GlobalConfiguration.h"
#include "GLPKWrapper.h"
#include "MStringf.h"
#include "Options.h"

#include <iostream>

using namespace std;

GLPKWrapper::GLPKWrapper()
{
    _lp = glp_create_prob();
    glp_set_prob_name( _lp, "prob" );
}

GLPKWrapper::~GLPKWrapper()
{
}

void GLPKWrapper::freeModelIfNeeded()
{
}

void GLPKWrapper::freeMemoryIfNeeded()
{
    glp_delete_prob( _lp );
}

void GLPKWrapper::resetModel()
{
}

void GLPKWrapper::setVerbosity( unsigned )
{
}

void GLPKWrapper::reset()
{
}

void GLPKWrapper::addVariable( String, double, double, VariableType )
{
}

void GLPKWrapper::setLowerBound( String, double )
{
}

void GLPKWrapper::setUpperBound( String, double )
{
}

double GLPKWrapper::getLowerBound( unsigned )
{
    return 0;
}

double GLPKWrapper::getUpperBound( unsigned )
{
    return 0;
}

void GLPKWrapper::setCutoff( double )
{
}

void GLPKWrapper::addLeqConstraint( const List<Term> &, double , String )
{
}

void GLPKWrapper::addGeqConstraint( const List<Term> &, double, String )
{
}

void GLPKWrapper::addEqConstraint( const List<Term> &, double, String )
{
}

void GLPKWrapper::removeConstraint( String /*constraintName*/ )
{
}

void GLPKWrapper::setCost( const List<Term> &/*terms*/ )
{
}

void GLPKWrapper::setObjective( const List<Term> &/*terms*/ )
{
}

void GLPKWrapper::setTimeLimit( double /*seconds*/ )
{
}

// Returns true iff an optimal solution has been found
bool GLPKWrapper::optimal()
{
    return glp_get_prim_stat( _lp ) == GLP_OPT;
}

// Returns true iff the cutoff value was used
bool GLPKWrapper::cutoffOccurred()
{
    return _retValue == GLP_EOBJLL || _retValue == GLP_EOBJUL;
}

// Returns true iff the instance is infeasible
bool GLPKWrapper::infeasible()
{
    return glp_get_prim_stat( _lp ) == GLP_NOFEAS;
}

// Returns true iff the instance timed out
bool GLPKWrapper::timeout()
{
    return _retValue == GLP_ETMLIM;
}

// Returns true iff a feasible solution has been found
bool GLPKWrapper::haveFeasibleSolution()
{
    return glp_get_prim_stat( _lp ) == GLP_FEAS;
}

void GLPKWrapper::solve()
{
    // The GLPK control parameters
    glp_smcp controlParameters;

    glp_init_smcp( &controlParameters );
    controlParameters.msg_lev = GLP_MSG_OFF;
    controlParameters.meth = GLP_PRIMAL;
    controlParameters.pricing = GLP_PT_PSE; // Steepest Edge
    controlParameters.r_test = GLP_RT_HAR; // Harris' two-pass
    controlParameters.presolve = 1;

    _retValue = glp_simplex( _lp, &controlParameters );
}

double getValue( unsigned )
{
    return 0;
}

double GLPKWrapper::getObjective()
{
    return glp_get_obj_val( _lp );
}


void GLPKWrapper::extractSolution( Map<String, double> &/*values*/, double &/*costOrObjective*/ )
{
}

double GLPKWrapper::getObjectiveBound()
{
    return glp_get_obj_val( _lp );
}

void GLPKWrapper::dumpModel( String /*name*/ )
{
    // GLPK doesn't support this.
}

unsigned GLPKWrapper::getNumberOfSimplexIterations()
{
    // GLPK doesn't support this.
    return 0;
}

unsigned GLPKWrapper::getNumberOfNodes()
{
    // GLPK doesn't support this.
    return 0;
}

void GLPKWrapper::log( const String &message )
{
    if ( GlobalConfiguration::GUROBI_LOGGING )
        printf( "GLPKWrapper: %s\n", message.ascii() );
}

#endif // ENABLE_GLPK

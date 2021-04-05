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
    : _lp( NULL )
    , _controlParameters( NULL )
    , _retValue( -1 )
    , _numRows( 0 )
    , _numCols( 0 )
{
    resetModel();
}

GLPKWrapper::~GLPKWrapper()
{
    freeMemoryIfNeeded();
}

void GLPKWrapper::freeMemoryIfNeeded()
{
    if ( _lp )
    {
        glp_delete_prob( _lp );
        _lp = NULL;
    }
    if ( _controlParameters )
    {
        delete _controlParameters;
        _controlParameters = NULL;
    }
}

void GLPKWrapper::resetModel()
{
    freeMemoryIfNeeded();
    _lp = glp_create_prob();
    glp_set_prob_name( _lp, "prob" );
    glp_set_obj_dir( _lp, GLP_MIN );
    glp_set_obj_coef( _lp, 0, 0 );

    // The GLPK control parameters
    _controlParameters = new glp_smcp();
    glp_init_smcp( _controlParameters );
    _controlParameters->msg_lev = GLP_MSG_OFF;
    _controlParameters->meth = GLP_PRIMAL;
    _controlParameters->pricing = GLP_PT_PSE; // Steepest Edge
    _controlParameters->r_test = GLP_RT_HAR; // Harris' two-pass
    _controlParameters->presolve = 1;
    _controlParameters->tol_bnd =
        GlobalConfiguration::DEFAULT_EPSILON_FOR_COMPARISONS;
    _controlParameters->tol_dj =
    GlobalConfiguration::DEFAULT_EPSILON_FOR_COMPARISONS;


    _retValue = -1;
    _numRows = 0;
    _numCols = 0;
}

void GLPKWrapper::setVerbosity( unsigned verbosity )
{
    _controlParameters->msg_lev = verbosity;
}

void GLPKWrapper::reset()
{
    resetModel();
}

void GLPKWrapper::addVariable( String name, double lb, double ub, VariableType )
{
    ASSERT( !_nameToVariable.exists( name ) );
    _retValue = -1;
    ++_numCols;
    int col = glp_add_cols( _lp, 1 );
    _nameToVariable[name] = col;
    glp_set_col_name( _lp, col, name.ascii() );
    ASSERT( FloatUtils::lte( lb, ub ) );
    if ( FloatUtils::areEqual( lb, ub ) )
        glp_set_col_bnds( _lp, col, GLP_FX, lb, lb );
    else
        glp_set_col_bnds( _lp, col, GLP_DB, lb, ub );
}

void GLPKWrapper::setLowerBound( String name, double lb )
{
    _retValue = -1;
    double ub = glp_get_col_ub( _lp, _nameToVariable[name] );
    if ( FloatUtils::areEqual( lb, ub ) )
        glp_set_col_bnds( _lp, _nameToVariable[name], GLP_FX, lb, lb );
    else
        glp_set_col_bnds( _lp, _nameToVariable[name], GLP_DB, lb, ub );
}

void GLPKWrapper::setUpperBound( String name, double ub )
{
    _retValue = -1;
    double lb = glp_get_col_lb( _lp, _nameToVariable[name] );
    if ( FloatUtils::areEqual( lb, ub ) )
        glp_set_col_bnds( _lp, _nameToVariable[name], GLP_FX, lb, lb );
    else
        glp_set_col_bnds( _lp, _nameToVariable[name], GLP_DB, lb, ub );
}

double GLPKWrapper::getLowerBound( unsigned var )
{
    return glp_get_col_lb( _lp, _nameToVariable[Stringf("x%u", var ).ascii()] );
}

double GLPKWrapper::getUpperBound( unsigned var )
{
    return glp_get_col_ub( _lp, _nameToVariable[Stringf("x%u", var ).ascii()] );
}

void GLPKWrapper::setCutoff( double cutoff )
{
    _retValue = -1;
    int dir = glp_get_obj_dir( _lp );
    if ( dir == GLP_MAX )
        _controlParameters->obj_ll = cutoff;
    else
        _controlParameters->obj_ul = cutoff;
}

void GLPKWrapper::addLeqConstraint( const List<Term> & terms, double scalar, String )
{
    _retValue = -1;
    ++_numRows;
    int row = glp_add_rows( _lp, 1 );
    glp_set_row_name( _lp, row, Stringf( "c%u", row ).ascii() );
    addConstraint( terms, 0, scalar, GLP_UP, row );
}

void GLPKWrapper::addGeqConstraint( const List<Term> &terms, double scalar, String )
{
    _retValue = -1;
    ++_numRows;
    int row = glp_add_rows( _lp, 1 );
    glp_set_row_name( _lp, row, Stringf( "c%u", row ).ascii() );
    addConstraint( terms, scalar, 0, GLP_LO, row );
}

void GLPKWrapper::addEqConstraint( const List<Term> &terms, double scalar, String )
{
    _retValue = -1;
    ++_numRows;
    int row = glp_add_rows( _lp, 1 );
    glp_set_row_name( _lp, row, Stringf( "c%u", row ).ascii() );
    addConstraint( terms, scalar, scalar, GLP_FX, row );
}

void GLPKWrapper::addConstraint( const List<Term> &terms, double lb, double ub,
                                 int sense, int row )
{
    glp_set_row_bnds( _lp, row, sense, lb, ub );
    int size = terms.size();
    int *indices = new int[size + 1];
    double *weights = new double[size + 1];

    unsigned counter = 1;
    for ( const auto &term : terms )
    {
        unsigned col = _nameToVariable[term._variable];
        double coeff = term._coefficient;
        indices[counter] = col;
        weights[counter] = coeff;
        ++counter;
    }

    glp_set_mat_row( _lp, row, size, indices, weights );

    delete[] indices;
    delete[] weights;
}

void GLPKWrapper::removeConstraint( String /*constraintName*/ )
{
}

void GLPKWrapper::setCost( const List<Term> &terms )
{
    _retValue = -1;
    glp_set_obj_dir( _lp, GLP_MIN );
    for ( const auto &term : terms )
    {
        unsigned col = _nameToVariable[term._variable];
        double coeff = term._coefficient;
        glp_set_obj_coef( _lp, col, coeff );
    }
}

void GLPKWrapper::setObjective( const List<Term> &terms )
{
    _retValue = -1;
    glp_set_obj_dir( _lp, GLP_MAX );
    for ( const auto &term : terms )
    {
        unsigned col = _nameToVariable[term._variable];
        double coeff = term._coefficient;
        glp_set_obj_coef( _lp, col, coeff );
    }
}

void GLPKWrapper::setTimeLimit( double seconds )
{
    _retValue = -1;
    _controlParameters->tm_lim = seconds * 1000;
}

// Returns true iff an optimal solution has been found
bool GLPKWrapper::optimal()
{
    return glp_get_status( _lp ) == GLP_OPT;
}

// Returns true iff the cutoff value was used
bool GLPKWrapper::cutoffOccurred()
{
    return _retValue == GLP_EOBJLL || _retValue == GLP_EOBJUL;
}

// Returns true iff the instance is infeasible
bool GLPKWrapper::infeasible()
{
    return _retValue == GLP_EBOUND ||
        _retValue == GLP_ENOPFS ||  glp_get_prim_stat( _lp ) == GLP_NOFEAS;
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
    log( Stringf( "Number of rows: %u", glp_get_num_rows( _lp ) ) );
    log( Stringf( "Number of columns: %u", glp_get_num_cols( _lp ) ) );
    _retValue = glp_simplex( _lp, _controlParameters );
    log( Stringf( "Return code: %u, status: %u", _retValue, glp_get_status( _lp ) ) );
}

double GLPKWrapper::getValue( unsigned variable )
{
    return glp_get_col_prim( _lp, _nameToVariable[Stringf("x%u", variable)] );
}

double GLPKWrapper::getObjective()
{
    return glp_get_obj_val( _lp );
}


void GLPKWrapper::extractSolution( Map<String, double> &values, double &costOrObjective )
{
    values.clear();

    for ( const auto &variable : _nameToVariable )
    {
        double value = glp_get_col_prim( _lp, variable.second );
        DEBUG({
                if ( Options::get()->getInt( Options::VERBOSITY ) > 1 )
                    log( Stringf( "%s (variable %d) is %f ", variable.first.ascii(),
                                  variable.second, value ) );
            });
        values[variable.first] = value;
    }

    costOrObjective = getObjective();
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

void GLPKWrapper::updateModel()
{
}

void GLPKWrapper::log( const String &message )
{
    if ( GlobalConfiguration::GUROBI_LOGGING )
        printf( "GLPKWrapper: %s\n", message.ascii() );
}

#endif // ENABLE_GLPK

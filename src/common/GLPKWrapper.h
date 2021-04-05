/*********************                                                        */
/*! \file GLPKWrapper.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Haoze Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#ifndef __GLPKWrapper_h__
#define __GLPKWrapper_h__

#include "LPSolver.h"
#include "MStringf.h"
#include "Map.h"

#include "glpk.h"

class GLPKWrapper : public LPSolver
{
public:
    GLPKWrapper();
    ~GLPKWrapper();

    // Add a new variabel to the model
    void addVariable( String name, double lb, double ub, VariableType type = CONTINUOUS );

    double getLowerBound( unsigned var );

    double getUpperBound( unsigned var );

    // Set the lower or upper bound for an existing variable
    void setLowerBound( String, double lb );
    void setUpperBound( String, double ub );

    // Add a new LEQ constraint, e.g. 3x + 4y <= -5
    void addLeqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    // Add a new GEQ constraint, e.g. 3x + 4y >= -5
    void addGeqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    // Add a new EQ constraint, e.g. 3x + 4y = -5
    void addEqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    void removeConstraint( String constraintName );

    // A cost function to minimize, or an objective function to maximize
    void setCost( const List<LPSolver::Term> &terms );
    void setObjective( const List<LPSolver::Term> &terms );

    // Set a cutoff value for the objective function. For example, if
    // maximizing x with cutoff value 0, Gurobi will return the
    // optimal value if greater than 0, and 0 if the optimal value is
    // less than 0.
    void setCutoff( double cutoff );

    // Returns true iff an optimal solution has been found
    bool optimal();

    // Returns true iff the cutoff value was used
    bool cutoffOccurred();

    // Returns true iff the instance is infeasible
    bool infeasible();

    // Returns true iff the instance timed out
    bool timeout();

    // Returns true iff a feasible solution has been found
    bool haveFeasibleSolution();

    // Specify a time limit, in seconds
    void setTimeLimit( double seconds );

    // Solve and extract the solution, or the best known bound on the
    // objective function
    void solve();
    double getValue( unsigned variable );

    double getObjective();
    void extractSolution( Map<String, double> &values, double &costOrObjective );
    double getObjectiveBound();

    // Reset the underlying model
    void reset();

    // Clear the underlying model and create a fresh model
    void resetModel();

    void setVerbosity( unsigned verbosity );

    // Dump the model to a file. Note that the suffix of the file is
    // used by Gurobi to determine the format. Using ".lp" is a good
    // default
    void dumpModel( String name );

    unsigned getNumberOfSimplexIterations();

    unsigned getNumberOfNodes();

    void updateModel();

private:
    glp_prob *_lp;
    glp_smcp *_controlParameters;
    int _retValue;
    unsigned _numRows;
    unsigned _numCols;

    Map<String, unsigned> _nameToVariable;

    void freeMemoryIfNeeded();

    void addConstraint( const List<LPSolver::Term> &terms, double lb, double ub,
                        int sense );

    static unsigned getVariable( String name )
    {
        auto subTokens = name.tokenize( "x" );
        unsigned justIndex = atoi( subTokens.rbegin()->ascii() );
        return justIndex;
    }

    static void log( const String &message );
};

#endif // __GLPKWrapper_h__

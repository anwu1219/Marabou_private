/*********************                                                        */
/*! \file GurobiWrapper.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#ifndef __GurobiWrapper_h__
#define __GurobiWrapper_h__

#include "LPSolver.h"
#include "MStringf.h"
#include "Map.h"

#include "gurobi_c++.h"

class GurobiWrapper : public LPSolver
{
public:
    GurobiWrapper();
    ~GurobiWrapper();

    // Add a new variabel to the model
    void addVariable( String name, double lb, double ub, VariableType type = CONTINUOUS );

    inline double getLowerBound( unsigned var )
    {
        return _model->getVarByName( Stringf("x%u", var ).ascii() ).get( GRB_DoubleAttr_LB );
    }

    inline double getUpperBound( unsigned var )
    {
        return _model->getVarByName( Stringf("x%u", var ).ascii() ).get( GRB_DoubleAttr_UB );
    }

    // Set the lower or upper bound for an existing variable
    void setLowerBound( String, double lb );
    void setUpperBound( String, double ub );

    // Add a new LEQ constraint, e.g. 3x + 4y <= -5
    void addLeqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    // Add a new GEQ constraint, e.g. 3x + 4y >= -5
    void addGeqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    // Add a new EQ constraint, e.g. 3x + 4y = -5
    void addEqConstraint( const List<LPSolver::Term> &terms, double scalar, String name="" );

    void addGenConstrIndicator( String varName, int binval, Tightening tightening );

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
    inline bool optimal()
    {
        return _model->get( GRB_IntAttr_Status ) == GRB_OPTIMAL;
    }

    // Returns true iff the cutoff value was used
    inline bool cutoffOccurred()
    {
        return _model->get( GRB_IntAttr_Status ) == GRB_CUTOFF;
    }

    // Returns true iff the instance is infeasible
    inline bool infeasible()
    {
        return _model->get( GRB_IntAttr_Status ) == GRB_INFEASIBLE;
    }

    // Returns true iff the instance timed out
    inline bool timeout()
    {
        return _model->get( GRB_IntAttr_Status ) == GRB_TIME_LIMIT;
    }

    // Returns true iff a feasible solution has been found
    inline bool haveFeasibleSolution()
    {
        return _model->get( GRB_IntAttr_SolCount ) > 0;
    }

    // Specify a time limit, in seconds
    void setTimeLimit( double seconds );

    // Solve and extract the solution, or the best known bound on the
    // objective function
    void solve( unsigned numThreads = 1 );
    inline double getValue( unsigned variable )
    {
        return _nameToVariable[Stringf("x%u", variable)]->get( GRB_DoubleAttr_X );
    }

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

    void updateModel()
    {
        _model->update();
    }

private:
    GRBEnv *_environment;
    GRBModel *_model;
    Map<String, GRBVar *> _nameToVariable;
    double _timeoutInSeconds;

    void addConstraint( const List<LPSolver::Term> &terms, double scalar, char sense, String name="" );

    void freeModelIfNeeded();
    void freeMemoryIfNeeded();

    static void log( const String &message );
};

#endif // __GurobiWrapper_h__

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

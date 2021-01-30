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

#include "MString.h"
#include "Map.h"

#include "gurobi_c++.h"

class GurobiWrapper
{
public:
    enum VariableType {
        CONTINUOUS = 0,
        BINARY = 1,
    };

    /*
      A term has the form: coefficient * variable
    */
    struct Term
    {
        Term( double coefficient, String variable )
            : _coefficient( coefficient )
            , _variable( variable )
        {
        }

        Term()
            : _coefficient( 0 )
            , _variable( "" )
        {
        }

        double _coefficient;
        String _variable;
    };

    GurobiWrapper();
    ~GurobiWrapper();

    // Add a new variabel to the model
    void addVariable( String name, double lb, double ub, VariableType type = CONTINUOUS );

    double getLowerBound( unsigned );
    double getUpperBound( unsigned );

    // Set the lower or upper bound for an existing variable
    void setLowerBound( String, double lb );
    void setUpperBound( String, double ub );

    // Add a new LEQ constraint, e.g. 3x + 4y <= -5
    void addLeqConstraint( const List<Term> &terms, double scalar );

    // Add a new GEQ constraint, e.g. 3x + 4y >= -5
    void addGeqConstraint( const List<Term> &terms, double scalar );

    // Add a new EQ constraint, e.g. 3x + 4y = -5
    void addEqConstraint( const List<Term> &terms, double scalar );

    // A cost function to minimize, or an objective function to maximize
    void setCost( const List<Term> &terms );
    void setObjective( const List<Term> &terms );

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

    // Dump the model to a file. Note that the suffix of the file is
    // used by Gurobi to determine the format. Using ".lp" is a good
    // default
    void dumpModel( String name );

    unsigned getNumberOfSimplexIterations();

private:
    GRBEnv *_environment;
    GRBModel *_model;
    Map<String, GRBVar *> _nameToVariable;
    double _timeoutInSeconds;

    void addConstraint( const List<Term> &terms, double scalar, char sense );

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

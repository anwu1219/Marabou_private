/*********************                                                        */
/*! \file LPSolver.h
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

#ifndef __LPSolver_h__
#define __LPSolver_h__

#include "MString.h"
#include "Map.h"
#include "Tightening.h"

class LPSolver
{
public:

    enum VariableType
        {
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

    virtual ~LPSolver() {};

    // Add a new variabel to the model
    virtual void addVariable( String name, double lb, double ub, VariableType type = CONTINUOUS ) = 0;

    virtual double getLowerBound( unsigned ) = 0;
    virtual double getUpperBound( unsigned ) = 0;

    // Set the lower or upper bound for an existing variable
    virtual void setLowerBound( String, double lb ) = 0;
    virtual void setUpperBound( String, double ub ) = 0;

    // Add a new LEQ constraint, e.g. 3x + 4y <= -5
    virtual void addLeqConstraint( const List<Term> &terms, double scalar, String name="" ) = 0;

    // Add a new GEQ constraint, e.g. 3x + 4y >= -5
    virtual void addGeqConstraint( const List<Term> &terms, double scalar, String name="" ) = 0;

    // Add a new EQ constraint, e.g. 3x + 4y = -5
    virtual void addEqConstraint( const List<Term> &terms, double scalar, String name="" ) = 0;

    virtual void addGenConstrIndicator( String , int , Tightening  ) {};

    virtual void removeConstraint( String ) = 0;

    // A cost function to minimize, or an objective function to maximize
    virtual void setCost( const List<Term> &terms ) = 0;
    virtual void setObjective( const List<Term> &terms ) = 0;

    // Set a cutoff value for the objective function. For example, if
    // maximizing x with cutoff value 0, Gurobi will return the
    // optimal value if greater than 0, and 0 if the optimal value is
    // less than 0.
    virtual void setCutoff(  double cutoff ) = 0;

    // Returns true iff an optimal solution has been found
    virtual bool optimal() = 0;

    // Returns true iff the cutoff value was used
    virtual bool cutoffOccurred() = 0;

    // Returns true iff the instance is infeasible
    virtual bool infeasible() = 0;

    // Returns true iff the instance timed out
    virtual bool timeout() = 0;

    // Returns true iff a feasible solution has been found
    virtual bool haveFeasibleSolution() = 0;

    // Specify a time limit, in seconds
    virtual void setTimeLimit(  double seconds ) = 0;

    // Solve and extract the solution, or the best known bound on the
    // objective function
    virtual void solve( unsigned numThreads = 1 ) = 0;
    virtual double getValue( unsigned variable ) = 0;
    virtual double getObjective() = 0;
    virtual void extractSolution( Map<String, double> &values, double &costOrObjective ) = 0;
    virtual double getObjectiveBound() = 0;

    // Reset the underlying model
    virtual void reset() = 0;

    // Clear the underlying model and create a fresh model
    virtual void resetModel() = 0;

    // Dump the model to a file. Note that the suffix of the file is
    // used by Gurobi to determine the format. Using ".lp" is a good
    // default
    virtual void dumpModel( String name ) = 0;

    virtual unsigned getNumberOfSimplexIterations() = 0;

    virtual void updateModel() {};

    virtual unsigned getNumberOfNodes() { return 0; };

    virtual void setVerbosity( unsigned ) {};
};

#endif // __LPSolver_h__

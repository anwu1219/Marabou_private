/*********************                                                        */
/*! \file MockSolver.h
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

#ifndef __MockSolver_h__
#define __MockSolver_h__

#include "MString.h"
#include "Map.h"

class MockSolver : public LPSolver
{
public:

    Map<String, double> _lowerBounds;
    Map<String, double> _upperBounds;
    Map<String, double> _value;


   // Add a new variabel to the model
    void addVariable( String, double, double, VariableType ) {}


    double getLowerBound( unsigned var ) { return _lowerBounds[Stringf( "x%u", var )]; }
    double getUpperBound( unsigned var ) { return _upperBounds[Stringf( "x%u", var )]; }

   // Set the lower or upper bound for an existing variable
    void setLowerBound( String name, double lb )
    {
        _lowerBounds[name] = lb;
    }

    void setUpperBound( String name, double ub )
    {
        _upperBounds[name] = ub;
    }

    void setValue( String name, double value )
    {
        _value[name] = value;
    }

   // Add a new LEQ constraint, e.g. 3x + 4y <= -5
    void addLeqConstraint( const List<LPSolver::Term> &, double, String ){}

   // Add a new GEQ constraint, e.g. 3x + 4y >= -5
    void addGeqConstraint( const List<LPSolver::Term> &, double, String ){}

   // Add a new EQ constraint, e.g. 3x + 4y = -5
    void addEqConstraint( const List<LPSolver::Term> &, double, String ){}

    void removeConstraint( String ){}

   // A cost function to minimize, or an objective function to maximize
    void setCost( const List<LPSolver::Term> & ){}
    void setObjective( const List<LPSolver::Term> & ){}

    void setCutoff(  double ){}

    bool _isOptimal = false;
    bool optimal(){ return _isOptimal; }

    bool cutoffOccurred(){ return false; }

    bool _isInfeasible = false;
    bool infeasible(){ return _isInfeasible; }

    bool timeout(){ return false; }

    bool _haveFeasibleSolution = false;
    bool haveFeasibleSolution(){ return _haveFeasibleSolution; }

   // Specify a time limit, in seconds
    void setTimeLimit(  double ){}

   // Solve and extract the solution, or the best known bound on the
   // objective function
    void solve( unsigned numWorkers = 1 ){ std::cout << numWorkers; };
    double getValue( unsigned variable ){ return _value[Stringf("x%u", variable )]; }

    double _objective = 0;
    double getObjective(){ return _objective; }

    void extractSolution( Map<String, double> &, double & ){}
    double getObjectiveBound() { return 0; }

   // Reset the underlying model
    void reset(){}

   // Clear the underlying model and create a fresh model
    void resetModel(){}

   // Dump the model to a file. Note that the suffix of the file is
   // used by Gurobi to determine the format. Using ".lp" is a good
   // default
    void dumpModel( String ){}

    unsigned getNumberOfSimplexIterations(){ return 0; }
};

#endif // __MockSolver_h__

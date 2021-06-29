/*********************                                                        */
/*! \file Engine.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Duligur Ibeling, Andrew Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#ifndef __Engine_h__
#define __Engine_h__

#include "AutoRowBoundTightener.h"
#include "AutoTableau.h"
#include "BoundManager.h"
#include "context/cdo.h"
#include "context/context.h"
#include "DivideStrategy.h"
#include "HeuristicCostManager.h"
#include "SnCDivideStrategy.h"
#include "GlobalConfiguration.h"
#include "GurobiWrapper.h"
#include "IEngine.h"
#include "InputQuery.h"
#include "LPSolver.h"
#include "Map.h"
#include "MILPEncoder.h"
#include "Preprocessor.h"
#include "PseudoCostTracker.h"
#include "SignalHandler.h"
#include "SmtCore.h"
#include "Statistics.h"
#include "SymbolicBoundTighteningType.h"

#include "Vector.h"

#include <atomic>

#ifdef _WIN32
#undef ERROR
#endif

#define ENGINE_LOG(x, ...) LOG(GlobalConfiguration::ENGINE_LOGGING, "Engine: %s\n", x)

class EngineState;
class InputQuery;
class PiecewiseLinearConstraint;
class String;

class Engine : public IEngine, public SignalHandler::Signalable
{
public:
    enum {
          MICROSECONDS_TO_SECONDS = 1000000,
    };

    Engine();

    void setDone( std::atomic_bool *done )
    {
        _done = done;
    }

    /*
      Attempt to find a feasible solution for the input within a time limit
      (a timeout of 0 means no time limit). Returns true if found, false if infeasible.
    */
    bool solve( unsigned timeoutInSeconds = 0 );

    /*
      Process the input query and pass the needed information to the
      underlying tableau. Return false if query is found to be infeasible,
      true otherwise.
     */
    bool processInputQuery( InputQuery &inputQuery );
    bool processInputQuery( InputQuery &inputQuery, bool preprocess );

    /*
      If the query is feasiable and has been successfully solved, this
      method can be used to extract the solution.
     */
    void extractSolution( InputQuery &inputQuery );

    /*
      Methods for storing and restoring the state of the engine.
    */
    void storeTableauState( TableauState &state ) const;
    void restoreTableauState( const TableauState &state );
    void storeState( EngineState &state, bool storeAlsoTableauState ) const;
    void restoreState( const EngineState &state );

    /*
      Preprocessor access.
    */
    bool preprocessingEnabled() const;
    const Preprocessor *getPreprocessor();

    /*
      A request from the user to terminate
    */
    void quitSignal();

    /*
      Add equations and tightenings from a split.
    */
    void applySplit( const PiecewiseLinearCaseSplit &split );

    /*
      Reset the state of the engine, before solving a new query
      (as part of DnC mode).
    */
    void reset();

    /*
      Reset the statistics object
    */
    void resetStatistics();

    /*
      Clear the violated PL constraints
    */
    void clearViolatedPLConstraints();

    void setSeed( unsigned seed );

    void setBranchingHeuristic( DivideStrategy strategy )
    {
        _splittingStrategy = strategy;
    }

    /*
      Set the Engine's level of verbosity
    */
    void setVerbosity( unsigned verbosity );

    /*
      Pick the piecewise linear constraint for splitting
    */
    PiecewiseLinearConstraint *pickSplitPLConstraint();

    /*
      Call-back from QueryDividers
      Pick the piecewise linear constraint for splitting
    */
    PiecewiseLinearConstraint *pickSplitPLConstraintSnC( SnCDivideStrategy strategy );

    inline Vector<PiecewiseLinearConstraint *> &getViolatedPiecewiseLinearConstraints()
    {
        return _violatedPlConstraints;
    }

    inline SmtCore *getSmtCore()
    {
        return &_smtCore;
    }

    inline List<PiecewiseLinearConstraint *> &getPiecewiseLinearConstraints()
    {
        return _plConstraints;
    }


    const Statistics *getStatistics() const;

    InputQuery *getInputQuery();

    /*
      Get the exit code
    */
    Engine::ExitCode getExitCode() const;

    /*
      Get the quitRequested flag
    */
    std::atomic_bool *getQuitRequested();

    /*
      Get the list of input variables
    */
    List<unsigned> getInputVariables() const;

    /*
      Collect all violated piecewise linear constraints.
    */
    void collectViolatedPlConstraints();

    /*
      PSA: The following two methods are for DnC only and should be used very
      cautiously.
     */
    void resetSmtCore();
    void resetExitCode();
    void resetBoundTighteners();

    /*
      Solve the query with MILP encoding
    */
    bool _solveWithMILP;

    std::atomic_bool *_done;

    /*
      Solve the input query with a MILP solver (Gurobi)
    */
    bool solveWithMILPEncoding( unsigned timeoutInSeconds );

    bool _solutionFoundAndStoredInOriginalQuery;

    unsigned _numWorkers;

    unsigned _constraintViolationThreshold;

    /*
      Type of symbolic bound tightening
    */
    SymbolicBoundTighteningType _symbolicBoundTighteningType;
private:
    enum BasisRestorationRequired {
        RESTORATION_NOT_NEEDED = 0,
        STRONG_RESTORATION_NEEDED = 1,
        WEAK_RESTORATION_NEEDED = 2
    };

    enum BasisRestorationPerformed {
        NO_RESTORATION_PERFORMED = 0,
        PERFORMED_STRONG_RESTORATION = 1,
        PERFORMED_WEAK_RESTORATION = 2,
    };

    /*
      CVC4 Context Data structure
    */
    CVC4::context::Context _context;

    BoundManager _boundManager;

    PseudoCostTracker _costTracker;

    /*
      Collect and print various statistics.
    */
    Statistics _statistics;

    /*
      The tableau object maintains the equations, assignments and bounds.
    */
    AutoTableau _tableau;

    /*
      The existing piecewise-linear constraints.
    */
    List<PiecewiseLinearConstraint *> _plConstraints;

    List<DisjunctionConstraint *> _disjunctionConstraints;

    /*
      Piecewise linear constraints that are currently violated.
    */
    Vector<PiecewiseLinearConstraint *> _violatedPlConstraints;

    /*
      Preprocessed InputQuery
    */
    InputQuery _preprocessedQuery;

    /*
      Bound tightener.
    */
    AutoRowBoundTightener _rowBoundTightener;

    /*
      The SMT engine is in charge of case splitting.
    */
    SmtCore _smtCore;

    /*
      Query preprocessor.
    */
    Preprocessor _preprocessor;

    /*
      Is preprocessing enabled?
    */
    bool _preprocessingEnabled;

    /*
      Indicates a user/DnCManager request to quit
    */
    std::atomic_bool _quitRequested;

    /*
      A code indicating how the run terminated.
    */
    ExitCode _exitCode;

    /*
      An object that knows the topology of the network being checked,
      and can be used for various operations such as network
      evaluation of topology-based bound tightening.
     */
    NLR::NetworkLevelReasoner *_networkLevelReasoner;

    /*
      Verbosity level:
      0: print out minimal information
      1: print out statistics only in the beginning and the end
      2: print out statistics during solving
    */
    unsigned _verbosity;

    /*
      Strategy used for internal splitting
    */
    DivideStrategy _splittingStrategy;

    /*
      Disjunction that is used for splitting but doesn't exist in the beginning
    */
    std::unique_ptr<PiecewiseLinearConstraint> _disjunctionForSplitting;

    /*
      LPSolver object
    */
    std::unique_ptr<LPSolver> _gurobi;

    /*
      MILPEncoder
    */
    std::unique_ptr<MILPEncoder> _milpEncoder;

    /*
      Return true iff all variables are within bounds.
     */
    bool allVarsWithinBounds() const;

    /*
      Return true iff all piecewise linear constraints hold.
    */
    inline bool allPlConstraintsHold()
    {
        return _violatedPlConstraints.empty();
    }

    /*
      Apply all valid case splits proposed by the constraints.
      Return true if a valid case split has been applied.
    */
    bool applyAllValidConstraintCaseSplits();
    bool applyValidConstraintCaseSplit( PiecewiseLinearConstraint *constraint );

    /*
      Update statitstics, print them if needed.
    */
    void mainLoopStatistics();

    /*
      Perform bound tightening on the constraint matrix A.
    */
    void tightenBoundsOnConstraintMatrix();

    /*
      Perform a round of symbolic bound tightening, taking into
      account the current state of the piecewise linear constraints.
    */
    void performSymbolicBoundTightening( unsigned numWorkers = 1, bool randomLowerBound = false );

    /*
      Check whether a timeout value has been provided and exceeded.
    */
    inline bool shouldExitDueToTimeout( unsigned timeout ) const
    {
        return timeout != 0 && _statistics.getTotalTime() / MICROSECONDS_TO_SECONDS > timeout;
    }

    /*
      Helper functions for input query preprocessing
    */
    void informConstraintsOfInitialBounds( InputQuery &inputQuery ) const;
    void invokePreprocessor( const InputQuery &inputQuery, bool preprocess );
    void printInputBounds( const InputQuery &inputQuery ) const;
    void removeRedundantEquations( const double *constraintMatrix );
    void selectInitialVariablesForBasis( const double *constraintMatrix, List<unsigned> &initialBasis, List<unsigned> &basicRows );
    void initializeTableau( const double *constraintMatrix, const List<unsigned> &initialBasis );
    void initializeNetworkLevelReasoning();
    double *createConstraintMatrix();
    void addAuxiliaryVariables();
    void augmentInitialBasisIfNeeded( List<unsigned> &initialBasis, const List<unsigned> &basicRows );
    void performMILPSolverBoundedTightening();

    PiecewiseLinearConstraint *pickSplitPLConstraintBasedOnPolaritySOI();
    
    PiecewiseLinearConstraint *pickSplitPLConstraintBasedOnSOI();

    /*
      Among the earliest K ReLUs, pick the one with Polarity closest to 0.
      K is equal to GlobalConfiguration::POLARITY_CANDIDATES_THRESHOLD
    */
    PiecewiseLinearConstraint *pickSplitPLConstraintBasedOnPolarity();

    /*
      Pick the first unfixed ReLU in the topological order
    */
    PiecewiseLinearConstraint *pickSplitPLConstraintBasedOnTopology();

    /*
      Pick the input variable with the largest interval
    */
    PiecewiseLinearConstraint *pickSplitPLConstraintBasedOnIntervalWidth();

    /*
      Extract the satisfying assignment from the MILP solver
    */
    void extractSolutionFromGurobi( InputQuery &inputQuery );

    /*
      Evaluate the input assignment in the tableau with the network-level reasoner.
    */
    void concretizeInputAssignment();

    /*
      Perform bound tightening operations that require
      access to the explicit basis matrix.
    */
    void explicitBasisBoundTightening();

    /****************************** local search ****************************/
    inline void pushContext()
    {
        _context.push();
    }

    void popContext();

    /*
      Copy of the original input query
    */
    InputQuery _originalInputQuery;
    /*
      Seed for random stuff
    */
    unsigned _seed;

    HeuristicCostManager _heuristicCostManager;

    bool _costFunctionInitialized;

    String _scoreMetric;

    bool _constructTableau;

    String _flippingStrategy;

    bool _infiniteBoundsExists;

    void performBoundTightening();

    void solveLPWithGurobi( List<LPSolver::Term> &cost );

    void informLPSolverOfBounds();

    bool performSimulation();

    bool solveWithGurobi( unsigned timeoutInSeconds );

    /*
      Performs local search at the search level.
      Either throws InfeasibleQueryException,
      or return false with _needToSplit set to true and a branching variable picked.
      or return true with satisfying solution stored in the tableau.
    */
    bool performLocalSearch();

    // Optimize w.r.t. the current heuristic cost function
    void optimizeForHeuristicCost();

    // Notify the plConstraints of the assignments from Gurobi
    void notifyPLConstraintsAssignments();

    bool acceptProposedUpdate( double preivousCost, double currentCost );

    /*
      For Debugging
    */
    void checkBoundConsistency();

    void updateScore( PiecewiseLinearConstraint *constraint,
                      double previousCost, double currentCost );

};

#endif // __Engine_h__

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

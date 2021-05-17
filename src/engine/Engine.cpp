/*********************                                                        */
/*! \file Engine.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Duligur Ibeling, Andrew Wu
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

#include "AutoConstraintMatrixAnalyzer.h"
#include "Debug.h"
#include "DisjunctionConstraint.h"
#include "Engine.h"
#include "InfeasibleQueryException.h"
#include "InputQuery.h"
#include "MStringf.h"
#include "MarabouError.h"
#include "NLRError.h"
#include "Options.h"
#include "PiecewiseLinearConstraint.h"
#include "Preprocessor.h"
#include "TimeUtils.h"

#include "math.h"
#include <cstdlib>
#include <string.h>

Engine::Engine()
    : _context()
    , _boundManager( _context )
    , _rowBoundTightener( *_tableau, _boundManager )
    , _smtCore( this, _context )
    , _preprocessingEnabled( false )
    , _quitRequested( false )
    , _exitCode( Engine::NOT_DONE )
    , _networkLevelReasoner( NULL )
    , _verbosity( Options::get()->getInt( Options::VERBOSITY ) )
    , _splittingStrategy( Options::get()->getDivideStrategy() )
    , _symbolicBoundTighteningType( Options::get()->getSymbolicBoundTighteningType() )
    , _solveWithMILP( Options::get()->getBool( Options::SOLVE_WITH_MILP ) )
    , _gurobi( nullptr )
    , _milpEncoder( nullptr )
    , _solutionFoundAndStoredInOriginalQuery( false )
    , _seed( 1219 )
    , _heuristicCostManager( this )
    , _costFunctionInitialized( false )
    , _scoreMetric( Options::get()->getString( Options::SCORE_METRIC ) )
    , _constructTableau( Options::get()->getBool( Options::CONSTRUCT_TABLEAU ) )
    , _constraintViolationThreshold( Options::get()->getInt( Options::CONSTRAINT_VIOLATION_THRESHOLD ) )
    , _flippingStrategy( Options::get()->getString( Options::FLIPPING_STRATEGY ) )
{
    _smtCore.setStatistics( &_statistics );
    _tableau->setStatistics( &_statistics );
    _tableau->setBoundManager( &_boundManager );
    _rowBoundTightener->setStatistics( &_statistics );
    _preprocessor.setStatistics( &_statistics );
    _heuristicCostManager.setStatistics( &_statistics );

    _statistics.stampStartingTime();

    std::srand( _seed );
}

void Engine::setVerbosity( unsigned verbosity )
{
    _verbosity = verbosity;
}

void Engine::optimizeForHeuristicCost()
{
    Map<unsigned, double> &heuristicCost = _heuristicCostManager.getHeuristicCost();
    List<LPSolver::Term> terms;
    for ( const auto &term : heuristicCost )
        terms.append( LPSolver::Term( term.second,
                                           Stringf( "x%u", term.first ) ) );

    solveLPWithGurobi( terms );
}

bool Engine::performLocalSearch()
{
    ENGINE_LOG( "Performing local search..." );

    if ( _constraintViolationThreshold == 0 )
    {
      _smtCore.reportRandomFlip();
      return false;
    }
    
    // All the linear constraints have been satisfied at this point.
    // Update the cost function
    _heuristicCostManager.initiateCostFunctionForLocalSearch();
    optimizeForHeuristicCost();
    _heuristicCostManager.updateCostTermsForSatisfiedPLConstraints();
    double previousCost = _heuristicCostManager.computeHeuristicCost();
    _smtCore.reportRandomFlip();
    double currentCost = FloatUtils::infinity();

    ASSERT( allVarsWithinBounds() );

    bool lastCostAccepted = true;
    while ( !_smtCore.needToSplit() )
    {
        if ( lastCostAccepted )
        {
            collectViolatedPlConstraints();
            if ( allPlConstraintsHold() )
            {
                ASSERT( FloatUtils::isZero( _heuristicCostManager.computeHeuristicCost() ) );
                ENGINE_LOG( "Performing local search - done" );
                return true;
            }
            else
            {
                ASSERT( !FloatUtils::isZero( _heuristicCostManager.computeHeuristicCost() ) );
            }
        }

        PiecewiseLinearConstraint *lastFlippedConstraint = _heuristicCostManager.updateHeuristicCost();
        optimizeForHeuristicCost();
        _heuristicCostManager.updateCostTermsForSatisfiedPLConstraints();
        currentCost = _heuristicCostManager.computeHeuristicCost();

        updateScore( lastFlippedConstraint, previousCost, currentCost );

        if ( _flippingStrategy == "walksat" )
        {
            if ( currentCost > previousCost )
                _smtCore.reportRandomFlip();
            previousCost = currentCost;
            lastCostAccepted = true;
            _statistics.incLongAttr( Statistics::NUM_ACCEPTED_FLIPS, 1 );
            _statistics.setDoubleAttr( Statistics::HEURISTIC_COST, currentCost );
        }
        else
        {
            if ( !_heuristicCostManager.acceptProposedUpdate( previousCost, currentCost ) )
            {
                _smtCore.reportRandomFlip();
                _heuristicCostManager.undoLastHeuristicCostUpdate();
                lastCostAccepted = false;
                _statistics.incLongAttr( Statistics::NUM_REJECTED_FLIPS, 1 );
            }
            else
            {
                previousCost = currentCost;
                lastCostAccepted = true;
                _statistics.incLongAttr( Statistics::NUM_ACCEPTED_FLIPS, 1 );
                _statistics.setDoubleAttr( Statistics::HEURISTIC_COST, currentCost );
            }
        }
    }
    ENGINE_LOG( "Performing local search - done" );
    return false;
}

void Engine::concretizeInputAssignment()
{
    if ( !_networkLevelReasoner )
        return;

    unsigned numInputVariables = _preprocessedQuery.getNumInputVariables();
    unsigned numOutputVariables = _preprocessedQuery.getNumOutputVariables();

    if ( numInputVariables == 0 )
    {
        // Trivial case: all inputs are fixed, nothing to evaluate
        return;
    }

    double *inputAssignment = new double[numInputVariables];
    double *outputAssignment = new double[numOutputVariables];

    for ( unsigned i = 0; i < numInputVariables; ++i )
    {
        unsigned variable = _preprocessedQuery.inputVariableByIndex( i );
        inputAssignment[i] = _gurobi->getValue( variable );
    }

    // Evaluate the network for this assignment
    _networkLevelReasoner->evaluate( inputAssignment, outputAssignment );

    delete[] outputAssignment;
    delete[] inputAssignment;
}

void Engine::solveLPWithGurobi( List<LPSolver::Term> &cost )
{
    struct timespec simplexStart = TimeUtils::sampleMicro();

    _gurobi->setCost( cost );
    _gurobi->solve();

    struct timespec simplexEnd = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_SIMPLEX_STEPS_MICRO,
                             TimeUtils::timePassed( simplexStart, simplexEnd ) );
    _statistics.incLongAttr( Statistics::NUM_SIMPLIEX_CALLS, 1 );
    _statistics.incLongAttr( Statistics::NUM_SIMPLIEX_STEPS,
                             _gurobi->getNumberOfSimplexIterations() );
}

void Engine::informLPSolverOfBounds()
{
    struct timespec start = TimeUtils::sampleMicro();
    for ( unsigned i = 0; i < _preprocessedQuery.getNumberOfVariables(); ++i )
    {
        _gurobi->setLowerBound( Stringf( "x%u", i ), _boundManager.getLowerBound( i ) );
        _gurobi->setUpperBound( Stringf( "x%u", i ), _boundManager.getUpperBound( i ) );
    }
    _gurobi->updateModel();
    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_ADDING_CONSTRAINTS_TO_LP_SOLVER_MICRO,
                             TimeUtils::timePassed( start, end ) );

}

bool Engine::solveWithGurobi( unsigned timeoutInSeconds )
{
    struct timespec mainLoopStart = TimeUtils::sampleMicro();

    _statistics.resetTimeStatsForMainLoop();

    struct timespec start = TimeUtils::sampleMicro();
    _gurobi = std::unique_ptr<GurobiWrapper>( new GurobiWrapper() );
    _milpEncoder = std::unique_ptr<MILPEncoder>( new MILPEncoder( _boundManager, true ) );
    _milpEncoder->encodeInputQuery( *_gurobi, _preprocessedQuery );
    ENGINE_LOG( "Query encoded in Gurobi...\n" );
    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_ADDING_CONSTRAINTS_TO_LP_SOLVER_MICRO,
                             TimeUtils::timePassed( start, end ) );

    for ( const auto &constraint : _plConstraints )
        constraint->registerGurobi( &( *_gurobi ) );
    _heuristicCostManager.setGurobi( &(*_gurobi) );

    _costTracker.initialize( _plConstraints );

    mainLoopStatistics();
    if ( _verbosity > 0 )
    {
        printf( "\nEngine::solve: Initial statistics\n" );
        _statistics.print();
        printf( "\n---\n" );
    }

    applyAllValidConstraintCaseSplits();

    bool splitJustPerformed = true;
    while ( true )
    {
        struct timespec mainLoopEnd = TimeUtils::sampleMicro();
        _statistics.incLongAttr( Statistics::TIME_MAIN_LOOP_MICRO,
                                 TimeUtils::timePassed( mainLoopStart, mainLoopEnd ) );
        mainLoopStart = mainLoopEnd;

        struct timespec checkStart = TimeUtils::sampleMicro();
        if ( shouldExitDueToTimeout( timeoutInSeconds ) )
        {
            if ( _verbosity > 0 )
            {
                printf( "\n\nEngine: quitting due to timeout...\n\n" );
                printf( "Final statistics:\n" );
                _statistics.print();
            }

            _exitCode = Engine::TIMEOUT;
            _statistics.timeout();
            return false;
        }

        if ( _quitRequested )
        {
            if ( _verbosity > 0 )
            {
                printf( "\n\nEngine: quitting due to external request...\n\n" );
                printf( "Final statistics:\n" );
                _statistics.print();
            }

            _exitCode = Engine::QUIT_REQUESTED;
            return false;
        }
        struct timespec checkEnd = TimeUtils::sampleMicro();
        _statistics.incLongAttr( Statistics::TIME_CHECKING_QUIT_CONDITION_MICRO,
                                 TimeUtils::timePassed( checkStart, checkEnd ) );

        try
        {
            mainLoopStatistics();
            if ( _verbosity > 1 &&  _statistics.getLongAttr( Statistics::NUM_MAIN_LOOP_ITERATIONS ) %
                 GlobalConfiguration::STATISTICS_PRINTING_FREQUENCY == 0 )
                    _statistics.print();
            if ( splitJustPerformed )
            {
                performBoundTightening();
                splitJustPerformed = false;
                informLPSolverOfBounds();

                DEBUG({ checkBoundConsistency(); });
            }

            // Perform any SmtCore-initiated case splits
            if ( _smtCore.needToSplit() )
            {
                _smtCore.performSplit();
                splitJustPerformed = true;
                continue;
            }

            if ( _gurobi->haveFeasibleSolution() )
            {
                collectViolatedPlConstraints();
                if ( allPlConstraintsHold() ||  performLocalSearch() )
                {
                    struct timespec mainLoopEnd = TimeUtils::sampleMicro();
                    _statistics.incLongAttr( Statistics::TIME_MAIN_LOOP_MICRO,
                                             TimeUtils::timePassed( mainLoopStart, mainLoopEnd ) );
                    if ( _verbosity > 0 )
                    {
                        printf( "\nEngine::solve: sat assignment found\n" );
                        _statistics.print();
                    }
                    _exitCode = Engine::SAT;
                    return true;
                }
                continue;
            }

            // The linear portion is not satisfied, call Simplex
            List<LPSolver::Term> obj;
            solveLPWithGurobi( obj );
            if ( _gurobi->infeasible() )
            {
                throw InfeasibleQueryException();
            }
        }
        catch ( const InfeasibleQueryException & )
        {
            // The current query is unsat, and we need to pop.
            // If we're at level 0, the whole query is unsat.
            if ( !_smtCore.popSplit() )
            {
                struct timespec mainLoopEnd = TimeUtils::sampleMicro();
                _statistics.incLongAttr( Statistics::TIME_MAIN_LOOP_MICRO,
                                         TimeUtils::timePassed( mainLoopStart, mainLoopEnd ) );
                if ( _verbosity > 0 )
                {
                    printf( "\nEngine::solve: unsat query\n" );
                    _statistics.print();
                }
                _exitCode = Engine::UNSAT;
                return false;
            }
            else
            {
                splitJustPerformed = true;
            }

        }
        catch ( ... )
        {
            _exitCode = Engine::ERROR;
            printf( "Engine: Unknown error!\n" );
            return false;
        }
    }
}

bool Engine::solve( unsigned timeoutInSeconds )
{
    SignalHandler::getInstance()->initialize();
    SignalHandler::getInstance()->registerClient( this );

    if ( _solveWithMILP )
        return solveWithMILPEncoding( timeoutInSeconds );
    else
        return solveWithGurobi( timeoutInSeconds );
}

void Engine::mainLoopStatistics()
{
    struct timespec start = TimeUtils::sampleMicro();
    unsigned activeConstraints = 0;
    for ( const auto &constraint : _plConstraints )
        if ( constraint->isActive() )
            ++activeConstraints;

    _statistics.setUnsignedAttr( Statistics::NUM_ACTIVE_PIECEWISE_LINEAR_CONSTRAINTS,
                                 activeConstraints );
    _statistics.incLongAttr( Statistics::NUM_MAIN_LOOP_ITERATIONS, 1 );
    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_HANDLING_STATISTICS_MICRO,
                             TimeUtils::timePassed( start, end ) );
}

void Engine::performBoundTightening()
{
    if ( _constructTableau )
    {
      if ( _smtCore.getStackDepth()
	   <= GlobalConfiguration::EXPLICIT_BOUND_TIGHTENING_DEPTH_THRESHOLD &&
             _tableau->basisMatrixAvailable() )
        {
            explicitBasisBoundTightening();
            applyAllValidConstraintCaseSplits();
        }

        tightenBoundsOnConstraintMatrix();

        applyAllValidConstraintCaseSplits();
    }

    do
    {
        performSymbolicBoundTightening();
    }
    while ( applyAllValidConstraintCaseSplits() );
}

bool Engine::processInputQuery( InputQuery &inputQuery )
{
    return processInputQuery( inputQuery, GlobalConfiguration::PREPROCESS_INPUT_QUERY );
}

void Engine::informConstraintsOfInitialBounds( InputQuery &inputQuery ) const
{
    for ( const auto &plConstraint : inputQuery.getPiecewiseLinearConstraints() )
    {
        List<unsigned> variables = plConstraint->getParticipatingVariables();
        for ( unsigned variable : variables )
        {
            plConstraint->notifyLowerBound( variable, inputQuery.getLowerBound( variable ) );
            plConstraint->notifyUpperBound( variable, inputQuery.getUpperBound( variable ) );
        }
    }
}

void Engine::invokePreprocessor( const InputQuery &inputQuery, bool preprocess )
{
    if ( _verbosity > 0 )
        printf( "Engine::processInputQuery: Input query (before preprocessing): "
                "%u equations, %u variables\n",
                inputQuery.getEquations().size(),
                inputQuery.getNumberOfVariables() );

    // If processing is enabled, invoke the preprocessor
    _preprocessingEnabled = preprocess;
    if ( _preprocessingEnabled )
        _preprocessedQuery = _preprocessor.preprocess
            ( inputQuery, GlobalConfiguration::PREPROCESSOR_ELIMINATE_VARIABLES );
    else
        _preprocessedQuery = inputQuery;

    if ( _verbosity > 0 )
        printf( "Engine::processInputQuery: Input query (after preprocessing): "
                "%u equations, %u variables\n\n",
                _preprocessedQuery.getEquations().size(),
                _preprocessedQuery.getNumberOfVariables() );

    unsigned infiniteBounds = _preprocessedQuery.countInfiniteBounds();
    if ( infiniteBounds != 0 )
    {
        _exitCode = Engine::ERROR;
        throw MarabouError( MarabouError::UNBOUNDED_VARIABLES_NOT_YET_SUPPORTED,
                             Stringf( "Error! Have %u infinite bounds", infiniteBounds ).ascii() );
    }
}

void Engine::printInputBounds( const InputQuery &inputQuery ) const
{
    printf( "Input bounds:\n" );
    for ( unsigned i = 0; i < inputQuery.getNumInputVariables(); ++i )
    {
        unsigned variable = inputQuery.inputVariableByIndex( i );
        double lb, ub;
        bool fixed = false;
        if ( _preprocessingEnabled )
        {
            // Fixed variables are easy: return the value they've been fixed to.
            if ( _preprocessor.variableIsFixed( variable ) )
            {
                fixed = true;
                lb = _preprocessor.getFixedValue( variable );
                ub = lb;
            }
            else
            {
                // Has the variable been merged into another?
                while ( _preprocessor.variableIsMerged( variable ) )
                    variable = _preprocessor.getMergedIndex( variable );

                // We know which variable to look for, but it may have been assigned
                // a new index, due to variable elimination
                variable = _preprocessor.getNewIndex( variable );

                lb = _preprocessedQuery.getLowerBound( variable );
                ub = _preprocessedQuery.getUpperBound( variable );
            }
        }
        else
        {
            lb = inputQuery.getLowerBound( variable );
            ub = inputQuery.getUpperBound( variable );
        }

        printf( "\tx%u: [%8.4lf, %8.4lf] %s\n", i, lb, ub, fixed ? "[FIXED]" : "" );
    }
    printf( "\n" );
}

double *Engine::createConstraintMatrix()
{
    const List<Equation> &equations( _preprocessedQuery.getEquations() );
    unsigned m = equations.size();
    unsigned n = _preprocessedQuery.getNumberOfVariables();

    // Step 1: create a constraint matrix from the equations
    double *constraintMatrix = new double[n*m];
    if ( !constraintMatrix )
        throw MarabouError( MarabouError::ALLOCATION_FAILED, "Engine::constraintMatrix" );
    std::fill_n( constraintMatrix, n*m, 0.0 );

    unsigned equationIndex = 0;
    for ( const auto &equation : equations )
    {
        if ( equation._type != Equation::EQ )
        {
            _exitCode = Engine::ERROR;
            throw MarabouError( MarabouError::NON_EQUALITY_INPUT_EQUATION_DISCOVERED );
        }

        for ( const auto &addend : equation._addends )
            constraintMatrix[equationIndex*n + addend._variable] = addend._coefficient;

        ++equationIndex;
    }

    return constraintMatrix;
}

void Engine::removeRedundantEquations( const double *constraintMatrix )
{
    const List<Equation> &equations( _preprocessedQuery.getEquations() );
    unsigned m = equations.size();
    unsigned n = _preprocessedQuery.getNumberOfVariables();

    // Step 1: analyze the matrix to identify redundant rows
    AutoConstraintMatrixAnalyzer analyzer;
    analyzer->analyze( constraintMatrix, m, n );

    ENGINE_LOG( Stringf( "Number of redundant rows: %u out of %u",
                         analyzer->getRedundantRows().size(), m ).ascii() );

    // Step 2: remove any equations corresponding to redundant rows
    Set<unsigned> redundantRows = analyzer->getRedundantRows();

    if ( !redundantRows.empty() )
    {
        _preprocessedQuery.removeEquationsByIndex( redundantRows );
        m = equations.size();
    }
}

void Engine::selectInitialVariablesForBasis( const double *constraintMatrix, List<unsigned> &initialBasis, List<unsigned> &basicRows )
{
    /*
      This method permutes rows and columns in the constraint matrix (prior
      to the addition of auxiliary variables), in order to obtain a set of
      column that constitue a lower triangular matrix. The variables
      corresponding to the columns of this matrix join the initial basis.

      (It is possible that not enough variables are obtained this way, in which
      case the initial basis will have to be augmented later).
    */

    const List<Equation> &equations( _preprocessedQuery.getEquations() );

    unsigned m = equations.size();
    unsigned n = _preprocessedQuery.getNumberOfVariables();

    // Trivial case, or if a trivial basis is requested
    if ( ( m == 0 ) || ( n == 0 ) || GlobalConfiguration::ONLY_AUX_INITIAL_BASIS )
    {
        for ( unsigned i = 0; i < m; ++i )
            basicRows.append( i );

        return;
    }

    unsigned *nnzInRow = new unsigned[m];
    unsigned *nnzInColumn = new unsigned[n];

    std::fill_n( nnzInRow, m, 0 );
    std::fill_n( nnzInColumn, n, 0 );

    unsigned *columnOrdering = new unsigned[n];
    unsigned *rowOrdering = new unsigned[m];

    for ( unsigned i = 0; i < m; ++i )
        rowOrdering[i] = i;

    for ( unsigned i = 0; i < n; ++i )
        columnOrdering[i] = i;

    // Initialize the counters
    for ( unsigned i = 0; i < m; ++i )
    {
        for ( unsigned j = 0; j < n; ++j )
        {
            if ( !FloatUtils::isZero( constraintMatrix[i*n + j] ) )
            {
                ++nnzInRow[i];
                ++nnzInColumn[j];
            }
        }
    }

    DEBUG({
            for ( unsigned i = 0; i < m; ++i )
            {
                ASSERT( nnzInRow[i] > 0 );
            }
        });

    unsigned numExcluded = 0;
    unsigned numTriangularRows = 0;
    unsigned temp;

    while ( numExcluded + numTriangularRows < n )
    {
        // Do we have a singleton row?
        unsigned singletonRow = m;
        for ( unsigned i = numTriangularRows; i < m; ++i )
        {
            if ( nnzInRow[i] == 1 )
            {
                singletonRow = i;
                break;
            }
        }

        if ( singletonRow < m )
        {
            // Have a singleton row! Swap it to the top and update counters
            temp = rowOrdering[singletonRow];
            rowOrdering[singletonRow] = rowOrdering[numTriangularRows];
            rowOrdering[numTriangularRows] = temp;

            temp = nnzInRow[numTriangularRows];
            nnzInRow[numTriangularRows] = nnzInRow[singletonRow];
            nnzInRow[singletonRow] = temp;

            // Find the non-zero entry in the row and swap it to the diagonal
            DEBUG( bool foundNonZero = false );
            for ( unsigned i = numTriangularRows; i < n - numExcluded; ++i )
            {
                if ( !FloatUtils::isZero( constraintMatrix[rowOrdering[numTriangularRows] * n + columnOrdering[i]] ) )
                {
                    temp = columnOrdering[i];
                    columnOrdering[i] = columnOrdering[numTriangularRows];
                    columnOrdering[numTriangularRows] = temp;

                    temp = nnzInColumn[numTriangularRows];
                    nnzInColumn[numTriangularRows] = nnzInColumn[i];
                    nnzInColumn[i] = temp;

                    DEBUG( foundNonZero = true );
                    break;
                }
            }

            ASSERT( foundNonZero );

            // Remove all entries under the diagonal entry from the row counters
            for ( unsigned i = numTriangularRows + 1; i < m; ++i )
            {
                if ( !FloatUtils::isZero( constraintMatrix[rowOrdering[i] * n + columnOrdering[numTriangularRows]] ) )
                    --nnzInRow[i];
            }

            ++numTriangularRows;
        }
        else
        {
            // No singleton rows. Exclude the densest column
            unsigned maxDensity = nnzInColumn[numTriangularRows];
            unsigned column = numTriangularRows;

            for ( unsigned i = numTriangularRows; i < n - numExcluded; ++i )
            {
                if ( nnzInColumn[i] > maxDensity )
                {
                    maxDensity = nnzInColumn[i];
                    column = i;
                }
            }

            // Update the row counters to account for the excluded column
            for ( unsigned i = numTriangularRows; i < m; ++i )
            {
                double element = constraintMatrix[rowOrdering[i]*n + columnOrdering[column]];
                if ( !FloatUtils::isZero( element ) )
                {
                    ASSERT( nnzInRow[i] > 1 );
                    --nnzInRow[i];
                }
            }

            columnOrdering[column] = columnOrdering[n - 1 - numExcluded];
            nnzInColumn[column] = nnzInColumn[n - 1 - numExcluded];
            ++numExcluded;
        }
    }

    // Final basis: diagonalized columns + non-diagonalized rows
    List<unsigned> result;

    for ( unsigned i = 0; i < numTriangularRows; ++i )
    {
        initialBasis.append( columnOrdering[i] );
    }

    for ( unsigned i = numTriangularRows; i < m; ++i )
    {
        basicRows.append( rowOrdering[i] );
    }

    // Cleanup
    delete[] nnzInRow;
    delete[] nnzInColumn;
    delete[] columnOrdering;
    delete[] rowOrdering;
}

void Engine::addAuxiliaryVariables()
{
    List<Equation> &equations( _preprocessedQuery.getEquations() );

    unsigned m = equations.size();
    unsigned originalN = _preprocessedQuery.getNumberOfVariables();
    unsigned n = originalN + m;

    _preprocessedQuery.setNumberOfVariables( n );

    // Add auxiliary variables to the equations and set their bounds
    unsigned count = 0;
    for ( auto &eq : equations )
    {
        unsigned auxVar = originalN + count;
        eq.addAddend( -1, auxVar );
        _preprocessedQuery.setLowerBound( auxVar, eq._scalar );
        _preprocessedQuery.setUpperBound( auxVar, eq._scalar );
        eq.setScalar( 0 );

        ++count;
    }
}

void Engine::augmentInitialBasisIfNeeded( List<unsigned> &initialBasis, const List<unsigned> &basicRows )
{
    unsigned m = _preprocessedQuery.getEquations().size();
    unsigned n = _preprocessedQuery.getNumberOfVariables();
    unsigned originalN = n - m;

    if ( initialBasis.size() != m )
    {
        for ( const auto &basicRow : basicRows )
            initialBasis.append( basicRow + originalN );
    }
}

void Engine::initializeTableau( const double *constraintMatrix, const List<unsigned> &initialBasis )
{
    const List<Equation> &equations( _preprocessedQuery.getEquations() );
    unsigned m = equations.size();
    unsigned n = _preprocessedQuery.getNumberOfVariables();

    _tableau->setDimensions( m, n );

    unsigned equationIndex = 0;
    for ( const auto &equation : equations )
    {
        _tableau->setRightHandSide( equationIndex, equation._scalar );
        ++equationIndex;
    }

    // Populate constriant matrix
    _tableau->setConstraintMatrix( constraintMatrix );

    _tableau->registerToWatchAllVariables( _rowBoundTightener );

    _rowBoundTightener->setDimensions();

    _tableau->initializeTableau( initialBasis );
}

void Engine::initializeNetworkLevelReasoning()
{
    _networkLevelReasoner = _preprocessedQuery.getNetworkLevelReasoner();
    _heuristicCostManager.setNetworkLevelReasoner( _networkLevelReasoner );

    if ( _networkLevelReasoner )
        _networkLevelReasoner->setBoundManager( &_boundManager );
}

bool Engine::processInputQuery( InputQuery &inputQuery, bool preprocess )
{
    ENGINE_LOG( "processInputQuery starting\n" );

    struct timespec start = TimeUtils::sampleMicro();

    for ( const auto &constraint : inputQuery.getPiecewiseLinearConstraints() )
        constraint->initializeCDOs( &_context );

    _originalInputQuery = inputQuery;

    try
    {
        informConstraintsOfInitialBounds( inputQuery );
        invokePreprocessor( inputQuery, preprocess );
        if ( _verbosity > 0 )
            printInputBounds( inputQuery );

        _plConstraints = _preprocessedQuery.getPiecewiseLinearConstraints();
        _heuristicCostManager.setPLConstraints( _plConstraints );

        if ( _constructTableau )
        {
            double *constraintMatrix = createConstraintMatrix();
            removeRedundantEquations( constraintMatrix );

            // The equations have changed, recreate the constraint matrix
            delete[] constraintMatrix;
            constraintMatrix = createConstraintMatrix();

            List<unsigned> initialBasis;
            List<unsigned> basicRows;
            selectInitialVariablesForBasis( constraintMatrix, initialBasis, basicRows );
            addAuxiliaryVariables();
            augmentInitialBasisIfNeeded( initialBasis, basicRows );

            // The equations have changed, recreate the constraint matrix
            delete[] constraintMatrix;
            constraintMatrix = createConstraintMatrix();

            initializeTableau( constraintMatrix, initialBasis );

            delete[] constraintMatrix;
        }

        _statistics.setUnsignedAttr( Statistics::NUM_PIECEWISE_LINEAR_CONSTRAINTS,
                                     _plConstraints.size() );

        _boundManager.initialize( _preprocessedQuery.getNumberOfVariables() );

        for ( unsigned i = 0; i < _preprocessedQuery.getNumberOfVariables(); ++i )
        {
            _boundManager.setLowerBound( i, _preprocessedQuery.getLowerBound( i ) );
            _boundManager.setUpperBound( i, _preprocessedQuery.getUpperBound( i ) );
        }

        _boundManager.registerTableauReference( _tableau );

        initializeNetworkLevelReasoning();

        for ( const auto &constraint : _plConstraints )
        {
            constraint->registerBoundManager( &_boundManager );
            constraint->registerAsWatcher( _tableau );
            constraint->setStatistics( &_statistics );
        }

        if ( preprocess )
        {
            performSymbolicBoundTightening();
            performMILPSolverBoundedTightening();
        }

        if ( Options::get()->getBool( Options::DUMP_BOUNDS ) )
            _networkLevelReasoner->dumpBounds();

        if ( _splittingStrategy == DivideStrategy::Auto )
        {
            _splittingStrategy =
                ( _preprocessedQuery.getInputVariables().size() <
                  GlobalConfiguration::INTERVAL_SPLITTING_THRESHOLD ) ?
                DivideStrategy::LargestInterval : DivideStrategy::SOI;
        }

        struct timespec end = TimeUtils::sampleMicro();
        _statistics.setLongAttr( Statistics::TIME_PREPROCESSING_MICRO,
                                 TimeUtils::timePassed( start, end ) );
    }
    catch ( const InfeasibleQueryException & )
    {
        ENGINE_LOG( "processInputQuery done\n" );

        struct timespec end = TimeUtils::sampleMicro();
        _statistics.setLongAttr( Statistics::TIME_PREPROCESSING_MICRO,
                                 TimeUtils::timePassed( start, end ) );

        _exitCode = Engine::UNSAT;
        return false;
    }

    ENGINE_LOG( "processInputQuery done\n" );

    DEBUG({
            // Initially, all constraints should be active
            for ( const auto &plc : _plConstraints )
                {
                    ASSERT( plc->isActive() );
                }
        });

    return true;
}

void Engine::extractSolution( InputQuery &inputQuery )
{
    if ( _solutionFoundAndStoredInOriginalQuery )
    {
        std::cout << "Solution found by concretizing input!" << std::endl;
        for ( unsigned i = 0; i < inputQuery.getNumberOfVariables(); ++i )
        {
            inputQuery.setSolutionValue( i, _originalInputQuery.getSolutionValue( i ) );
            inputQuery.setLowerBound( i, _originalInputQuery.getSolutionValue( i ) );
            inputQuery.setUpperBound( i, _originalInputQuery.getSolutionValue( i ) );
        }
    }
    else
    {
        extractSolutionFromGurobi( inputQuery );
    }

    DEBUG({
            for ( const auto &eq : inputQuery.getEquations() )
            {
                auto addends = eq._addends;
                double scalar = eq._scalar;
                auto type = eq._type;
                double sum = 0;
                for ( const auto &addend : addends )
                {
                    sum += addend._coefficient *
                        inputQuery.getSolutionValue( addend._variable );
                }
                if ( type == Equation::EQ )
                    ASSERT( FloatUtils::areEqual( sum, scalar ) );
                if ( type == Equation::GE )
                    ASSERT( FloatUtils::gte( sum, scalar ) );
                if ( type == Equation::LE )
                    ASSERT( FloatUtils::lte( sum, scalar ) );
            }
        });
}

bool Engine::allVarsWithinBounds() const
{
    return _gurobi->haveFeasibleSolution();
}

void Engine::collectViolatedPlConstraints()
{
    struct timespec start = TimeUtils::sampleMicro();

    _violatedPlConstraints.clear();
    for ( const auto &constraint : _plConstraints )
    {
        if ( constraint->isActive() && !constraint->satisfied() )
            _violatedPlConstraints.append( constraint );
    }
    struct timespec end = TimeUtils::sampleMicro();

    _statistics.incLongAttr( Statistics::TIME_COLLECTING_VIOLATED_PLCONSTRAINT_MICRO,
                             TimeUtils::timePassed( start, end ) );
}

void Engine::applySplit( const PiecewiseLinearCaseSplit &split )
{
    ENGINE_LOG( "" );
    ENGINE_LOG( "Applying a split. " );

    const List<Tightening> &bounds = split.getBoundTightenings();
    ASSERT( split.getEquations().size() == 0 );

    for ( auto &bound : bounds )
    {
        if ( bound._type == Tightening::LB )
        {
            ENGINE_LOG( Stringf( "x%u: lower bound set to %.3lf", bound._variable, bound._value ).ascii() );
            _boundManager.tightenLowerBound( bound._variable, bound._value );
        }
        else
        {
            ENGINE_LOG( Stringf( "x%u: upper bound set to %.3lf", bound._variable, bound._value ).ascii() );
            _boundManager.tightenUpperBound( bound._variable, bound._value );
        }
    }
    ENGINE_LOG( "Done with split\n" );
}

bool Engine::applyAllValidConstraintCaseSplits()
{
    struct timespec start = TimeUtils::sampleMicro();

    bool appliedSplit = false;
    for ( auto &constraint : _plConstraints )
        if ( applyValidConstraintCaseSplit( constraint ) )
            appliedSplit = true;

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_PERFORMING_VALID_CASE_SPLITS_MICRO,
                             TimeUtils::timePassed( start, end ) );

    return appliedSplit;
}

bool Engine::applyValidConstraintCaseSplit( PiecewiseLinearConstraint *constraint )
{
    if ( constraint->isActive() && constraint->phaseFixed() )
    {
        String constraintString;
        constraint->dump( constraintString );
        ENGINE_LOG( Stringf( "A constraint has become valid. Dumping constraint: %s",
                             constraintString.ascii() ).ascii() );
        constraint->setActiveConstraint( false );
        PiecewiseLinearCaseSplit validSplit = constraint->getValidCaseSplit();
        applySplit( validSplit );
        _heuristicCostManager.removeCostComponentFromHeuristicCost( constraint );
        return true;
    }

    return false;
}

void Engine::explicitBasisBoundTightening()
{
    struct timespec start = TimeUtils::sampleMicro();

    bool saturation = GlobalConfiguration::EXPLICIT_BOUND_TIGHTENING_UNTIL_SATURATION;

    _statistics.incLongAttr( Statistics::NUM_EXPLICIT_BASIS_BOUND_TIGHTENING_ATTEMPT, 1 );

    switch ( GlobalConfiguration::EXPLICIT_BASIS_BOUND_TIGHTENING_TYPE )
    {
        case GlobalConfiguration::COMPUTE_INVERTED_BASIS_MATRIX:
            _rowBoundTightener->examineInvertedBasisMatrix( saturation );
            break;

        case GlobalConfiguration::USE_IMPLICIT_INVERTED_BASIS_MATRIX:
            _rowBoundTightener->examineImplicitInvertedBasisMatrix( saturation );
            break;

        case GlobalConfiguration::DISABLE_EXPLICIT_BASIS_TIGHTENING:
            break;
    }

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_EXPLICIT_BASIS_BOUND_TIGHTENING_MICRO, TimeUtils::timePassed( start, end ) );
}

void Engine::tightenBoundsOnConstraintMatrix()
{
    struct timespec start = TimeUtils::sampleMicro();

    _rowBoundTightener->examineConstraintMatrix( true );
    _statistics.incLongAttr( Statistics::NUM_CONSTRAINT_MATRIX_BOUND_TIGHTENING_ATTEMPT, 1 );

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_CONSTRAINT_MATRIX_TIGHTENING_MICRO,
                             TimeUtils::timePassed( start, end ) );
}

void Engine::performSymbolicBoundTightening()
{
    if ( _symbolicBoundTighteningType == SymbolicBoundTighteningType::NONE ||
         ( !_networkLevelReasoner ) )
        return;

    struct timespec start = TimeUtils::sampleMicro();

    unsigned numTightenedBounds = 0;

    // Step 1: tell the NLR about the current bounds
    _networkLevelReasoner->obtainCurrentBounds();

    // Step 2: perform SBT
    if ( _symbolicBoundTighteningType ==
         SymbolicBoundTighteningType::SYMBOLIC_BOUND_TIGHTENING )
        _networkLevelReasoner->symbolicBoundPropagation();
    else if ( _symbolicBoundTighteningType ==
         SymbolicBoundTighteningType::DEEP_POLY )
        _networkLevelReasoner->deepPolyPropagation();

    // Step 3: Extract the bounds
    List<Tightening> tightenings;
    _networkLevelReasoner->getConstraintTightenings( tightenings );

    for ( const auto &tightening : tightenings )
    {

        if ( tightening._type == Tightening::LB &&
             FloatUtils::gt( tightening._value, _boundManager.getLowerBound( tightening._variable ) ) )
        {
            _boundManager.tightenLowerBound( tightening._variable, tightening._value );
            ++numTightenedBounds;
        }

        if ( tightening._type == Tightening::UB &&
             FloatUtils::lt( tightening._value, _boundManager.getUpperBound( tightening._variable ) ) )
        {
            _boundManager.tightenUpperBound( tightening._variable, tightening._value );
            ++numTightenedBounds;
        }
    }

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_SYMBOLIC_BOUND_TIGHTENING_MICRO, TimeUtils::timePassed( start, end ) );
    _statistics.incLongAttr( Statistics::NUM_SYMBOLIC_BOUND_TIGHTENING, numTightenedBounds );
    _statistics.incLongAttr( Statistics::NUM_SYMBOLIC_BOUND_TIGHTENING_ATTEMPT, 1 );
}


void Engine::performMILPSolverBoundedTightening()
{
    if ( _networkLevelReasoner && Options::get()->gurobiEnabled() )
    {

        struct timespec start = TimeUtils::sampleMicro();

        unsigned numTightenedBounds = 0;

        _networkLevelReasoner->obtainCurrentBounds();

        switch ( Options::get()->getMILPSolverBoundTighteningType() )
        {
        case MILPSolverBoundTighteningType::LP_RELAXATION:
        case MILPSolverBoundTighteningType::LP_RELAXATION_INCREMENTAL:
            _networkLevelReasoner->lpRelaxationPropagation();
            break;

        case MILPSolverBoundTighteningType::MILP_ENCODING:
        case MILPSolverBoundTighteningType::MILP_ENCODING_INCREMENTAL:
            _networkLevelReasoner->MILPPropagation();
            break;
        case MILPSolverBoundTighteningType::ITERATIVE_PROPAGATION:
            _networkLevelReasoner->iterativePropagation();
            break;
        case MILPSolverBoundTighteningType::NONE:
            return;
        }

        List<Tightening> tightenings;
        _networkLevelReasoner->getConstraintTightenings( tightenings );

        for ( const auto &tightening : tightenings )
        {
            if ( tightening._type == Tightening::LB &&
                 FloatUtils::gt( tightening._value, _boundManager.getLowerBound( tightening._variable ) ) )
            {
                _boundManager.tightenLowerBound( tightening._variable, tightening._value );
                ++numTightenedBounds;
            }

            if ( tightening._type == Tightening::UB &&
                 FloatUtils::lt( tightening._value, _boundManager.getUpperBound( tightening._variable ) ) )
            {
                _boundManager.tightenUpperBound( tightening._variable, tightening._value );
                ++numTightenedBounds;
            }
        }

        struct timespec end = TimeUtils::sampleMicro();
        _statistics.incLongAttr( Statistics::TIME_LP_TIGHTENING_MICRO, TimeUtils::timePassed( start, end ) );
        _statistics.incLongAttr( Statistics::NUM_LP_BOUND_TIGHTENING, numTightenedBounds );
        _statistics.incLongAttr( Statistics::NUM_LP_BOUND_TIGHTENING_ATTEMPT, 1 );
    }
}

const Statistics *Engine::getStatistics() const
{
    return &_statistics;
}

InputQuery *Engine::getInputQuery()
{
    return &_preprocessedQuery;
}

void Engine::quitSignal()
{
    _quitRequested = true;
}

Engine::ExitCode Engine::getExitCode() const
{
    return _exitCode;
}

std::atomic_bool *Engine::getQuitRequested()
{
    return &_quitRequested;
}

List<unsigned> Engine::getInputVariables() const
{
    return _preprocessedQuery.getInputVariables();
}

void Engine::reset()
{
    resetStatistics();
    clearViolatedPLConstraints();
    resetSmtCore();
    resetExitCode();
}

void Engine::resetStatistics()
{
    Statistics statistics;
    _statistics = statistics;
    _smtCore.setStatistics( &_statistics );
    _tableau->setStatistics( &_statistics );
    _rowBoundTightener->setStatistics( &_statistics );
    _preprocessor.setStatistics( &_statistics );
    _statistics.stampStartingTime();
}

void Engine::clearViolatedPLConstraints()
{
    _violatedPlConstraints.clear();
}

void Engine::resetSmtCore()
{
    _smtCore.reset();
}

void Engine::resetExitCode()
{
    _exitCode = Engine::NOT_DONE;
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintBasedOnSOI()
{
    ENGINE_LOG( Stringf( "Using SOI-based heuristics..." ).ascii() );
    return _costTracker.topUnfixed();
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintBasedOnPolaritySOI()
{
    ENGINE_LOG( Stringf( "Using SOI-Polarity-based heuristics..." ).ascii() );
    if ( _smtCore.getStackDepth() < 3 )
    {
      return pickSplitPLConstraintBasedOnPolarity();
    }
    else
      return _costTracker.topUnfixed();
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintBasedOnPolarity()
{
    ENGINE_LOG( Stringf( "Using Polarity-based heuristics..." ).ascii() );

    if ( _preprocessedQuery.getInputVariables().size() <
	 GlobalConfiguration::INTERVAL_SPLITTING_THRESHOLD )
      return pickSplitPLConstraintBasedOnIntervalWidth();
    
    if ( !_networkLevelReasoner )
        throw MarabouError( MarabouError::NETWORK_LEVEL_REASONER_NOT_AVAILABLE );

    List<PiecewiseLinearConstraint *> &constraints =
        _networkLevelReasoner->getConstraintsInTopologicalOrder();

    Map<double, PiecewiseLinearConstraint *> scoreToConstraint;
    for ( auto &plConstraint : constraints )
    {
        if ( plConstraint->supportPolarity() &&
             plConstraint->isActive() && !plConstraint->phaseFixed() )
        {
            plConstraint->updateScoreBasedOnPolarity();
            scoreToConstraint[plConstraint->getScore()] = plConstraint;
            if ( scoreToConstraint.size() >=
                 GlobalConfiguration::POLARITY_CANDIDATES_THRESHOLD )
                break;
        }
    }
    if ( scoreToConstraint.size() > 0 )
    {
        ENGINE_LOG( Stringf( "Score of the picked ReLU: %f",
                             ( *scoreToConstraint.begin() ).first ).ascii() );
        return (*scoreToConstraint.begin()).second;
    }
    else
        return NULL;
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintBasedOnTopology()
{
    // We push the first unfixed ReLU in the topology order to the _candidatePlConstraints
    ENGINE_LOG( Stringf( "Using EarliestReLU heuristics..." ).ascii() );

    if ( !_networkLevelReasoner )
        throw MarabouError( MarabouError::NETWORK_LEVEL_REASONER_NOT_AVAILABLE );

    List<PiecewiseLinearConstraint *> constraints =
        _networkLevelReasoner->getConstraintsInTopologicalOrder();

    for ( auto &plConstraint : constraints )
    {
        if ( plConstraint->isActive() && !plConstraint->phaseFixed() )
            return plConstraint;
    }
    return NULL;
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintBasedOnIntervalWidth()
{
    // We push the first unfixed ReLU in the topology order to the _candidatePlConstraints
    ENGINE_LOG( Stringf( "Using LargestInterval heuristics..." ).ascii() );

    unsigned inputVariableWithLargestInterval = 0;
    double largestIntervalSoFar = 0;
    for ( const auto &variable : _preprocessedQuery.getInputVariables() )
    {
        double interval = _boundManager.getUpperBound( variable ) -
            _boundManager.getLowerBound( variable );
        if ( interval > largestIntervalSoFar )
        {
            inputVariableWithLargestInterval = variable;
            largestIntervalSoFar = interval;
        }
    }

    if ( largestIntervalSoFar == 0 )
        return NULL;
    else
    {
        double mid = ( _boundManager.getLowerBound( inputVariableWithLargestInterval )
                       + _boundManager.getUpperBound( inputVariableWithLargestInterval )
                       ) / 2;
        PiecewiseLinearCaseSplit s1;
        s1.storeBoundTightening( Tightening( inputVariableWithLargestInterval,
                                             mid, Tightening::UB ) );
        PiecewiseLinearCaseSplit s2;
        s2.storeBoundTightening( Tightening( inputVariableWithLargestInterval,
                                             mid, Tightening::LB ) );

        List<PiecewiseLinearCaseSplit> splits;
        splits.append( s1 );
        splits.append( s2 );
        _disjunctionForSplitting = std::unique_ptr<DisjunctionConstraint>
            ( new DisjunctionConstraint( splits ) );
        _disjunctionForSplitting->initializeCDOs( &_context );
        return _disjunctionForSplitting.get();
    }
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraint()
{
    ENGINE_LOG( Stringf( "Picking a split PLConstraint..." ).ascii() );

    PiecewiseLinearConstraint *candidatePLConstraint = NULL;
    if ( _splittingStrategy == DivideStrategy::EarliestReLU )
        candidatePLConstraint = pickSplitPLConstraintBasedOnTopology();
    else if ( _splittingStrategy == DivideStrategy::Polarity )
        candidatePLConstraint = pickSplitPLConstraintBasedOnPolarity();
    else if ( _splittingStrategy == DivideStrategy::SOI )
        candidatePLConstraint = pickSplitPLConstraintBasedOnSOI();
    else if ( _splittingStrategy == DivideStrategy::SOIPolarity )
        candidatePLConstraint = pickSplitPLConstraintBasedOnPolaritySOI();

    else if ( _splittingStrategy == DivideStrategy::LargestInterval )
    {
        // Conduct interval splitting periodically.
        if ( _smtCore.getStackDepth() %
             GlobalConfiguration::INTERVAL_SPLITTING_FREQUENCY == 0 )
            candidatePLConstraint = pickSplitPLConstraintBasedOnIntervalWidth();
        else
            candidatePLConstraint = pickSplitPLConstraintBasedOnTopology();
    }
    else
    {
        ASSERT( false );
    }
    ENGINE_LOG( Stringf( ( candidatePLConstraint ?
                           "Picked..." :
                           "Unable to pick using the current strategy..." ) ).ascii() );
    
    return candidatePLConstraint;
}

PiecewiseLinearConstraint *Engine::pickSplitPLConstraintSnC( SnCDivideStrategy strategy )
{
    PiecewiseLinearConstraint *candidatePLConstraint = NULL;
    if ( strategy == SnCDivideStrategy::Polarity )
        candidatePLConstraint = pickSplitPLConstraintBasedOnPolarity();
    else if ( strategy == SnCDivideStrategy::EarliestReLU )
        candidatePLConstraint = pickSplitPLConstraintBasedOnTopology();

    ENGINE_LOG( Stringf( "Done updating scores..." ).ascii() );
    ENGINE_LOG( Stringf( ( candidatePLConstraint ?
                           "Picked..." :
                           "Unable to pick using the current strategy..." ) ).ascii() );
    return candidatePLConstraint;
}

bool Engine::solveWithMILPEncoding( unsigned timeoutInSeconds )
{
    try
    {
        performBoundTightening();
    }
    catch ( const InfeasibleQueryException & )
    {
        _exitCode = Engine::UNSAT;
        return false;
    }

    struct timespec start = TimeUtils::sampleMicro();
    ENGINE_LOG( "Encoding the input query with Gurobi...\n" );
    _gurobi = std::unique_ptr<GurobiWrapper>( new GurobiWrapper() );
    _milpEncoder = std::unique_ptr<MILPEncoder>( new MILPEncoder( _boundManager ) );
    _milpEncoder->encodeInputQuery( *_gurobi, _preprocessedQuery );
    ENGINE_LOG( "Query encoded in Gurobi...\n" );

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_ADDING_CONSTRAINTS_TO_LP_SOLVER_MICRO, TimeUtils::timePassed( start, end ) );

    double timeoutForGurobi = ( timeoutInSeconds == 0 ? FloatUtils::infinity()
                                : timeoutInSeconds );
    ENGINE_LOG( Stringf( "Gurobi timeout set to %f\n", timeoutForGurobi ).ascii() )
    _gurobi->setTimeLimit( timeoutForGurobi );
    _gurobi->setVerbosity( _verbosity );

    _gurobi->solve();

    _statistics.setUnsignedAttr( Statistics::NUM_VISITED_TREE_STATES, _gurobi->getNumberOfNodes() );
    if ( _verbosity > 0 )
        _statistics.print();
    if ( _gurobi->haveFeasibleSolution() )
    {
        _exitCode = IEngine::SAT;
        return true;
    }
    else if ( _gurobi->infeasible() )
        _exitCode = IEngine::UNSAT;
    else if ( _gurobi->timeout() )
        _exitCode = IEngine::TIMEOUT;
    else
        throw NLRError( NLRError::UNEXPECTED_RETURN_STATUS_FROM_GUROBI );
    return false;
}

void Engine::extractSolutionFromGurobi( InputQuery &inputQuery )
{
    ASSERT( _gurobi != nullptr );
    Map<String, double> assignment;
    double costOrObjective;
    _gurobi->extractSolution( assignment, costOrObjective );

    for ( unsigned i = 0; i < inputQuery.getNumberOfVariables(); ++i )
    {
        if ( _preprocessingEnabled )
        {
            // Has the variable been merged into another?
            unsigned variable = i;
            while ( _preprocessor.variableIsMerged( variable ) )
                variable = _preprocessor.getMergedIndex( variable );

            // Fixed variables are easy: return the value they've been fixed to.
            if ( _preprocessor.variableIsFixed( variable ) )
            {
                inputQuery.setSolutionValue( i, _preprocessor.getFixedValue( variable ) );
                inputQuery.setLowerBound( i, _preprocessor.getFixedValue( variable ) );
                inputQuery.setUpperBound( i, _preprocessor.getFixedValue( variable ) );
                continue;
            }

            // We know which variable to look for, but it may have been assigned
            // a new index, due to variable elimination
            variable = _preprocessor.getNewIndex( variable );

            // Finally, set the assigned value
            String variableName = _milpEncoder->getVariableNameFromVariable( variable );
            inputQuery.setSolutionValue( i, assignment[variableName] );
        }
        else
        {
            String variableName = _milpEncoder->getVariableNameFromVariable( i );
            inputQuery.setSolutionValue( i, assignment[variableName] );
        }
    }
}

void Engine::popContext()
{
    _context.pop();
}

void Engine::checkBoundConsistency()
{
    for ( unsigned i = 0; i < _preprocessedQuery.getNumberOfVariables(); ++i )
    {
        if ( !FloatUtils::areEqual( _gurobi->getLowerBound( i ), _boundManager.getLowerBound( i ) ) )
        {
            printf( "x%u lower bound inconsistent! In Gurobi: %f, in BoundManager %f",
                    i, _gurobi->getLowerBound( i ), _boundManager.getLowerBound( i ) );
            ASSERT( false );
        }
        if ( !FloatUtils::areEqual( _gurobi->getUpperBound( i ), _boundManager.getUpperBound( i ) ) )
        {
            printf( "x%u upper bound inconsistent! In Gurobi: %f, in BoundManager %f",
                    i, _gurobi->getUpperBound( i ), _boundManager.getUpperBound( i ) );
            ASSERT( false );
        }
    }
}

void Engine::updateScore( PiecewiseLinearConstraint *constraint,
                          double previousCost, double currentCost )
{
    struct timespec start = TimeUtils::sampleMicro();

    ASSERT( constraint != NULL );
    double score = 0;
    if ( _scoreMetric == "increase" )
    {
        score = currentCost - previousCost;
	if ( score < 0 )
	  score = 0;
    }
    else if ( _scoreMetric == "reduction" )
    {
        score = previousCost - currentCost;
    }
    else if ( _scoreMetric == "change" )
    {
        score = fabs( previousCost - currentCost );
        ASSERT( score >= 0 );
    }
    else
    {
        throw MarabouError( MarabouError::UNKNOWN_LOCAL_SEARCH_STRATEGY );
    }

    ENGINE_LOG( Stringf( "new score is %f (metric: %s)", score, _scoreMetric.ascii() ).ascii() );

    _costTracker.updateScore( constraint, score );

    struct timespec end = TimeUtils::sampleMicro();
    _statistics.incLongAttr( Statistics::TIME_BRANCHING_HEURISTICS_MICRO,
                             TimeUtils::timePassed( start, end ) );
}

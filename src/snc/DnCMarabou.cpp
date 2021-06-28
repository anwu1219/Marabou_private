/*********************                                                        */
/*! \file Marabou.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Haoze Wu
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

#include "DnCMarabou.h"
#include "File.h"
#include "MStringf.h"
#include "Options.h"
#include "PropertyParser.h"
#include "MarabouError.h"
#include "QueryLoader.h"
#include "AcasParser.h"

#include <boost/thread.hpp>
#include <csignal>

DnCMarabou::DnCMarabou()
    : _dncManager( nullptr )
    , _inputQuery( InputQuery() )
{
}

void DnCMarabou::run()
{
    String inputQueryFilePath = Options::get()->getString( Options::INPUT_QUERY_FILE_PATH );
    if ( inputQueryFilePath.length() > 0 )
    {
        /*
          Step 1: extract the query
        */
        if ( !File::exists( inputQueryFilePath ) )
        {
            printf( "Error: the specified inputQuery file (%s) doesn't exist!\n", inputQueryFilePath.ascii() );
            throw MarabouError( MarabouError::FILE_DOESNT_EXIST, inputQueryFilePath.ascii() );
        }

        printf( "InputQuery: %s\n", inputQueryFilePath.ascii() );
        _inputQuery = QueryLoader::loadQuery( inputQueryFilePath );
        _inputQuery.constructNetworkLevelReasoner();
    }
    else
    {
        /*
          Step 1: extract the network
        */
        String networkFilePath = Options::get()->getString( Options::INPUT_FILE_PATH );
        if ( !File::exists( networkFilePath ) )
        {
            printf( "Error: the specified network file (%s) doesn't exist!\n",
                    networkFilePath.ascii() );
            throw MarabouError( MarabouError::FILE_DOESNT_EXIST,
                                networkFilePath.ascii() );
        }
        printf( "Network: %s\n", networkFilePath.ascii() );

        AcasParser acasParser( networkFilePath );
        acasParser.generateQuery( _inputQuery );
        _inputQuery.constructNetworkLevelReasoner();

        /*
          Step 2: extract the property in question
        */
        String propertyFilePath = Options::get()->getString( Options::PROPERTY_FILE_PATH );
        if ( propertyFilePath != "" )
        {
            printf( "Property: %s\n", propertyFilePath.ascii() );
            PropertyParser().parse( propertyFilePath, _inputQuery );
        }
        else
            printf( "Property: None\n" );
    }
    printf( "\n" );

    String queryDumpFilePath = Options::get()->getString( Options::QUERY_DUMP_FILE );
    if ( queryDumpFilePath.length() > 0 )
    {
        _inputQuery.saveQuery( queryDumpFilePath );
        printf( "\nInput query successfully dumped to file\n" );
        exit( 0 );
    }

    /*
      Step 3: initialize the DNC core
    */

    // 1. preprocess with 48 threads
    _engine1._numWorkers = 48;
    _engine1.setVerbosity(0);
    if ( !_engine1.processInputQuery( _inputQuery ) )
    {
        std::cout << "Solved by preprocessing" << std::endl;
        String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );
        File summaryFile( summaryFilePath );
        summaryFile.open( File::MODE_WRITE_TRUNCATE );
        summaryFile.write( "holds\n" );
        return;
    }

    // If disjunction exists
    // 1. # disj * ( SOI + Polarity ) * (2 seed)
    // 2. rest goes to Gurobi

    if ( _inputQuery.getDisjunctionConstraints().size() == 1 )
    {
        solveDisjunction();
    }
    else if ( _inputQuery.getDisjunctionConstraints().size() == 0 )
    {
        solveNoDisjunction();
    }
}

void DnCMarabou::solveDisjunction()
{
    struct timespec start = TimeUtils::sampleMicro();

    boost::thread *threads = new boost::thread[5];
    std::unique_ptr<DnCManager> dncManager1 = std::unique_ptr<DnCManager>
        ( new DnCManager( _engine1.getInputQuery() ) );
    std::unique_ptr<DnCManager> dncManager2 = std::unique_ptr<DnCManager>
        ( new DnCManager( _engine1.getInputQuery() ) );
    std::unique_ptr<DnCManager> dncManager3 = std::unique_ptr<DnCManager>
        ( new DnCManager( _engine1.getInputQuery() ) );
    std::unique_ptr<DnCManager> dncManager4 = std::unique_ptr<DnCManager>
        ( new DnCManager( _engine1.getInputQuery() ) );

    std::atomic_bool done (false);
    dncManager1->setDone( &done );
    dncManager2->setDone( &done );
    dncManager3->setDone( &done );
    dncManager4->setDone( &done );
    _engine1.setDone( &done );

    std::unique_ptr<InputQuery> newInputQuery =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );

    std::mutex mtx;
    DisjunctionConstraint *disj = *(_inputQuery.getDisjunctionConstraints().begin());
    unsigned numDisj = disj->getCaseSplits().size();
    std::cout << "number of disjunctions: " << numDisj << std::endl;

    threads[0] = boost::thread( solveDnC, DnCArgument( &(*dncManager1), &mtx, 0, numDisj ) );
    threads[1] = boost::thread( solveDnC, DnCArgument( &(*dncManager2), &mtx, 1, numDisj ) );
    threads[2] = boost::thread( solveDnC, DnCArgument( &(*dncManager3), &mtx, 2, numDisj ) );
    threads[3] = boost::thread( solveDnC, DnCArgument( &(*dncManager4), &mtx, 3, numDisj ) );
    threads[4] = boost::thread( solveMILP, DnCArgument( &_engine1, &(*newInputQuery), &mtx, 48 - 4 * numDisj ) );

    boost::chrono::milliseconds waitTime( 100 );
    while ( !done.load() )
        boost::this_thread::sleep_for( waitTime );

    if ( done.load() )
    {
        struct timespec end = TimeUtils::sampleMicro();

        unsigned long long totalElapsed = TimeUtils::timePassed( start, end );
        displayResults( totalElapsed );

        for ( unsigned i = 0; i < 1; ++i )
        {
            pthread_kill(threads[i].native_handle(), 9);
            threads[i].join();
        }
    }
}

void DnCMarabou::solveNoDisjunction()
{
    struct timespec start = TimeUtils::sampleMicro();

    boost::thread *threads = new boost::thread[10];
    std::unique_ptr<DnCManager> dncManager1 = std::unique_ptr<DnCManager>
        ( new DnCManager( _engine1.getInputQuery() ) );
    Engine engine1;
    Engine engine2;
    Engine engine3;
    Engine engine4;
    Engine engine5;
    Engine engine6;
    Engine engine7;
    Engine engine8;

    engine1.setSeed(1);
    engine2.setSeed(2);
    engine3.setSeed(3);
    engine4.setSeed(4);
    engine5.setSeed(5);
    engine6.setSeed(6);
    engine7.setSeed(7);
    engine8.setSeed(8);

    engine1.setBranchingHeuristic(DivideStrategy::Polarity);
    engine2.setBranchingHeuristic(DivideStrategy::Polarity);
    engine3.setBranchingHeuristic(DivideStrategy::Polarity);
    engine4.setBranchingHeuristic(DivideStrategy::SOI);
    engine5.setBranchingHeuristic(DivideStrategy::SOI);
    engine6.setBranchingHeuristic(DivideStrategy::SOI);
    engine7.setBranchingHeuristic(DivideStrategy::SOI);
    engine8.setBranchingHeuristic(DivideStrategy::SOI);

    std::atomic_bool done (false);
    dncManager1->setDone( &done );
    _engine1.setDone( &done );
    engine1.setDone( &done );
    engine2.setDone( &done );
    engine3.setDone( &done );
    engine4.setDone( &done );
    engine5.setDone( &done );
    engine6.setDone( &done );
    engine7.setDone( &done );
    engine8.setDone( &done );

    std::unique_ptr<InputQuery> newInputQuery =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery1 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery2 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery3 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery4 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery5 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery6 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery7 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery8 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );

    _inputQuery = InputQuery();
    std::mutex mtx;
    threads[0] = boost::thread( solveDnC, DnCArgument( &(*dncManager1), &mtx, 1, 16 ) );
    threads[1] = boost::thread( solveSingleThread, DnCArgument( &engine1, &(*newInputQuery1), &mtx, 1 ) );
    threads[2] = boost::thread( solveSingleThread, DnCArgument( &engine2, &(*newInputQuery2), &mtx, 1 ) );
    threads[3] = boost::thread( solveSingleThread, DnCArgument( &engine3, &(*newInputQuery3), &mtx, 1 ) );
    threads[4] = boost::thread( solveSingleThread, DnCArgument( &engine4, &(*newInputQuery4), &mtx, 1 ) );
    threads[5] = boost::thread( solveSingleThread, DnCArgument( &engine5, &(*newInputQuery5), &mtx, 1 ) );
    threads[6] = boost::thread( solveSingleThread, DnCArgument( &engine6, &(*newInputQuery6), &mtx, 1 ) );
    threads[7] = boost::thread( solveSingleThread, DnCArgument( &engine7, &(*newInputQuery7), &mtx, 1 ) );
    threads[8] = boost::thread( solveSingleThread, DnCArgument( &engine8, &(*newInputQuery8), &mtx, 1 ) );
    threads[9] = boost::thread( solveMILP, DnCArgument( &_engine1, &(*newInputQuery), &mtx, 24 ) );

    boost::chrono::milliseconds waitTime( 100 );
    while ( !done.load() )
        boost::this_thread::sleep_for( waitTime );

    if ( done.load() )
    {
        struct timespec end = TimeUtils::sampleMicro();

        unsigned long long totalElapsed = TimeUtils::timePassed( start, end );
        displayResults( totalElapsed );

        for ( unsigned i = 0; i < 10; ++i )
        {
            pthread_kill(threads[i].native_handle(), 9);
            threads[i].join();
        }
    }
}

void DnCMarabou::solveDnC( DnCArgument argument )
{
    DnCManager *dncManager = argument._dncManager;
    std::mutex &mtx = *(argument._mtx);
    unsigned numDisj = argument._numDisj;
    unsigned id = argument._id;
    std::cout << "SnC starting " << id << std::endl;
    if ( id == 0 )
        dncManager->solve( numDisj, 0 );
    else if ( id == 1 )
        dncManager->solve( numDisj, 1 );
    else if ( id == 2 )
        dncManager->solve( numDisj, 2 );
    else if ( id == 3 )
        dncManager->solve( numDisj, 3 );
    mtx.lock();
    dncManager->printResult();
    std::cout << "Solved by DnC " << id << std::endl;
    String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );
    File summaryFile( summaryFilePath );
    summaryFile.open( File::MODE_WRITE_TRUNCATE );
    if ( dncManager->getExitCode() == DnCManager::UNSAT )
        summaryFile.write( "holds\n" );
    else if ( dncManager->getExitCode() == DnCManager::SAT )
        summaryFile.write( "violated\n" );
    mtx.unlock();

    *(dncManager->_done) = true;
}

void DnCMarabou::solveMILP( DnCArgument argument )
{
    Engine *engine = argument._engine;
    InputQuery *inputQuery = argument._inputQuery;
    std::mutex &mtx = *(argument._mtx);
    engine->_solveWithMILP = true;
    unsigned numDisj = argument._numDisj;
    std::cout << "MILP threads: " << numDisj << std::endl;
    engine->_numWorkers = numDisj;
    engine->setVerbosity(0);
    engine->solveWithMILPEncoding(0);
    if ( engine->getExitCode() == Engine::SAT )
        engine->extractSolution( *inputQuery );

    mtx.lock();
    Engine::ExitCode result = engine->getExitCode();

    if ( result == Engine::UNSAT )
    {
        printf( "unsat\n" );
    }
    else if ( result == Engine::SAT )
    {
        if ( inputQuery->_networkLevelReasoner && !engine->_solutionFoundAndStoredInOriginalQuery )
        {
            double *input = new double[inputQuery->getNumInputVariables()];
            for ( unsigned i = 0; i < inputQuery->getNumInputVariables(); ++i )
                input[i] = inputQuery->getSolutionValue( inputQuery->inputVariableByIndex( i ) );

            NLR::NetworkLevelReasoner *nlr = inputQuery->_networkLevelReasoner;
            NLR::Layer *lastLayer = nlr->getLayer( nlr->getNumberOfLayers() - 1 );
            double *output = new double[lastLayer->getSize()];

            nlr->evaluate( input, output );

            printf( "\n" );
            printf( "Output:\n" );
            for ( unsigned i = 0; i < lastLayer->getSize(); ++i )
                printf( "y%u = %lf\n", i, output[i] );
            printf( "\n" );

            printf( "sat\n" );
        }
        else
        {
            printf( "sat\n" );
        }
    }
    else if ( result == Engine::TIMEOUT )
    {
        printf( "Timeout\n" );
    }
    else if ( result == Engine::ERROR )
    {
        printf( "Error\n" );
    }
    else
    {
        printf( "UNKNOWN EXIT CODE! (this should not happen)" );
    }

    std::cout << "Solved by MILP" << std::endl;
    String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );
    File summaryFile( summaryFilePath );
    summaryFile.open( File::MODE_WRITE_TRUNCATE );
    if ( engine->getExitCode() == Engine::UNSAT )
        summaryFile.write( "holds\n" );
    else if ( engine->getExitCode() == Engine::SAT )
        summaryFile.write( "violated\n" );
    mtx.unlock();
    *(engine->_done) = true;
}

void DnCMarabou::solveSingleThread( DnCArgument argument )
{
    Engine *engine = argument._engine;
    InputQuery *inputQuery = argument._inputQuery;
    std::mutex &mtx = *(argument._mtx);
    engine->_numWorkers = 1;
    engine->setVerbosity(0);
    printf( "Single thread starting... \n" );
    engine->solve(0);
    if ( engine->getExitCode() == Engine::SAT )
        engine->extractSolution( *inputQuery );

    mtx.lock();
    Engine::ExitCode result = engine->getExitCode();

    if ( result == Engine::UNSAT )
    {
        printf( "unsat\n" );
    }
    else if ( result == Engine::SAT )
    {
        if ( inputQuery->_networkLevelReasoner && !engine->_solutionFoundAndStoredInOriginalQuery )
        {
            double *input = new double[inputQuery->getNumInputVariables()];
            for ( unsigned i = 0; i < inputQuery->getNumInputVariables(); ++i )
                input[i] = inputQuery->getSolutionValue( inputQuery->inputVariableByIndex( i ) );

            NLR::NetworkLevelReasoner *nlr = inputQuery->_networkLevelReasoner;
            NLR::Layer *lastLayer = nlr->getLayer( nlr->getNumberOfLayers() - 1 );
            double *output = new double[lastLayer->getSize()];

            nlr->evaluate( input, output );

            printf( "\n" );
            printf( "Output:\n" );
            for ( unsigned i = 0; i < lastLayer->getSize(); ++i )
                printf( "y%u = %lf\n", i, output[i] );
            printf( "\n" );

            printf( "sat\n" );
        }
        else
        {
            printf( "sat\n" );
        }
    }
    else if ( result == Engine::TIMEOUT )
    {
        printf( "Timeout\n" );
    }
    else if ( result == Engine::ERROR )
    {
        printf( "Error\n" );
    }
    else
    {
        printf( "UNKNOWN EXIT CODE! (this should not happen)" );
    }

    std::cout << "Solved by singled thread soi" << std::endl;
    String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );
    File summaryFile( summaryFilePath );
    summaryFile.open( File::MODE_WRITE_TRUNCATE );
    if ( engine->getExitCode() == Engine::UNSAT )
        summaryFile.write( "holds\n" );
    else if ( engine->getExitCode() == Engine::SAT )
        summaryFile.write( "violated\n" );
    mtx.unlock();
    *(engine->_done) = true;
}

void DnCMarabou::displayResults( unsigned long long microSecondsElapsed ) const
{
    std::cout << "Runtime is " << microSecondsElapsed / 1000000 << std::endl;
}

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

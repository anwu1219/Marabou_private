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

    struct timespec start = TimeUtils::sampleMicro();

    boost::thread *threads = new boost::thread[3];
    _dncManager = std::unique_ptr<DnCManager>
        ( new DnCManager( &_inputQuery ) );
    std::atomic_bool done (false);
    _dncManager->setDone( &done );
    _engine1.setDone( &done );
    _engine2.setDone( &done );

    std::unique_ptr<InputQuery> newInputQuery1 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );
    std::unique_ptr<InputQuery> newInputQuery2 =
        std::unique_ptr<InputQuery>( new InputQuery( _inputQuery ) );

    std::mutex mtx;

    threads[0] = boost::thread( solveDnC, DnCArgument( &(*_dncManager), &mtx ) );
    threads[1] = boost::thread( solveMILP, DnCArgument( &_engine1, &(*newInputQuery1), &mtx ) );
    threads[2] = boost::thread( solveSingleThread, DnCArgument( &_engine2, &(*newInputQuery2), &mtx ) );
    
    boost::chrono::milliseconds waitTime( 100 );
    while ( !done.load() )
        boost::this_thread::sleep_for( waitTime );

    if ( done.load() )
    {
        struct timespec end = TimeUtils::sampleMicro();

        unsigned long long totalElapsed = TimeUtils::timePassed( start, end );
        displayResults( totalElapsed );

        for ( unsigned i = 0; i < 3; ++i )
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
    dncManager->solve();
    mtx.lock();
    dncManager->printResult();
    std::cout << "Solved by DnC" << std::endl;
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

    engine->_numWorkers = 23;
    engine->setVerbosity(0);
    if ( engine->processInputQuery( *inputQuery ) )
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

    engine->setVerbosity(0);
    engine->_numWorkers = 1;
    if ( engine->processInputQuery( *inputQuery ) )
        engine->solve();
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

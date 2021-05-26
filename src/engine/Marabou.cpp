/*********************                                                        */
/*! \file Marabou.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz
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

#include "AcasParser.h"
#include "Equation.h"
#include "GlobalConfiguration.h"
#include "File.h"
#include "FloatUtils.h"
#include "MStringf.h"
#include "Marabou.h"
#include "Options.h"
#include "PropertyParser.h"
#include "MarabouError.h"
#include "QueryLoader.h"

#ifdef _WIN32
#undef ERROR
#endif

Marabou::Marabou()
    : _inputQuery( NULL )
    , _acasParser( NULL )
    , _engine( std::unique_ptr<Engine>(new Engine()) )
{
}

Marabou::~Marabou()
{
    if ( _acasParser )
    {
        delete _acasParser;
        _acasParser = NULL;
    }
}

void Marabou::run()
{
    String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );    
    if ( File::exists( summaryFilePath ) )
    {
	std::cout << "File exists!" << std::endl;
	return;
    }

    struct timespec start = TimeUtils::sampleMicro();
	
    prepareInputQuery();
    solveQuery();

    struct timespec end = TimeUtils::sampleMicro();

    unsigned long long totalElapsed = TimeUtils::timePassed( start, end );
    displayResults( totalElapsed );
}

void Marabou::prepareInputQuery()
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
        std::cout << "loading query" << std::endl;
        _inputQuery = QueryLoader::loadQuery( inputQueryFilePath );
    }
    else
    {
        _inputQuery = new InputQuery();
        /*
          Step 1: extract the network
        */
        String networkFilePath = Options::get()->getString( Options::INPUT_FILE_PATH );
        if ( !File::exists( networkFilePath ) )
        {
            printf( "Error: the specified network file (%s) doesn't exist!\n", networkFilePath.ascii() );
            throw MarabouError( MarabouError::FILE_DOESNT_EXIST, networkFilePath.ascii() );
        }
        printf( "Network: %s\n", networkFilePath.ascii() );

        // For now, assume the network is given in ACAS format
        _acasParser = new AcasParser( networkFilePath );
        _acasParser->generateQuery( *_inputQuery );
        _inputQuery->constructNetworkLevelReasoner();

        /*
          Step 2: extract the property in question
        */
        String propertyFilePath = Options::get()->getString( Options::PROPERTY_FILE_PATH );
        if ( propertyFilePath != "" )
        {
            printf( "Property: %s\n", propertyFilePath.ascii() );
            PropertyParser().parse( propertyFilePath, *_inputQuery );
        }
        else
            printf( "Property: None\n" );

        printf( "\n" );
    }

    String queryDumpFilePath = Options::get()->getString( Options::QUERY_DUMP_FILE );
    if ( queryDumpFilePath.length() > 0 )
    {
        _inputQuery->saveQuery( queryDumpFilePath );
        printf( "\nInput query successfully dumped to file\n" );
        exit( 0 );
    }
}

void Marabou::solveQuery()
{
    int correct = Options::get()->getInt( Options::CORRECT_OUTPUT );
    if ( correct == -1)
    {
        if ( _engine->processInputQuery( *_inputQuery ) )
        {
            _engine->solve( Options::get()->getInt( Options::TIMEOUT ) );
        }
        if ( _engine->getExitCode() == Engine::SAT )
            _engine->extractSolution( *_inputQuery );
    }
    else
    {
        for ( int target = 0; target < 10; ++target)
        {
            if ( target == correct )
                continue;
            _engine = std::unique_ptr<Engine>( new Engine());
            InputQuery tmpInputQuery = *_inputQuery;
            for ( unsigned other= 0; other < 10; ++other )
            {
                Equation eq(Equation::GE);
                eq.addAddend( 1, tmpInputQuery.getOutputVariableByIndex(target) );
                eq.addAddend( -1, tmpInputQuery.getOutputVariableByIndex(other) );
                tmpInputQuery.addEquation(eq);
            }

            if ( _engine->processInputQuery( tmpInputQuery ) )
            {
                _engine->solve( Options::get()->getInt( Options::TIMEOUT ) );
            }
            if ( _engine->getExitCode() == Engine::SAT )
            {
                _engine->extractSolution( *_inputQuery );
                return;
            }
        }
    }
}

void Marabou::displayResults( unsigned long long microSecondsElapsed ) const
{
    Engine::ExitCode result = _engine->getExitCode();
    String resultString;

    if ( result == Engine::UNSAT )
    {
        resultString = "unsat";
        printf( "unsat\n" );
    }
    else if ( result == Engine::SAT )
    {
        resultString = "sat";
        printf( "Input assignment:\n" );
        for ( unsigned i = 0; i < _inputQuery->getNumInputVariables(); ++i )
            printf( "x%u = %lf\n", i, _inputQuery->getSolutionValue( _inputQuery->inputVariableByIndex( i ) ) );

        if ( _inputQuery->_networkLevelReasoner )
        {
            double *input = new double[_inputQuery->getNumInputVariables()];
            for ( unsigned i = 0; i < _inputQuery->getNumInputVariables(); ++i )
                input[i] = _inputQuery->getSolutionValue( _inputQuery->inputVariableByIndex( i ) );

            NLR::NetworkLevelReasoner *nlr = _inputQuery->_networkLevelReasoner;
            NLR::Layer *lastLayer = nlr->getLayer( nlr->getNumberOfLayers() - 1 );
            double *output = new double[lastLayer->getSize()];

            nlr->evaluate( input, output );

            printf( "\n" );
            printf( "Output:\n" );
            for ( unsigned i = 0; i < lastLayer->getSize(); ++i )
                printf( "y%u = %lf\n", i, output[i] );
            printf( "\n" );

            bool sat = true;
            int maxOutput = Options::get()->getInt( Options::MAX_OUTPUT );
            if ( maxOutput != -1 )
            {
                for ( unsigned i = 0; i < lastLayer->getSize(); ++i )
                {
                    if ( FloatUtils::gt( output[i],  output[maxOutput] ) )
                    {
                        sat = false;
                        break;
                    }
                }
            }
            delete[] input;
            delete[] output;

            if ( sat )
                printf( "sat\n" );
        }
        else
        {
            printf( "\n" );
            printf( "Output:\n" );
            for ( unsigned i = 0; i < _inputQuery->getNumOutputVariables(); ++i )
                printf( "y%u = %lf\n", i, _inputQuery->getSolutionValue( _inputQuery->outputVariableByIndex( i ) ) );
            printf( "\n" );
        }
        printf( "sat\n" );
    }
    else if ( result == Engine::TIMEOUT )
    {
        resultString = "TIMEOUT";
        printf( "Timeout\n" );
    }
    else if ( result == Engine::ERROR )
    {
        resultString = "ERROR";
        printf( "Error\n" );
    }
    else
    {
        resultString = "UNKNOWN";
        printf( "UNKNOWN EXIT CODE! (this should not happen)" );
    }

    // Create a summary file, if requested
    String summaryFilePath = Options::get()->getString( Options::SUMMARY_FILE );
    if ( summaryFilePath != "" )
    {
        File summaryFile( summaryFilePath );
        summaryFile.open( File::MODE_WRITE_TRUNCATE );

        // Field #1: result
        summaryFile.write( resultString );

        // Field #2: total elapsed time
        summaryFile.write( Stringf( " %u ", microSecondsElapsed / 1000000 ) ); // In seconds

        // Field #3: number of visited tree states
        summaryFile.write( Stringf( "%u ",
                                    _engine->getStatistics()->getUnsignedAttr( Statistics::NUM_VISITED_TREE_STATES ) ) );

        // Field #4: number of flips
        summaryFile.write( Stringf( "%llu ",
                                    _engine->getStatistics()->getLongAttr( Statistics::NUM_PROPOSED_FLIPS ) ) );

        // Field #5: number of accepted flips
        summaryFile.write( Stringf( "%llu ",
                                    _engine->getStatistics()->getLongAttr( Statistics::NUM_ACCEPTED_FLIPS ) ) );

        // Field #6: number of rejected flips
        summaryFile.write( Stringf( "%llu",
                                    _engine->getStatistics()->getLongAttr( Statistics::NUM_REJECTED_FLIPS ) ) );

        summaryFile.write( "\n" );
        if ( resultString == "sat" )
        {
            for ( unsigned i = 0; i < _inputQuery->getNumberOfVariables(); ++i )
                summaryFile.write( Stringf( "x%u = %lf\n", i, _inputQuery->getSolutionValue( i ) ) );
        }
    }
}

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

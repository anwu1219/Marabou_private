/*********************                                                        */
/*! \file SmtCore.cpp
 ** \verbatim
 ** Top contributors (to current version):
 **   Guy Katz, Parth Shah, Duligur Ibeling
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

 **/

#include "Debug.h"
#include "DivideStrategy.h"
#include "FloatUtils.h"
#include "GlobalConfiguration.h"
#include "IEngine.h"
#include "MStringf.h"
#include "MarabouError.h"
#include "Options.h"
#include "ReluConstraint.h"
#include "SmtCore.h"

using namespace CVC4::context;

SmtCore::SmtCore( IEngine *engine, Context &ctx )
    : _context( ctx )
    , _statistics( NULL )
    , _engine( engine )
    , _needToSplit( false )
    , _constraintForSplitting( NULL )
    , _constraintViolationThreshold( Options::get()->getInt( Options::CONSTRAINT_VIOLATION_THRESHOLD ) )
    , _localSearch( Options::get()->getBool( Options::LOCAL_SEARCH ) )
    , _numberOfRandomFlips( 0 )
{
}

SmtCore::~SmtCore()
{
    freeMemory();
}

void SmtCore::freeMemory()
{
    for ( const auto &stackEntry : _stack )
    {
        delete stackEntry;
    }

    _stack.clear();
}

void SmtCore::reset()
{
    freeMemory();
    _needToSplit = false;
    _constraintForSplitting = NULL;
    _numberOfRandomFlips = 0;
}

void SmtCore::reportRandomFlip()
{
    struct timespec start = TimeUtils::sampleMicro();

    if ( _localSearch )
        return;
    if ( _numberOfRandomFlips++ >= _constraintViolationThreshold )
    {
        _needToSplit = true;
        pickSplitPLConstraint();
    }

    if ( _statistics )
    {
        struct timespec end = TimeUtils::sampleMicro();
        _statistics->incLongAttr( Statistics::TIME_BRANCHING_HEURISTICS_MICRO,
                                  TimeUtils::timePassed( start, end ) );
    }

}

void SmtCore::performSplit()
{
    struct timespec start = TimeUtils::sampleMicro();

    SMT_LOG( Stringf( "Performing a case split @ level %u", _context.getLevel() ).ascii() );

    ASSERT( _needToSplit );
    ASSERT( _constraintForSplitting->isActive() );

    resetReportedViolations();

    if ( _statistics )
    {
        _statistics->incUnsignedAttr( Statistics::NUM_VISITED_TREE_STATES, 1 );
    }

    _constraintForSplitting->setActiveConstraint( false );

    _engine->pushContext();
    SMT_LOG( Stringf( "Pushed context. Current level: %u", _context.getLevel() ).ascii() );

    // Before storing the state of the engine, we:
    //   1. Obtain the splits.
    //   2. Disable the constraint, so that it is marked as disbaled in the EngineState.
    List<PiecewiseLinearCaseSplit> splits = _constraintForSplitting->getCaseSplits();
    ASSERT( !splits.empty() );
    ASSERT( splits.size() >= 2 ); // Not really necessary, can add code to handle this case.

    SmtStackEntry *stackEntry = new SmtStackEntry;
    // Perform the first split: add bounds and equations
    List<PiecewiseLinearCaseSplit>::iterator split = splits.begin();

    _engine->applySplit( *split );
    stackEntry->_activeSplit = *split;

    ++split;
    while ( split != splits.end() )
    {
        stackEntry->_alternativeSplits.append( *split );
        ++split;
    }

    _stack.append( stackEntry );

    _constraintForSplitting = NULL;

    if ( _statistics )
    {
        _statistics->setUnsignedAttr( Statistics::CURRENT_STACK_DEPTH, getStackDepth() );
        struct timespec end = TimeUtils::sampleMicro();
        _statistics->incLongAttr( Statistics::TIME_SMT_CORE_PUSH_MICRO,
                                  TimeUtils::timePassed( start, end ) );
    }
}

bool SmtCore::popSplit()
{
    struct timespec start = TimeUtils::sampleMicro();

    SMT_LOG( Stringf( "Backtracking @ level %u", _context.getLevel() ).ascii() );

    if ( _stack.empty() )
    {
        ASSERT( _context.getLevel() == 0 );
        return false;
    }

    resetReportedViolations();

    if ( _statistics )
    {
        _statistics->incUnsignedAttr( Statistics::NUM_VISITED_TREE_STATES, 1 );
    }

    // Remove any entries that have no alternatives
    String error;
    while ( _stack.back()->_alternativeSplits.empty() )
    {
        delete _stack.back();
        _stack.popBack();
        _context.pop();
        SMT_LOG( Stringf( "Popped context. Current level: %u", _context.getLevel() ).ascii() );

        if ( _stack.empty() )
        {
            if ( _statistics )
            {
                _statistics->setUnsignedAttr( Statistics::CURRENT_STACK_DEPTH, getStackDepth() );
                struct timespec end = TimeUtils::sampleMicro();
                _statistics->incLongAttr( Statistics::TIME_SMT_CORE_POP_MICRO,
                                          TimeUtils::timePassed( start, end ) );
            }
            return false;
        }
    }

    _engine->popContext();
    SMT_LOG( Stringf( "Popped context. Current level: %u", _context.getLevel() ).ascii() );
    SmtStackEntry *stackEntry = _stack.back();

    // Apply the new split and erase it from the list
    auto split = stackEntry->_alternativeSplits.begin();

    _engine->pushContext();
    SMT_LOG( Stringf( "Pushed context. Current level: %u", _context.getLevel() ).ascii() );
    SMT_LOG( "\tApplying new split..." );
    _engine->applySplit( *split );
    SMT_LOG( "\tApplying new split - DONE" );

    stackEntry->_activeSplit = *split;
    stackEntry->_alternativeSplits.erase( split );

    if ( _statistics )
    {
        _statistics->setUnsignedAttr( Statistics::CURRENT_STACK_DEPTH, getStackDepth() );
        struct timespec end = TimeUtils::sampleMicro();
        _statistics->incLongAttr( Statistics::TIME_SMT_CORE_POP_MICRO,
                                  TimeUtils::timePassed( start, end ) );
    }

    return true;
}

void SmtCore::resetReportedViolations()
{
    _numberOfRandomFlips = 0;
    _needToSplit = false;
}

void SmtCore::setStatistics( Statistics *statistics )
{
    _statistics = statistics;
}

bool SmtCore::pickSplitPLConstraint()
{
    if ( _needToSplit )
        _constraintForSplitting = _engine->pickSplitPLConstraint();
    return _constraintForSplitting != NULL;
}

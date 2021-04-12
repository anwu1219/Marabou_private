/*********************                                                        */
/*! \file PseudoCostTracker.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Haoze Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** [[ Add lengthier description here ]]

**/

#ifndef __PseudoCostTracker_h__
#define __PseudoCostTracker_h__

#include "List.h"
#include "MStringf.h"
#include "PiecewiseLinearConstraint.h"
#include "Statistics.h"

#include <memory>
#include <set>
#include <random>

#define COST_TRACKER_LOG(x, ...) LOG(GlobalConfiguration::PSEUDO_COST_TRACKER_LOGGING, "PseudoCostTracker: %s\n", x)

struct ScoreEntry
{
    ScoreEntry( PiecewiseLinearConstraint *constraint, double score )
        : _constraint( constraint )
        , _score( score )
    {};

    bool operator<(const ScoreEntry& other ) const
    {
        if ( _score == other._score )
            return _constraint > other._constraint;
        else
            return _score > other._score;
    }

    PiecewiseLinearConstraint *_constraint;
    double _score;
};

typedef std::set<ScoreEntry> Scores;

class PseudoCostTracker
{
public:

    PseudoCostTracker();

    void initialize( List<PiecewiseLinearConstraint *> &plConstraints );

    void updateScore( PiecewiseLinearConstraint *constraint, double score );

    /*
      Return the unfixed PLConstraint with the largest estimated reduced cost
    */
    inline PiecewiseLinearConstraint *top()
    {
        return _scores.begin()->_constraint;
    }

    PiecewiseLinearConstraint *topUnfixed()
    {
        for ( const auto &entry : _scores )
            if ( entry._constraint->isActive() && !entry._constraint->phaseFixed() )
            {
                COST_TRACKER_LOG( Stringf( "Score of top unfixed plConstraint: %.2f",
                                           entry._score ).ascii() );
                return entry._constraint;
            }
        ASSERT( false );
        return NULL;
    }

private:

    friend class PseudoCostTrackerTestSuite;

    Scores _scores;
    Map<PiecewiseLinearConstraint *, double> _plConstraintToScore;
};

#endif // __PseudoCostTracker_h__

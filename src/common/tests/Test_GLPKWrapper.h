/*********************                                                        */
/*! \file Test_GLPKWrapper.h
 ** \verbatim
 ** Top contributors (to current version):
 **   Haoze Wu
 ** This file is part of the Marabou project.
 ** Copyright (c) 2017-2019 by the authors gurobiWrappered in the file AUTHORS
 ** in the top-level source directory) and their institutional affiliations.
 ** All rights reserved. See the file COPYING in the top-level source
 ** directory for licensing information.\endverbatim
 **
 ** \brief [[ Add one-line brief description here ]]
 **
 ** [[ Add lengthier description here ]]
 **/

#include <cxxtest/TestSuite.h>

#include "FloatUtils.h"
#include "GLPKWrapper.h"
#include "MString.h"
#include "MockErrno.h"

class GLPKWrapperTestSuite : public CxxTest::TestSuite
{
public:
    MockErrno *mockErrno;

    void setUp()
    {
        TS_ASSERT( mockErrno = new MockErrno );
    }

    void tearDown()
    {
        TS_ASSERT_THROWS_NOTHING( delete mockErrno );
    }

    void test_optimize()
    {
#ifdef ENABLE_GLPK
        GLPKWrapper glpk( NULL );

        glpk.addVariable( "x", 0, 3 );
        glpk.addVariable( "y", 0, 3 );
        glpk.addVariable( "z", 0, 3 );

        // x + y + z <= 5
        List<LPSolver::Term> contraint = {
            GLPKWrapper::Term( 1, "x" ),
            GLPKWrapper::Term( 1, "y" ),
            GLPKWrapper::Term( 1, "z" ),
        };

        glpk.addLeqConstraint( contraint, 5 );

        // Cost: -x - 2y + z
        List<LPSolver::Term> cost = {
            GLPKWrapper::Term( -1, "x" ),
            GLPKWrapper::Term( -2, "y" ),
            GLPKWrapper::Term( +1, "z" ),
        };

        glpk.setCost( cost );

        // Solve and extract
        TS_ASSERT_THROWS_NOTHING( glpk.solve() );

        Map<String, double> solution;
        double costValue;

        TS_ASSERT_THROWS_NOTHING( glpk.extractSolution( solution, costValue ) );

        TS_ASSERT( FloatUtils::areEqual( solution["x"], 2 ) );
        TS_ASSERT( FloatUtils::areEqual( solution["y"], 3 ) );
        TS_ASSERT( FloatUtils::areEqual( solution["z"], 0 ) );

        TS_ASSERT( FloatUtils::areEqual( costValue, -8 ) );

#else
        TS_ASSERT( true );
#endif // ENABLE_GLPK
    }

    void test_incremental()
    {
#ifdef ENABLE_GLPK
        GLPKWrapper glpk( NULL );

        glpk.addVariable( "x", 2, 4 );
        glpk.addVariable( "y", 3, 5 );

        // x + y <= 5
        List<LPSolver::Term> contraint = {
            GLPKWrapper::Term( 1, "x" ),
            GLPKWrapper::Term( 1, "y" )
        };

        glpk.addLeqConstraint( contraint, 5 );

        // Solve and extract
        TS_ASSERT_THROWS_NOTHING( glpk.solve() );

        TS_ASSERT( glpk.haveFeasibleSolution() );
        TS_ASSERT( glpk.optimal() );
        TS_ASSERT( !glpk.infeasible() );

        Map<String, double> solution;
        double costValue;

        TS_ASSERT_THROWS_NOTHING( glpk.extractSolution( solution, costValue ) );

        TS_ASSERT( FloatUtils::areEqual( solution["x"], 2 ) );
        TS_ASSERT( FloatUtils::areEqual( solution["y"], 3 ) );

        TS_ASSERT_THROWS_NOTHING( glpk.setLowerBound( "y", 4 ) );

        TS_ASSERT_THROWS_NOTHING( glpk.solve() );

        TS_ASSERT( glpk.infeasible() );

        TS_ASSERT_THROWS_NOTHING( glpk.setLowerBound( "y", 2 ) );

        TS_ASSERT( !glpk.infeasible() );
        TS_ASSERT( !glpk.haveFeasibleSolution() );

        TS_ASSERT_THROWS_NOTHING( glpk.solve() );

        TS_ASSERT( glpk.haveFeasibleSolution() );
        TS_ASSERT( glpk.optimal() );
        TS_ASSERT( !glpk.infeasible() );

#else
        TS_ASSERT( true );
#endif // ENABLE_GLPK
    }
};

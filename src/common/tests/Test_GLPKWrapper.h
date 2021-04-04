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
        GLPKWrapper gurobi;

        gurobi.addVariable( "x", 0, 3 );
        gurobi.addVariable( "y", 0, 3 );
        gurobi.addVariable( "z", 0, 3 );

        // x + y + z <= 5
        List<LPSolver::Term> contraint = {
            GLPKWrapper::Term( 1, "x" ),
            GLPKWrapper::Term( 1, "y" ),
            GLPKWrapper::Term( 1, "z" ),
        };

        gurobi.addLeqConstraint( contraint, 5 );

        // Cost: -x - 2y + z
        List<LPSolver::Term> cost = {
            GLPKWrapper::Term( -1, "x" ),
            GLPKWrapper::Term( -2, "y" ),
            GLPKWrapper::Term( +1, "z" ),
        };

        gurobi.setCost( cost );

        // Solve and extract
        TS_ASSERT_THROWS_NOTHING( gurobi.solve() );

        Map<String, double> solution;
        double costValue;

        TS_ASSERT_THROWS_NOTHING( gurobi.extractSolution( solution, costValue ) );

        TS_ASSERT( FloatUtils::areEqual( solution["x"], 2 ) );
        TS_ASSERT( FloatUtils::areEqual( solution["y"], 3 ) );
        TS_ASSERT( FloatUtils::areEqual( solution["z"], 0 ) );

        TS_ASSERT( FloatUtils::areEqual( costValue, -8 ) );

#else
        TS_ASSERT( true );
#endif // ENABLE_GUROBI
    }

    void test_incremental()
    {
#ifdef ENABLE_GUROBI
        GLPKWrapper gurobi;

        gurobi.addVariable( "x", 2, 4 );
        gurobi.addVariable( "y", 3, 5 );

        // x + y <= 5
        List<LPSolver::Term> contraint = {
            GLPKWrapper::Term( 1, "x" ),
            GLPKWrapper::Term( 1, "y" )
        };

        gurobi.addLeqConstraint( contraint, 5 );

        // Solve and extract
        TS_ASSERT_THROWS_NOTHING( gurobi.solve() );

        TS_ASSERT( gurobi.haveFeasibleSolution() );

        TS_ASSERT_THROWS_NOTHING( gurobi.setLowerBound( "y", 4 ) );

        TS_ASSERT_THROWS_NOTHING( gurobi.updateModel() );

        TS_ASSERT( !gurobi.haveFeasibleSolution() );

        TS_ASSERT_THROWS_NOTHING( gurobi.solve() );

        TS_ASSERT( gurobi.infeasible() );

        TS_ASSERT_THROWS_NOTHING( gurobi.setLowerBound( "y", 2 ) );
        TS_ASSERT_THROWS_NOTHING( gurobi.updateModel() );

        TS_ASSERT( !gurobi.infeasible() );
        TS_ASSERT( !gurobi.haveFeasibleSolution() );

        TS_ASSERT_THROWS_NOTHING( gurobi.solve() );

        TS_ASSERT( gurobi.haveFeasibleSolution() );

#else
        TS_ASSERT( true );
#endif // ENABLE_GLPK
    }
};

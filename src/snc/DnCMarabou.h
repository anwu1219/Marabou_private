/*********************                                                        */
/*! \file DnCMarabou.h
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

#ifndef __DnCMarabou_h__
#define __DnCMarabou_h__

#include "DnCManager.h"
#include "Options.h"
#include "InputQuery.h"

#include <atomic>
#include <boost/chrono.hpp>
#include <mutex>


class DnCMarabou
{
public:
    struct DnCArgument{
        DnCArgument( Engine *engine, InputQuery *inputQuery, std::mutex *mtx, unsigned numDisj )
            : _mtx( mtx )
        {
            _engine = engine;
            _inputQuery = inputQuery;
            _numDisj = numDisj;
        }

        DnCArgument( DnCManager *dncManager, std::mutex *mtx, unsigned id, unsigned numDisj )
            : _mtx( mtx )
        {
            _dncManager = dncManager;
            _id = id;
            _numDisj = numDisj;
        }

        DnCManager *_dncManager = NULL;
        Engine *_engine;
        InputQuery *_inputQuery;
        std::mutex *_mtx;
        unsigned _id = 0;
        unsigned _numDisj = 0;
    };

    DnCMarabou();

    /*
      Entry point of this class
    */
    void run();

private:
    std::unique_ptr<DnCManager> _dncManager;
    InputQuery _inputQuery;
    Engine _engine1;
    Engine _engine2;

    /*
      Display the results
    */
    void displayResults( unsigned long long microSecondsElapsed ) const;

    static void solveDnC( DnCArgument argument );

    static void solveMILP( DnCArgument argument );

    static void solveSingleThread( DnCArgument argument );
};

#endif // __DnCMarabou_h__

//
// Local Variables:
// compile-command: "make -C ../.. "
// tags-file-name: "../../TAGS"
// c-basic-offset: 4
// End:
//

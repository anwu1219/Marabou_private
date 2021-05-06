#!/bin/bash
curdir=$pwd
mydir=$(realpath "${0%/*}")

cd $mydir

if [ ! -f glpk-5.0.tar.gz ]
then
    echo "Downloding GLPK"
    wget -q https://ftp.gnu.org/gnu/glpk/glpk-5.0.tar.gz -O glpk-5.0.tar.gz
fi

if [ ! -d glpk-5.0 ]
then
    echo "Unzipping GLPK"
    tar -xzf glpk-5.0.tar.gz >> /dev/null
fi

echo "Installing GLPK"
cd glpk-5.0
./configure
make -j4
make prefix=$mydir/glpk-5.0/installed install
cd $curdir

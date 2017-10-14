#!/bin/bash

# Let script run from top level or scripts folder
SCRIPT=`realpath $0`
SCRIPT=`realpath $0`
SCRIPTPATH=`dirname $SCRIPT`

if [ $SCRIPTPATH == `pwd` ]; then
  cd ..
fi

`pwd`/devel/setup.bash
catkin_make
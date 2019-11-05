#!/bin/sh
for i in `find . -name '*.wrl'`; do
    meshlabserver -i $i -o `pwd`/$(echo `basename $i` | sed 's|.wrl$|.dae|')
done

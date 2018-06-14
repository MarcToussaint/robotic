#/bin/sh

callpath=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

export PYTHONPATH="$PYTHONPATH:$callpath/src"

echo "new PYTHONPATH = " $PYTHONPATH

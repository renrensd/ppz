export ARTISTIC_STYLE_OPTIONS=.astylerc

SOURCES_PATH='./sw/airborne'

find $SOURCES_PATH -name '*.c' -exec astyle {} \;
find $SOURCES_PATH -name '*.h' -exec astyle {} \;

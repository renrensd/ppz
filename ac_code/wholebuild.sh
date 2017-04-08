export PAPARAZZI_SRC=$PWD
export PAPARAZZI_HOME=$PWD
#echo $PAPARAZZI_SRC
make clean
make
make -f Makefile.ac AIRCRAFT=krooz_quad clean_ac
make -f Makefile.ac AIRCRAFT=krooz_quad ap.compile

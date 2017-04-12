export PAPARAZZI_SRC=$PWD
export PAPARAZZI_HOME=$PWD

filename=$1

${PAPARAZZI_HOME}/sw/logalizer/sd2log ${PAPARAZZI_HOME}/var/logs/${filename}

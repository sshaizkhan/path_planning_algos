#!/bin/bash -e

YELLOW='\033[1;33m'
GREEN='\033[1;32m'
RED='\033[1;91m'
RESETCOLOUR='\033[0m'

# Please note: workers 4 was selected for better stability when
# hitting gitlab.dorabot.com from Australia.

echo -e "${YELLOW}\nInstalling Python deps... \n${RESETCOLOUR}"

cd ~/path_planner_ws/src/path_planning_algos
pip3 install -r ./requirements.txt --user

result_install_python_deps=$?

echo -e "${YELLOW}\nSummary:\n${RESETCOLOUR}"

function print_result()
{
  printf "${RESETCOLOUR}" "\n"
  if [ $2 -eq 0 ]; then
    printf "${GREEN}%22s" "OK"
  else
    printf "${RED}%22s" "ERROR"
  fi
  echo -e "${RESETCOLOUR}\r${1}"
}

print_result "Install Python deps:" $result_install_python_deps


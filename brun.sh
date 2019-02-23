#!/bin/sh

printf "**************************************************\n"
printf "*                                                *\n"
printf "*                                                *\n"
printf "*              NEW    RUN                        *\n"
printf "*                                                *\n"
printf "*                                                *\n"
printf "**************************************************\n"


cd build
rm path_planning
time make
./path_planning

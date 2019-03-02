#!/bin/sh
printf "**************************************************\n"
printf "*                                                *\n"
printf "*                                                *\n"
printf "*              NEW    RUN                        *\n"
printf "*                                                *\n"
printf "*                                                *\n"
printf "**************************************************\n"


rm -rf build/*
cd build
cmake ..

date | tee make_time.txt
make | tee make_output.txt
date | tee -a make_time.txt

mv make_time.txt ../
mv make_output.txt ../
cd ../

echo "- - - - - - - - - - - - - - - - - - - -" > output.txt
echo "- - - - START     - - - - - - - - - - -" >> output.txt
date | tee -a output.txt
echo "- - - - - - - - - - - - - - - - - - - -" >> output.txt
printf "\n\n\n\n" >> output.txt

./build/path_planning | tee -a output.txt

printf "\n\n\n\n" >> output.txt
echo "- - - - - - - - - - - - - - - - - - - -" >> output.txt
echo "- - - - END   - - - - - - - - - - - - -" >> output.txt
date | tee -a output.txt
echo "- - - - - - - - - - - - - - - - - - - -" >> output.txt

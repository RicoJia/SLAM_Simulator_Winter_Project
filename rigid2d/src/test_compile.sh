#!/bin/sh
g++ -o ./test.o ./test1.cpp ./rigid2d.cpp
#./test.o

if [ -a ./test1_input.txt ];
then
  echo "test1_input.txt is found!"
#  ./test.o<./test1_input.txt 2>&1
  if ./test.o<./test1_input.txt | cmp ./test1_answer.txt;
  then
    echo "Success!"
  else
    echo "Failure!"
  fi
else
  echo "test1_input.txt doesn't exist"
fi

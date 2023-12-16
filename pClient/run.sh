#!/bin/bash

challenge="4"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:f2:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 2
        # activate virtual environment, if needed
        python3 mainC2.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv output.txt $outfile.map             # if needed
        mv output2.txt $outfile.path 
        ;;
esac
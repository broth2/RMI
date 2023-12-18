#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
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
        # how to call agent for challenge 1
	# activate virtual environment, if needed
        python3 mainC1.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 mainC2.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv output.txt $outfile.map             # if needed
        ;;
    3)
        # how to call agent for challenge 3
        python3 mainC3.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv output2.txt $outfile.path           # if needed
        ;;
    4)
        rm -f *.path *.map  # do not remove this line

        # how to call agent for challenge 4
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv output.txt $outfile.map
        mv output2.txt $outfile.path
        ;;
esac


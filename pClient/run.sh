#!/bin/bash

challenge="4"
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
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

case $challenge in
    4)
        # how to call agent for challenge 2
        # activate virtual environment, if needed
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv output.txt "$outfile.map"
        mv output2.txt "$outfile.path"
        ;;
esac

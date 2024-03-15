#!/usr/bin/bash

split -l 1 new_version_trajectory.txt
 i=1
 for x in `ls x* | sort`
 do
     mv $x $i.txt
    i=$(($i+1))
done

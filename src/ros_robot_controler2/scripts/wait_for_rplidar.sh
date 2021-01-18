#!/bin/bash

scan_topic = "/scan";

until [ "$started" ]; do

    MULTILINE=$(rostopic list);
    for line in $MULTILINE
    do
        if [ "$line" == "$scan_topic" ]; then
            started=1
        fi
    done

done

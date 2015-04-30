#!/bin/sh

while read posrow <&3; do
    read
    echo $posrow
done 3< $1

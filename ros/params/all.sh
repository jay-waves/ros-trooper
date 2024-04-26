#!/bin/bash

# Define the directories
dir1="controller"
dir2="planner"
dir3="smoother"

for file1 in "$dir1"/*; do
    for file2 in "$dir2"/*; do
        for file3 in "$dir3"/*; do
          fn="$(basename $file1).$(basename $file2).$(basename $file3).yaml"
            cat "params.yaml" "$file1" "$file2" "$file3" > "all/$fn"
        done
    done
done


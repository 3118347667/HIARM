#!/bin/bash

# Check if the root directory is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <root_directory>"
    exit 1
fi

root_directory="$1"
search_word="charm"
replace_word="flygripper"

# Find all files and directories with the word "charm" in the name
find "$root_directory" -depth -name "*$search_word*" | while IFS= read -r path; do
    # Extract the directory and filename
    dir=$(dirname "$path")
    filename=$(basename "$path")

    # Replace "charm" with "flygripper"
    new_filename="${filename//$search_word/$replace_word}"

    # Construct the new path
    new_path="$dir/$new_filename"

    # Rename the file or directory
    mv "$path" "$new_path"

    echo "Renamed: $path -> $new_path"
done

echo "Renaming complete."
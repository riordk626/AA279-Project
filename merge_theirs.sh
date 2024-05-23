#!/bin/bash

# Start the merge
echo "Starting the merge..."
git merge main

# Check if there were conflicts
merge_status=$?
echo "Merge status: $merge_status"

if [ $merge_status -ne 0 ]; then
    # Get the list of conflicted files
    conflicted_files=$(git diff --name-only --diff-filter=U)
    echo "Conflicted files:"
    echo "$conflicted_files"

    if [ -z "$conflicted_files" ]; then
        echo "No conflicted files found."
        exit 0
    fi

    # Resolve each conflict by checking out 'theirs'
    echo "$conflicted_files" | while IFS= read -r file; do
        if [ -n "$file" ]; then
            echo "Checking out theirs for: $file"
            git checkout --theirs "$file"
            git add "$file"
        else
            echo "Empty file path detected."
        fi
    done

    # Commit the merge
    echo "Committing the merge..."
    git commit -m "Merge master and accept theirs in conflicts"
else
    echo "Merge completed successfully without conflicts."
fi


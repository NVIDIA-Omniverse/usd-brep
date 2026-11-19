#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Wrapper script to run brep_array_handler on all USD files in a directory
# and summarize output messages by type for each file.

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if directory argument is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 <directory_path>"
    echo ""
    echo "This script runs brep_array_handler on all USD files (.usd, .usda, .usdc) in the specified directory"
    echo "and summarizes validation messages by type for each file."
    exit 1
fi

TARGET_DIR="$1"

# Check if target directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo "ERROR: Directory does not exist: $TARGET_DIR"
    exit 1
fi

# Convert to absolute path
TARGET_DIR="$(cd "$TARGET_DIR" && pwd)"

# Source setup_env.sh to set up the environment (suppress output)
source "$SCRIPT_DIR/setup_env.sh" > /dev/null 2>&1

# Get the path to brep_array_handler.py
BREP_HANDLER="$SCRIPT_DIR/brep_array_handler.py"

# Check if brep_array_handler.py exists
if [ ! -f "$BREP_HANDLER" ]; then
    echo "ERROR: brep_array_handler.py not found at $BREP_HANDLER"
    exit 1
fi

# Find all USD files in the target directory
USD_FILES=()
while IFS= read -r -d '' file; do
    USD_FILES+=("$file")
done < <(find "$TARGET_DIR" -maxdepth 1 -type f \( -name "*.usd" -o -name "*.usda" -o -name "*.usdc" \) -print0 | sort -z)

if [ ${#USD_FILES[@]} -eq 0 ]; then
    exit 0
fi

# Function to categorize and count messages by type
# Returns counts via global variables: _FAILURE_COUNT, _ERROR_COUNT, _WARNING_COUNT, _INFO_COUNT, _PASSED, _NO_BREP_ARRAYS
categorize_output() {
    local output="$1"
    local file="$2"
    
    # Initialize counters (using global variables with _ prefix)
    _FAILURE_COUNT=0
    _ERROR_COUNT=0
    _WARNING_COUNT=0
    _INFO_COUNT=0
    _PASSED=false
    _NO_BREP_ARRAYS=false
    
    # Use associative arrays to track unique messages with their requirement codes
    # Key is normalized message, value is "message|requirement"
    declare -A unique_errors
    declare -A unique_warnings
    declare -A unique_infos
    declare -A unique_other
    
    # Track requirement code counts (key: requirement code, value: count)
    # Declare as global so they persist after function returns
    declare -gA req_error_counts
    declare -gA req_warning_counts
    declare -gA req_info_counts
    
    # Track requirement descriptions (key: requirement code, value: message description)
    declare -gA req_error_descriptions
    declare -gA req_warning_descriptions
    declare -gA req_info_descriptions
    
    # Initialize arrays (clear any previous values)
    req_error_counts=()
    req_warning_counts=()
    req_info_counts=()
    req_error_descriptions=()
    req_warning_descriptions=()
    req_info_descriptions=()
    
    # Helper function to normalize a line (trim whitespace)
    normalize_line() {
        local line="$1"
        # Remove leading whitespace
        line="${line#"${line%%[![:space:]]*}"}"
        # Remove trailing whitespace
        line="${line%"${line##*[![:space:]]}"}"
        echo "$line"
    }
    
    # Helper function to extract requirement code from a line
    extract_requirement() {
        local line="$1"
        if [[ "$line" =~ Requirement:[[:space:]]*([A-Z]+\.[0-9]+) ]]; then
            echo "${BASH_REMATCH[1]}"
        fi
    }
    
    # Convert output to array of lines for easier lookahead
    mapfile -t output_lines <<< "$output"
    
    # Process each line of output
    for i in "${!output_lines[@]}"; do
        line="${output_lines[$i]}"
        normalized=$(normalize_line "$line")
        
        if [[ -z "$normalized" ]]; then
            continue
        fi
        
        # Check for severity markers
        if [[ "$normalized" =~ \[IssueSeverity\.FAILURE\] ]]; then
            ((_FAILURE_COUNT++))
            
            # Look ahead for requirement code (check next 5 lines to handle Suggestion lines)
            requirement=""
            for j in 1 2 3 4 5; do
                if [ $((i + j)) -lt ${#output_lines[@]} ]; then
                    next_line=$(normalize_line "${output_lines[$((i + j))]}")
                    req=$(extract_requirement "$next_line")
                    if [ -n "$req" ]; then
                        requirement="$req"
                        ((req_error_counts["$req"]++))
                        # Store description if not already set (use first message we see)
                        if [ -z "${req_error_descriptions[$req]}" ]; then
                            # Extract message text (remove severity prefix and rule suffix)
                            msg_text="${normalized#*] }"
                            msg_text="${msg_text% (Rule: *}"
                            req_error_descriptions["$req"]="$msg_text"
                        fi
                        break
                    fi
                    # Stop if we hit another severity line (start of next issue)
                    if [[ "$next_line" =~ \[IssueSeverity\. ]]; then
                        break
                    fi
                fi
            done
            
            # Store unique message with requirement code
            if [[ ! -v unique_errors["$normalized"] ]]; then
                unique_errors["$normalized"]="${normalized}|${requirement}"
            fi
        elif [[ "$normalized" =~ \[IssueSeverity\.ERROR\] ]]; then
            ((_ERROR_COUNT++))
            
            # Look ahead for requirement code (check next 5 lines to handle Suggestion lines)
            requirement=""
            for j in 1 2 3 4 5; do
                if [ $((i + j)) -lt ${#output_lines[@]} ]; then
                    next_line=$(normalize_line "${output_lines[$((i + j))]}")
                    req=$(extract_requirement "$next_line")
                    if [ -n "$req" ]; then
                        requirement="$req"
                        ((req_error_counts["$req"]++))
                        # Store description if not already set (use first message we see)
                        if [ -z "${req_error_descriptions[$req]}" ]; then
                            # Extract message text (remove severity prefix and rule suffix)
                            msg_text="${normalized#*] }"
                            msg_text="${msg_text% (Rule: *}"
                            req_error_descriptions["$req"]="$msg_text"
                        fi
                        break
                    fi
                    # Stop if we hit another severity line (start of next issue)
                    if [[ "$next_line" =~ \[IssueSeverity\. ]]; then
                        break
                    fi
                fi
            done
            
            # Store unique message with requirement code
            if [[ ! -v unique_errors["$normalized"] ]]; then
                unique_errors["$normalized"]="${normalized}|${requirement}"
            fi
        elif [[ "$normalized" =~ \[IssueSeverity\.WARNING\] ]]; then
            ((_WARNING_COUNT++))
            
            # Look ahead for requirement code (check next 5 lines to handle Suggestion lines)
            requirement=""
            for j in 1 2 3 4 5; do
                if [ $((i + j)) -lt ${#output_lines[@]} ]; then
                    next_line=$(normalize_line "${output_lines[$((i + j))]}")
                    req=$(extract_requirement "$next_line")
                    if [ -n "$req" ]; then
                        requirement="$req"
                        ((req_warning_counts["$req"]++))
                        # Store description if not already set (use first message we see)
                        if [ -z "${req_warning_descriptions[$req]}" ]; then
                            # Extract message text (remove severity prefix and rule suffix)
                            msg_text="${normalized#*] }"
                            msg_text="${msg_text% (Rule: *}"
                            req_warning_descriptions["$req"]="$msg_text"
                        fi
                        break
                    fi
                    # Stop if we hit another severity line (start of next issue)
                    if [[ "$next_line" =~ \[IssueSeverity\. ]]; then
                        break
                    fi
                fi
            done
            
            # Store unique message with requirement code
            if [[ ! -v unique_warnings["$normalized"] ]]; then
                unique_warnings["$normalized"]="${normalized}|${requirement}"
            fi
        elif [[ "$normalized" =~ \[IssueSeverity\.INFO\] ]]; then
            ((_INFO_COUNT++))
            
            # Look ahead for requirement code (check next 5 lines to handle Suggestion lines)
            requirement=""
            for j in 1 2 3 4 5; do
                if [ $((i + j)) -lt ${#output_lines[@]} ]; then
                    next_line=$(normalize_line "${output_lines[$((i + j))]}")
                    req=$(extract_requirement "$next_line")
                    if [ -n "$req" ]; then
                        requirement="$req"
                        ((req_info_counts["$req"]++))
                        # Store description if not already set (use first message we see)
                        if [ -z "${req_info_descriptions[$req]}" ]; then
                            # Extract message text (remove severity prefix and rule suffix)
                            msg_text="${normalized#*] }"
                            msg_text="${msg_text% (Rule: *}"
                            req_info_descriptions["$req"]="$msg_text"
                        fi
                        break
                    fi
                    # Stop if we hit another severity line (start of next issue)
                    if [[ "$next_line" =~ \[IssueSeverity\. ]]; then
                        break
                    fi
                fi
            done
            
            # Store unique message with requirement code
            if [[ ! -v unique_infos["$normalized"] ]]; then
                unique_infos["$normalized"]="${normalized}|${requirement}"
            fi
        elif [[ "$normalized" =~ "Validation passed: No issues found" ]]; then
            _PASSED=true
        elif [[ "$normalized" =~ "No BrepArray prims found" ]]; then
            _NO_BREP_ARRAYS=true
        elif [[ "$normalized" =~ ^(ERROR|Error:) ]]; then
            # Store unique message only if not already present
            if [[ ! -v unique_other["$normalized"] ]]; then
                unique_other["$normalized"]="$normalized|"
            fi
        fi
    done
    
    # Print summary for this file
    echo "Path: $file"
    
    if [ "$_NO_BREP_ARRAYS" = true ]; then
        echo "  Status: No BrepArray prims found"
    elif [ "$_PASSED" = true ]; then
        echo "  Status: PASSED - No issues found"
    elif [ $_FAILURE_COUNT -gt 0 ] || [ $_ERROR_COUNT -gt 0 ]; then
        echo "  Status: FAILED"
    elif [ $_WARNING_COUNT -gt 0 ] || [ $_INFO_COUNT -gt 0 ]; then
        echo "  Status: WARNINGS/INFO"
    else
        echo "  Status: UNKNOWN"
    fi
    
    # Print requirement code summaries for this file
    if [ ${#req_error_counts[@]} -gt 0 ] || [ ${#req_warning_counts[@]} -gt 0 ] || [ ${#req_info_counts[@]} -gt 0 ]; then
        echo "  Requirements:"
        # Sort requirement codes for consistent output
        for req in $(printf '%s\n' "${!req_error_counts[@]}" | sort); do
            desc="${req_error_descriptions[$req]}"
            if [ -n "$desc" ]; then
                echo "    - $req: ${req_error_counts[$req]} error(s) - $desc"
            else
                echo "    - $req: ${req_error_counts[$req]} error(s)"
            fi
        done
        for req in $(printf '%s\n' "${!req_warning_counts[@]}" | sort); do
            desc="${req_warning_descriptions[$req]}"
            if [ -n "$desc" ]; then
                echo "    - $req: ${req_warning_counts[$req]} warning(s) - $desc"
            else
                echo "    - $req: ${req_warning_counts[$req]} warning(s)"
            fi
        done
        for req in $(printf '%s\n' "${!req_info_counts[@]}" | sort); do
            desc="${req_info_descriptions[$req]}"
            if [ -n "$desc" ]; then
                echo "    - $req: ${req_info_counts[$req]} info message(s) - $desc"
            else
                echo "    - $req: ${req_info_counts[$req]} info message(s)"
            fi
        done
    fi
    
    # Print info messages and other messages if any
    if [ ${#unique_infos[@]} -gt 0 ]; then
        echo "  Info Messages:"
        for msg_key in "${!unique_infos[@]}"; do
            IFS='|' read -r msg requirement <<< "${unique_infos[$msg_key]}"
            if [ -n "$requirement" ]; then
                echo "    [$requirement] $msg"
            else
                echo "    $msg"
            fi
        done
    fi
    
    if [ ${#unique_other[@]} -gt 0 ]; then
        echo "  Other Messages:"
        for msg in "${!unique_other[@]}"; do
            echo "    $msg"
        done
    fi
    
    echo ""
}

# Function to process a single file (runs in parallel)
process_file() {
    local usd_file="$1"
    local temp_file="$2"
    local repo_root="$3"
    local brep_handler="$4"
    
    # Run brep_array_handler and capture output
    output=$(cd "$repo_root" && python3 "$brep_handler" "$usd_file" 2>&1)
    exit_code=$?
    
    # Write results to temp file
    if [ $exit_code -eq 0 ]; then
        # Categorize the output (this sets global variables and associative arrays)
        # Redirect output to temp file, then append counts
        categorize_output "$output" "$usd_file" > "$temp_file"
        # Append counts line (global variables are set by categorize_output)
        echo "COUNTS:${_FAILURE_COUNT:-0}:${_ERROR_COUNT:-0}:${_WARNING_COUNT:-0}:${_INFO_COUNT:-0}:${_PASSED:-false}:${_NO_BREP_ARRAYS:-false}" >> "$temp_file"
        # Append requirement counts and descriptions
        {
            for req in "${!req_error_counts[@]}"; do
                desc="${req_error_descriptions[$req]}"
                # Replace colons in description with semicolons to avoid parsing issues
                desc_safe="${desc//:/;}"
                echo "REQ_ERROR:$req:${req_error_counts[$req]}:$desc_safe"
            done
            for req in "${!req_warning_counts[@]}"; do
                desc="${req_warning_descriptions[$req]}"
                desc_safe="${desc//:/;}"
                echo "REQ_WARNING:$req:${req_warning_counts[$req]}:$desc_safe"
            done
            for req in "${!req_info_counts[@]}"; do
                desc="${req_info_descriptions[$req]}"
                desc_safe="${desc//:/;}"
                echo "REQ_INFO:$req:${req_info_counts[$req]}:$desc_safe"
            done
        } >> "$temp_file"
    else
        {
            echo "File: $(basename "$usd_file")"
            echo "  Path: $usd_file"
            echo "  ERROR: Failed to run validation (exit code: $exit_code)"
            echo "  Output:"
            echo "$output" | sed 's/^/    /'
            echo ""
            echo "COUNTS:0:0:0:0:false:false"
        } > "$temp_file"
    fi
}

# Process files in parallel
TOTAL_FILES=${#USD_FILES[@]}
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Create temporary directory for results
TEMP_DIR=$(mktemp -d)
trap "rm -rf '$TEMP_DIR'" EXIT

# Launch parallel jobs
job_count=0
declare -a job_pids
declare -a job_files
declare -a job_usd_files

for usd_file in "${USD_FILES[@]}"; do
    temp_file="$TEMP_DIR/result_${job_count}.txt"
    job_files+=("$temp_file")
    job_usd_files+=("$usd_file")
    
    # Launch background job
    process_file "$usd_file" "$temp_file" "$REPO_ROOT" "$BREP_HANDLER" &
    job_pids+=($!)
    ((job_count++))
done

# Wait for all jobs to complete
for pid in "${job_pids[@]}"; do
    wait "$pid"
done

# Collect results first (store file outputs for later display)
TOTAL_FAILURES=0
TOTAL_ERRORS=0
TOTAL_WARNINGS=0
TOTAL_INFOS=0
PASSED_FILES=0
FAILED_FILES=0

# Aggregate requirement counts across all files
declare -A total_req_error_counts
declare -A total_req_warning_counts
declare -A total_req_info_counts
declare -A total_req_error_descriptions
declare -A total_req_warning_descriptions
declare -A total_req_info_descriptions

# Store file outputs for later display
declare -a file_outputs

for i in "${!job_files[@]}"; do
    temp_file="${job_files[$i]}"
    usd_file="${job_usd_files[$i]}"
    
    if [ -f "$temp_file" ]; then
        # Store the main output (everything except COUNTS and REQ_ lines) for later display
        file_output=$(grep -v "^COUNTS:" "$temp_file" | grep -v "^REQ_" || true)
        file_outputs+=("$file_output")
        
        # Extract counts from the COUNTS line
        counts_line=$(grep "^COUNTS:" "$temp_file" || echo "COUNTS:0:0:0:0:false:false")
        IFS=':' read -r _ failures errors warnings infos passed no_brep <<< "$counts_line"
        
        # Provide default values if parsing failed or values are empty
        failures=${failures:-0}
        errors=${errors:-0}
        warnings=${warnings:-0}
        infos=${infos:-0}
        passed=${passed:-false}
        no_brep=${no_brep:-false}
        
        # Accumulate counts (ensure they're treated as integers)
        TOTAL_FAILURES=$((TOTAL_FAILURES + failures))
        TOTAL_ERRORS=$((TOTAL_ERRORS + errors))
        TOTAL_WARNINGS=$((TOTAL_WARNINGS + warnings))
        TOTAL_INFOS=$((TOTAL_INFOS + infos))
        
        # Aggregate requirement counts and descriptions
        while IFS=':' read -r type req count desc; do
            count=${count:-0}  # Default to 0 if empty
            # Restore colons in description (they were replaced with semicolons)
            desc="${desc//;/:}"
            if [ "$type" = "REQ_ERROR" ] && [ -n "$req" ]; then
                total_req_error_counts["$req"]=$((${total_req_error_counts["$req"]:-0} + count))
                # Store description if not already set
                if [ -z "${total_req_error_descriptions[$req]}" ] && [ -n "$desc" ]; then
                    total_req_error_descriptions["$req"]="$desc"
                fi
            elif [ "$type" = "REQ_WARNING" ] && [ -n "$req" ]; then
                total_req_warning_counts["$req"]=$((${total_req_warning_counts["$req"]:-0} + count))
                # Store description if not already set
                if [ -z "${total_req_warning_descriptions[$req]}" ] && [ -n "$desc" ]; then
                    total_req_warning_descriptions["$req"]="$desc"
                fi
            elif [ "$type" = "REQ_INFO" ] && [ -n "$req" ]; then
                total_req_info_counts["$req"]=$((${total_req_info_counts["$req"]:-0} + count))
                # Store description if not already set
                if [ -z "${total_req_info_descriptions[$req]}" ] && [ -n "$desc" ]; then
                    total_req_info_descriptions["$req"]="$desc"
                fi
            fi
        done < <(grep "^REQ_" "$temp_file" 2>/dev/null || true)
        
        # Check if validation passed
        if [ "$passed" = "true" ]; then
            ((PASSED_FILES++))
        elif [ "$failures" -gt 0 ] || [ "$errors" -gt 0 ]; then
            ((FAILED_FILES++))
        elif [ "$no_brep" = "false" ]; then
            # File processed but had warnings/info or unknown status
            # Count as passed if no failures/errors
            if [ "$failures" -eq 0 ] && [ "$errors" -eq 0 ]; then
                ((PASSED_FILES++))
            fi
        fi
    fi
done

# Print overall summary first
echo "=========================================="
echo "Overall Summary"
echo "=========================================="
echo "Total files processed: $TOTAL_FILES"
echo "Files passed: $PASSED_FILES"
echo "Files failed: $FAILED_FILES"
echo ""

# Print requirement code summary (same format as per-file, but aggregated)
if [ ${#total_req_error_counts[@]} -gt 0 ] || [ ${#total_req_warning_counts[@]} -gt 0 ] || [ ${#total_req_info_counts[@]} -gt 0 ]; then
    echo "Requirements Summary:"
    # Sort requirement codes for consistent output
    for req in $(printf '%s\n' "${!total_req_error_counts[@]}" | sort); do
        desc="${total_req_error_descriptions[$req]}"
        if [ -n "$desc" ]; then
            echo "    - $req: ${total_req_error_counts[$req]} error(s) - $desc"
        else
            echo "    - $req: ${total_req_error_counts[$req]} error(s)"
        fi
    done
    for req in $(printf '%s\n' "${!total_req_warning_counts[@]}" | sort); do
        desc="${total_req_warning_descriptions[$req]}"
        if [ -n "$desc" ]; then
            echo "    - $req: ${total_req_warning_counts[$req]} warning(s) - $desc"
        else
            echo "    - $req: ${total_req_warning_counts[$req]} warning(s)"
        fi
    done
    for req in $(printf '%s\n' "${!total_req_info_counts[@]}" | sort); do
        desc="${total_req_info_descriptions[$req]}"
        if [ -n "$desc" ]; then
            echo "    - $req: ${total_req_info_counts[$req]} info message(s) - $desc"
        else
            echo "    - $req: ${total_req_info_counts[$req]} info message(s)"
        fi
    done
    echo ""
fi

echo "=========================================="
echo "Individual File Results"
echo "=========================================="
echo ""

# Now display individual file results
for file_output in "${file_outputs[@]}"; do
    echo "$file_output"
done

if [ $FAILED_FILES -eq 0 ] && [ $TOTAL_FAILURES -eq 0 ] && [ $TOTAL_ERRORS -eq 0 ]; then
    echo "SUCCESS: All files validated successfully!"
    exit 0
else
    echo "WARNING: Some files had validation issues. Review the output above for details."
    exit 1
fi

# This script is used to convert 128bit text hex values in 32bit text hex values for use by the text2octave script

import sys

def split_hex_value(hex_value):
    # Split the 128-bit hex value into four 32-bit hex values
    return [hex_value[i:i+8] for i in range(0, len(hex_value), 8)]

input_file = sys.argv[1] ; #"cfo_output_sim_128.txt"
output_file = sys.argv[2] ;#"cfo_output_sim.txt"

with open(input_file, "r") as file:
    input_data = file.read()

# Split the input data into individual lines
lines = input_data.split("\n")

# Process each line and split the 128-bit hex value into 32-bit hex values
split_values = [split_hex_value(line) for line in lines if line]

rr = [reversed(line) for line in split_values];

flattened_values = [value for sublist in rr for value in sublist]

# Write the split values to the output file
with open(output_file, "w") as file:
    file.write("\n".join(flattened_values))

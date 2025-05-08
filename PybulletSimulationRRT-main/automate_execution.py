import subprocess

# List of objects (directories)
objects = [
    "003_cracker_box"
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
    "011_banana",
    "012_strawberry",
    "016_pear",
    "017_orange",
    "018_plum",
    "024_bowl",
    "030_fork",
    "031_spoon",
    "032_knife",
    "033_spatula",
    "035_power_drill",
    "038_padlock",
    "040_large_marker",
    "042_adjustable_wrench",
    "043_phillips_screwdriver",
    "044_flat_screwdriver",
    "048_hammer",
    "065-c_cups",
    "072-a_toy_airplane",
    "077_rubiks_cube"
]

script_name = "pipeline_vizrrt.py"

# Open the log file once to append results
with open("CGNresults_multiple_objects.log", "a") as log_file:
    for obj in objects:
        for idx in range(16):
            print(f"Running script for object={obj} with idx={idx}...")
            
            # Run the script and capture output
            result = subprocess.run(
                ["python", script_name, obj, str(idx)],
                stdout=subprocess.PIPE,  # Capture standard output
                stderr=subprocess.PIPE,  # Capture standard error
                text=True  # Get output as a string
            )
            
            # Write the executed command to the log
            log_file.write(f"Object: {obj}, Command: python {script_name} {obj} {idx}\n")
            
            # Check for "Success" or "Failure" in the output
            if "Success" in result.stdout:
                log_file.write("Result: Success\n")
                print(f"Success received for object={obj} with idx={idx}")
                break
            elif "Failure" in result.stdout:
                log_file.write("Result: Failure\n")
                print(f"Failure for object={obj} with idx={idx}")
            else:
                log_file.write("Result: Unexpected Output\n")
                print(f"Unexpected output for object={obj} with idx={idx}. Check results_multiple_objects.log.")
            
            # Add a separator for readability in the log
            log_file.write("\n")

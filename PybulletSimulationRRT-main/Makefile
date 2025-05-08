# # Makefile to run the creating_pipeline.py script for all URDF files

# # Define the directory for URDF files and output directory
# URDF_DIR := ./Other_urdf
# OUTPUT_DIR := pipeline_output

# # Find all .urdf files in the URDF directory
# URDF_FILES := $(wildcard $(URDF_DIR)/*.urdf)

# # Default target that will run the script for each URDF file
# all: run_pipeline_for_all

# run_pipeline_for_all:
# 	@for urdf in $(URDF_FILES); do \
# 		echo "Processing $$urdf..."; \
# 		python creating_pipeline.py $$urdf $(OUTPUT_DIR); \
# 	done



# Compiler/Interpreter
PYTHON = python

# Script to execute
SCRIPT = mycode.py

# Base directories
URDF_DIR = E:/linux_storage/PybulletSimRRT/Other_urdf
NPZ_DIR = E:/linux_storage/PybulletSimRRT/generated_grasps
# sponge
# 032_knife \

# List of files (without extensions)
# FILES = \

FILES = \
    028_skillet_lid \
    029_plate \
    030_fork \
    031_spoon \
    032_knife \
    033_spatula \
    034_screwdriver \
    035_power_drill \
    036_wood_block \
    037_scissors \
    038_padlock \
    039_bench_vise \
    040_large_marker \
    041_eraser \
    042_adjustable_wrench \
    043_phillips_screwdriver \
    044_flat_screwdriver \
    048_hammer \
    050_medium_clamp \
    051_large_clamp \
    052_extra_large_clamp \
    053_mini_soccer_ball \
    054_softball \
    055_baseball \
    065-c_cups \
    065-d_cups \
    065-e_cups \
    072-a_toy_airplane \
    073-f_lego_duplo \
    077_rubiks_cube


# Pattern rule for executing the script
.PHONY: all
all: $(FILES)

$(FILES):
	$(PYTHON) $(SCRIPT) $(URDF_DIR)/$@.urdf $(NPZ_DIR)/$@.npz
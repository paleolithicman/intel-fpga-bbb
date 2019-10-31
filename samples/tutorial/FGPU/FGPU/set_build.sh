#!/bin/sh
BUILD_DIR=build_test

afu_synth_setup -s ./hw/RTL/sources.txt $BUILD_DIR
cp -r ./hw/project_quartus/sigmoid_sp_s10_500_ver ${BUILD_DIR}/build/
cp -r ./hw/project_quartus/tanh_sp_s10_500_ver ${BUILD_DIR}/build/
echo "set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008" >> ${BUILD_DIR}/build/afu_default.qsf

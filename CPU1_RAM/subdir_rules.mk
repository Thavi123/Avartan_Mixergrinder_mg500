################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1873601423: ../c2000.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs2040/ccs/utils/sysconfig_1.26.0/sysconfig_cli.bat" -s "C:/ti/C2000Ware_6_00_01_00/.metadata/sdk.json" -d "F280013x" -p "64PM" -r "F280013x_64PM" --script "D:/git_code/Avartan_Mixergrinder_mg500/c2000.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-1873601423 ../c2000.syscfg
syscfg/board.h: build-1873601423
syscfg/board.cmd.genlibs: build-1873601423
syscfg/board.opt: build-1873601423
syscfg/board.json: build-1873601423
syscfg/pinmux.csv: build-1873601423
syscfg/epwm.dot: build-1873601423
syscfg/adc.dot: build-1873601423
syscfg/c2000ware_libraries.cmd.genlibs: build-1873601423
syscfg/c2000ware_libraries.opt: build-1873601423
syscfg/c2000ware_libraries.c: build-1873601423
syscfg/c2000ware_libraries.h: build-1873601423
syscfg/clocktree.h: build-1873601423
syscfg: build-1873601423

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 -Ooff --include_path="D:/git_code/Avartan_Mixergrinder_mg500" --include_path="C:/ti/C2000Ware_6_00_01_00" --include_path="D:/git_code/Avartan_Mixergrinder_mg500/device" --include_path="C:/ti/C2000Ware_6_00_01_00/driverlib/f280013x/driverlib/" --include_path="C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=DEBUG --define=RAM --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="D:/git_code/Avartan_Mixergrinder_mg500/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 -Ooff --include_path="D:/git_code/Avartan_Mixergrinder_mg500" --include_path="C:/ti/C2000Ware_6_00_01_00" --include_path="D:/git_code/Avartan_Mixergrinder_mg500/device" --include_path="C:/ti/C2000Ware_6_00_01_00/driverlib/f280013x/driverlib/" --include_path="C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=DEBUG --define=RAM --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="D:/git_code/Avartan_Mixergrinder_mg500/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



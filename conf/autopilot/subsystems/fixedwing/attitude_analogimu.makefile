# attitude via analog imu


ifeq ($(ARCH), lpc21)
ap.CFLAGS += -DANALOG_IMU -DADC -DUSE_ADC_0 -DUSE_ADC_1 -DUSE_ADC_2 -DUSE_ADC_3 -DUSE_ADC_4  -DUSE_ADC_5 -DUSE_ADC_6 -DUSE_ADC_7 

ap.srcs += $(SRC_FIXEDWING)/subsystems/analogimu/dcm.c $(SRC_FIXEDWING)/subsystems/analogimu/arduimu.c $(SRC_FIXEDWING)/subsystems/analogimu/matrix.c $(SRC_FIXEDWING)/subsystems/analogimu/vector.c

ap.srcs += $(SRC_FIXEDWING)/subsystems/analogimu/analogimu.c $(SRC_FIXEDWING)/subsystems/analogimu/analogimu_util.c
endif


sim.srcs += $(SRC_ARCH)/sim_ir.c
jsbsim.srcs += $(SRC_ARCH)/jsbsim_ir.c

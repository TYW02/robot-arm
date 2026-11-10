################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

ifeq ($(CLAW_TEST),1)
# No mission, wheel driver, vision, or IMU application code in the test image.
TEMP_SRCS = ../app_program/claw_app.c ../app_program/claw_serial_test_app.c
else
TEMP_SRCS = $(filter-out ../app_program/app_main.c ../app_program/demo_tasks.c ../app_program/claw_serial_test_app.c,$(wildcard ../app_program/*.c))
endif
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

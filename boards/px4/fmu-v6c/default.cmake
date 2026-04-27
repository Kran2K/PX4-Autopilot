
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v6c
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS0
		GPS2:/dev/ttyS6
		TEL1:/dev/ttyS5
		TEL2:/dev/ttyS3
		TEL3:/dev/ttyS1
	DRIVERS
		adc
		barometer
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure
		distance_sensor
		dshot
		gps
		heater
		imu/bosch/bmi055
		imu/invensense/icm42688p
		lights
		magnetometer
		optical_flow
		power_monitor/ina226
		#power_monitor/ina228 # Might not exist in 1.11
		#power_monitor/ina238 # Might not exist in 1.11
		pwm_out
		pwm_out_sim
		px4io
		telemetry
		tone_alarm
		uavcan
	MODULES
		airspeed_selector
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		esc_battery
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		rover_pos_control
		sensors
		sih
		temperature_compensation
		vmount
		vtol_att_control
	SYSTEMCMDS
		bl_update
		dmesg
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		sd_bench
		top
		topic_listener
		tune_control
		ver
		work_queue
	)

/*
 *
 * Copyright (c) [2018] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/* Driver */
#include "imu/inv_imu_driver.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* 
 * FSYNC toggle frequency emulating a camera module
 */
#define FSYNC_FREQUENCY_HZ 30

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is asserted */
static volatile uint64_t int1_timestamp; /* Timestamp when INT1 is asserted */

/* Static variables for command interface */
static uint8_t fifo_en; /* Indicates if data are read from FIFO (1) or registers (0) */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_fifo();
static void int_cb(void *context, unsigned int int_num);
static void fsync_cb(void *context);
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static int  get_uart_command();
static int  print_help();
static int  print_current_config();

/* Main function implementation */
int main(void)
{
	int rc = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example EIS");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	fifo_en = 1;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			si_enable_irq();

			if (fifo_en)
				rc |= inv_imu_get_data_from_fifo(&imu_dev);
			else
				rc |= inv_imu_get_data_from_registers(&imu_dev);

			SI_CHECK_RC(rc);
			rc = 0; /* reset `rc` (contains the number of packet read if above check is passing) */
		}

		rc |= get_uart_command();
	} while (rc == 0);

	return rc;
}

/* Initializes MCU peripherals. */
static int setup_mcu()
{
	int rc = 0;

	rc |= si_board_init();

	/* Configure UART for log */
	rc |= si_config_uart_for_print(SI_UART_ID_FTDI, INV_MSG_LEVEL_DEBUG);

	/* Configure GPIO to call `int_cb` when INT1 fires. */
	rc |= si_init_gpio_int(SI_GPIO_INT1, int_cb);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	/* Send FSYNC signal to IMU to emulate camera module at 30Hz */
	rc |= si_start_gpio_fsync(2 * FSYNC_FREQUENCY_HZ /* FSYNC with rising edge @ 30Hz */, fsync_cb);

	return rc;
}

static int setup_imu()
{
	int                       rc = 0;
	inv_imu_serif_t           imu_serif;
	uint8_t                   whoami;
	inv_imu_int1_pin_config_t int1_pin_config;

	/* Initialize serial interface between MCU and IMU */
	imu_serif.context    = 0; /* no need */
	imu_serif.read_reg   = si_io_imu_read_reg;
	imu_serif.write_reg  = si_io_imu_write_reg;
	imu_serif.max_read   = 1024 * 32; /* maximum number of bytes allowed per serial read */
	imu_serif.max_write  = 1024 * 32; /* maximum number of bytes allowed per serial write */
	imu_serif.serif_type = SERIF_TYPE;

	/* Init device */
	rc |= inv_imu_init(&imu_dev, &imu_serif, sensor_event_cb);
	SI_CHECK_RC(rc);

#if SERIF_TYPE == UI_SPI4
	/* Configure slew-rate to 19 ns (required when using EVB) */
	rc |= inv_imu_set_spi_slew_rate(&imu_dev, DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_19_NS);
	SI_CHECK_RC(rc);
#endif

	/* Check WHOAMI */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return INV_ERROR;
	}

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
	int1_pin_config.int_mode     = INT_CONFIG_INT1_MODE_PULSED;
	int1_pin_config.int_drive    = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
	rc |= inv_imu_set_pin_config_int1(&imu_dev, &int1_pin_config);

	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_FS_SEL_2000dps);
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_ODR_200_HZ);
	rc |= configure_fifo(&imu_dev);
	rc |= inv_imu_enable_fsync(&imu_dev);
	rc |= inv_imu_enable_gyro_low_noise_mode(&imu_dev);

	return rc;
}

static int configure_fifo()
{
	int                           rc          = 0;
	uint8_t                       data        = 0;
	inv_imu_interrupt_parameter_t int1_config = { (inv_imu_interrupt_value)0 };

	rc |= inv_imu_configure_fifo(&imu_dev, fifo_en ? INV_IMU_FIFO_ENABLED : INV_IMU_FIFO_DISABLED);

	if (fifo_en) {
		int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
	} else {
		int1_config.INV_UI_DRDY = INV_IMU_ENABLE;

		/* Configure IMU to tag FSYNC in TEMP register LSB */
		rc |= inv_imu_read_reg(&imu_dev, FSYNC_CONFIG_MREG1, 1, &data);
		data &= ~FSYNC_CONFIG_FSYNC_UI_SEL_MASK;
		data |= (uint8_t)FSYNC_CONFIG_UI_SEL_TEMP;
		rc |= inv_imu_write_reg(&imu_dev, FSYNC_CONFIG_MREG1, 1, &data);
	}
	rc |= inv_imu_set_config_int1(&imu_dev, &int1_config);

	return rc;
}

/* IMU interrupt handler. */
static void int_cb(void *context, unsigned int int_num)
{
	(void)context;

	if (int_num == SI_GPIO_INT1) {
		int1_timestamp = si_get_time_us();
		int1_flag      = 1;
	}
}

/* FSYNC callback. */
static void fsync_cb(void *context)
{
	(void)context;
	si_toggle_gpio_fsync();
}

static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
	uint64_t int_timestamp  = 0;
	int      fsync_detected = 0;
	uint16_t fsync_delay    = 0;

	/* Get timestamp */
	si_disable_irq();
	int_timestamp = int1_timestamp;
	si_enable_irq();

	/* Check FSYNC tag */
	if (fifo_en ? (event->sensor_mask & (1 << INV_SENSOR_FSYNC_EVENT)) : (event->temperature & 1)) {
		/*
		 * When data are read from FIFO, FSYNC is tagged in FIFO's header and FSYNC delay replaces
		 * timestamp in the `timestamp_fsync` field.
		 *
		 * When data are read from registers, FSYNC is tagged in sensor data LSB (configured in this
		 * example to TEMP register LSB).
		 * FSYNC delay is read from TMST_FSYNC registers.
		 */
		if (!fifo_en) {
			uint8_t fsync_count[2];

			inv_imu_read_reg(&imu_dev, TMST_FSYNCH, 2, fsync_count);
			format_u16_data(imu_dev.endianness_data, &fsync_count[0], &event->timestamp_fsync);
		}

		fsync_detected = 1;
		fsync_delay    = event->timestamp_fsync;
	}

	/* Check gyro data */
	if (fifo_en ? ((event->sensor_mask & (1 << INV_SENSOR_GYRO)) &&
	               (event->sensor_mask & (1 << INV_SENSOR_TEMPERATURE))) :
                  (event->gyro[0] != INVALID_VALUE_FIFO)) {
		/*
		 * Gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768)
		 * with the configured FSR (2000 dps, see `setup_imu()` function).
		 */
		int   max_lsb      = 32768; /* 16-bits signed (max_lsb = 2^(16-1) = 32768) */
		int   gyro_fsr_dps = 2000; /* 2000 dps, see `setup_imu()` function */
		float gyro_dps[3];
		char  string_fsync[32] = "";

		/* Convert raw data from FIFO data to SI units */
		gyro_dps[0] = (float)(event->gyro[0] * gyro_fsr_dps) / max_lsb;
		gyro_dps[1] = (float)(event->gyro[1] * gyro_fsr_dps) / max_lsb;
		gyro_dps[2] = (float)(event->gyro[2] * gyro_fsr_dps) / max_lsb;

		/* FSYNC delay has a 1 us resolution, regardless of `TMST_RES` field. */
		if (fsync_detected)
			snprintf(string_fsync, sizeof(string_fsync), "FSYNC occurred %hu us ago", fsync_delay);

		INV_MSG(INV_MSG_LEVEL_INFO, "%10llu us   Gyro:% 8.2f % 8.2f % 8.2f dps   %s", int_timestamp,
		        gyro_dps[0], gyro_dps[1], gyro_dps[2], string_fsync);
	}
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'f': /* Use FIFO or sensor register */
		fifo_en = !fifo_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s FIFO.", fifo_en ? "Enabling" : "Disabling");
		rc |= configure_fifo();
		break;
	case 'c':
		rc |= print_current_config();
		break;
	case 'h':
	case 'H':
		rc |= print_help();
		break;
	case 0:
		break; /* No command received */
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", cmd);
		rc |= print_help();
		break;
	}

	return rc;
}

/* Help for UART command interface */
static int print_help()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Help");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'f' : Toggle FIFO usage");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'c' : Print current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'h' : Print this helper");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}

/* Print current sample configuration */
static int print_current_config()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# FIFO: %s", fifo_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}

/* Get time implementation for IMU driver */
uint64_t inv_imu_get_time_us(void)
{
	return si_get_time_us();
}

/* Sleep implementation for IMU driver */
void inv_imu_sleep_us(uint32_t us)
{
	si_sleep_us(us);
}

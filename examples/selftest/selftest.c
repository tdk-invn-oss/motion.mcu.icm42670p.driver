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
#include "imu/inv_imu_selftest.h"

/* Board drivers */
#include "system_interface.h"

#include <stddef.h> /* NULL */

/*
 * This example showcases how to run self-test with DMP.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* Static variables */
static inv_imu_device_t imu_dev; /* Driver structure */

/* Static functions definition */
static int setup_mcu();
static int setup_imu();

/* Main function implementation */
int main(void)
{
	int rc = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example Self-Test");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	do {
		int                           rc   = 0;
		static int                    iter = 0;
		inv_imu_selftest_output_t     out;
		inv_imu_selftest_parameters_t params;

		rc |= inv_imu_init_selftest_parameters_struct(&imu_dev, &params);
		/* Update `params` if needed here */
		rc |= inv_imu_run_selftest(&imu_dev, params, &out);
		SI_CHECK_RC(rc);

		/* Print self-test status */
		INV_MSG(INV_MSG_LEVEL_INFO, "[%u] Accel self-test %s", iter,
		        out.accel_status == 1 ? "OK" : "KO");
		if (out.accel_status != 1) {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel X: %s", out.ax_status == 1 ? "OK" : "KO");
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel Y: %s", out.ay_status == 1 ? "OK" : "KO");
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel Z: %s", out.az_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}

#if INV_IMU_IS_GYRO_SUPPORTED
		INV_MSG(INV_MSG_LEVEL_INFO, "[%u] Gyro self-test %s", iter,
		        out.gyro_status == 1 ? "OK" : "KO");
		if (out.gyro_status != 1) {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro X: %s", out.gx_status == 1 ? "OK" : "KO");
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro Y: %s", out.gy_status == 1 ? "OK" : "KO");
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro Z: %s", out.gz_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}

		/* Check incomplete state */
		if (out.gyro_status & 0x2) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "[%u] Gyro self-test are incomplete.", iter);
			rc |= INV_ERROR;
		}
#endif

		iter++;

		/* Print empty line to ease readability */
		INV_MSG(INV_MSG_LEVEL_INFO, "");
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

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	return rc;
}

static int setup_imu()
{
	int             rc = 0;
	inv_imu_serif_t imu_serif;
	uint8_t         whoami;

	/* Initialize serial interface between MCU and IMU */
	imu_serif.context    = 0; /* no need */
	imu_serif.read_reg   = si_io_imu_read_reg;
	imu_serif.write_reg  = si_io_imu_write_reg;
	imu_serif.max_read   = 1024 * 32; /* maximum number of bytes allowed per serial read */
	imu_serif.max_write  = 1024 * 32; /* maximum number of bytes allowed per serial write */
	imu_serif.serif_type = SERIF_TYPE;

	/* Init device */
	rc |= inv_imu_init(&imu_dev, &imu_serif, NULL);
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

	return rc;
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

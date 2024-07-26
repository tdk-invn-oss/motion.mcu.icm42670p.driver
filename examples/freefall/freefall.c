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
#include "imu/inv_imu_apex.h"

/* Board drivers */
#include "system_interface.h"

#include <stddef.h> /* NULL */

/*
 * This example showcases how to configure and use FreeFall.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */

/* Static functions definition */
static int   setup_mcu();
static int   setup_imu();
static void  int_cb(void *context, unsigned int int_num);
static float convert_ff_duration_sample_to_cm(uint16_t ff_duration_samples);

/* Main function implementation */
int main(void)
{
	int rc = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example FreeFall");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			uint8_t  int_status3;
			uint64_t timestamp;

			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			/* Retrieve timestamp */
			timestamp = int1_timestamp;
			si_enable_irq();

			/* Read FreeFall interrupt status */
			rc |= inv_imu_read_reg(&imu_dev, INT_STATUS3, 1, &int_status3);
			SI_CHECK_RC(rc);

			if (int_status3 & (INT_STATUS3_LOWG_DET_INT_MASK))
				INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   LOW_G", timestamp);

			if (int_status3 & (INT_STATUS3_FF_DET_INT_MASK)) {
				uint8_t  data[2];
				uint16_t ff_duration_samples;
				float    ff_distance_cm;

				rc |= inv_imu_read_reg(&imu_dev, APEX_DATA4, 2, &data[0]);
				ff_duration_samples = (uint16_t)((data[1] << 8) | data[0]);

				ff_distance_cm = convert_ff_duration_sample_to_cm(ff_duration_samples);

				INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   FREEFALL     length: %.2f cm (%d ms)",
				        timestamp, ff_distance_cm, ff_duration_samples);
			}
		}
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

	return rc;
}

static int setup_imu()
{
	int                           rc = 0;
	inv_imu_serif_t               imu_serif;
	uint8_t                       whoami;
	inv_imu_apex_parameters_t     apex_inputs;
	inv_imu_int1_pin_config_t     int1_pin_config;
	inv_imu_interrupt_parameter_t int1_config = { (inv_imu_interrupt_value)0 };

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

	/* Configure interrupts sources */
	int1_config.INV_FF   = INV_IMU_ENABLE;
	int1_config.INV_LOWG = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int1(&imu_dev, &int1_config);

	/* Disabling FIFO usage to optimize power consumption */
	rc |= inv_imu_configure_fifo(&imu_dev, INV_IMU_FIFO_DISABLED);

	/* Configure accel frequency */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_400_HZ);
	rc |= inv_imu_apex_set_frequency(&imu_dev, APEX_CONFIG1_DMP_ODR_400Hz);

	/* Set 2X averaging to minimize power consumption */
	rc |= inv_imu_set_accel_lp_avg(&imu_dev, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);

	/* Enable accel in LP mode */
	rc |= inv_imu_enable_accel_low_power_mode(&imu_dev);

	/* Set APEX parameters */
	rc |= inv_imu_apex_init_parameters_struct(&imu_dev, &apex_inputs);
	/* Parameters can be modified here if needed */
	apex_inputs.power_save = APEX_CONFIG0_DMP_POWER_SAVE_DIS; /* Disable power save mode */
	rc |= inv_imu_apex_configure_parameters(&imu_dev, &apex_inputs);

	/* Enable FreeFall */
	rc |= inv_imu_apex_enable_ff(&imu_dev);

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

static float convert_ff_duration_sample_to_cm(uint16_t ff_duration_samples)
{
	uint16_t ff_duration_ms;
	float    ff_distance_cm;

	ff_duration_ms = (uint16_t)(((uint32_t)ff_duration_samples * 2500) / 1000);
	/* dist = speed_average * time = (9.81 * time ^ 2) / 2) */
	ff_distance_cm = (9.81f * ff_duration_ms * ff_duration_ms) / 2 / 10000 /* to cm/s^2 */;

	return ff_distance_cm;
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

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
 * This example showcases how to configure and use Pedometer.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* WOM threshold */
#if INV_IMU_HFSR_SUPPORTED
#define WOM_THRESHOLD 7 /* ~50 mg */
#else
#define WOM_THRESHOLD 13 /* ~50 mg */
#endif

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint8_t           dmp_odr_hz; /* DMP ODR */

/* Static variables for command interface */
static uint8_t use_lp_config; /* Indicates if pedometer is configured in Low Power mode */
static uint8_t use_power_save; /* Indicates if power save mode is being used */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_pedometer();
static void int_cb(void *context, unsigned int int_num);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example Pedometer");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	use_lp_config  = 0;
	use_power_save = 0;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			uint8_t         int_status3;
			uint64_t        timestamp;
			static uint8_t  step_cnt_ovflw = 0;
			static uint64_t step_cnt       = 0;

			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			/* Retrieve timestamp */
			timestamp = int1_timestamp;
			si_enable_irq();

			/* Read Pedometer interrupt status */
			rc |= inv_imu_read_reg(&imu_dev, INT_STATUS3, 1, &int_status3);
			SI_CHECK_RC(rc);

			if (int_status3 & INT_STATUS3_STEP_CNT_OVF_INT_MASK) {
				step_cnt_ovflw++;
				INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   STEP_CNT_OVFL", timestamp);
			}

			if (int_status3 & INT_STATUS3_STEP_DET_INT_MASK) {
				inv_imu_apex_step_activity_t ped_data;
				float                        nb_samples            = 0;
				float                        cadence_steps_per_sec = 0;

				rc |= inv_imu_apex_get_data_activity(&imu_dev, &ped_data);
				SI_CHECK_RC(rc);

				/* Decode `step_candence` into number of samples */
				nb_samples = (float)ped_data.step_cadence / 4;

				/* Compute cadence in steps per sec */
				cadence_steps_per_sec = (float)dmp_odr_hz / nb_samples;

				/* Compute step count */
				step_cnt = ped_data.step_cnt + step_cnt_ovflw * UINT16_MAX;

				INV_MSG(
				    INV_MSG_LEVEL_INFO,
				    "   %10llu us   STEP_DET     count: %5llu steps  cadence: %.1f steps/s  activity: %s",
				    timestamp, step_cnt, cadence_steps_per_sec,
				    ped_data.activity_class == APEX_DATA3_ACTIVITY_CLASS_WALK ? "Walk" :
				    ped_data.activity_class == APEX_DATA3_ACTIVITY_CLASS_RUN  ? "Run" :
                                                                                "Unknown");
			}
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

	return rc;
}

static int setup_imu()
{
	int                           rc = 0;
	inv_imu_serif_t               imu_serif;
	uint8_t                       whoami;
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
	int1_config.INV_STEP_DET      = INV_IMU_ENABLE;
	int1_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int1(&imu_dev, &int1_config);

	/* Disabling FIFO usage to optimize power consumption */
	rc |= inv_imu_configure_fifo(&imu_dev, INV_IMU_FIFO_DISABLED);

	/* Set 2X averaging to minimize power consumption */
	rc |= inv_imu_set_accel_lp_avg(&imu_dev, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);

	/* Enable accel in LP mode */
	rc |= inv_imu_enable_accel_low_power_mode(&imu_dev);

	rc |= configure_pedometer();

	return rc;
}

static int configure_pedometer()
{
	int                       rc = 0;
	inv_imu_apex_parameters_t apex_inputs;

	/* Disable Pedometer before configuring it */
	rc |= inv_imu_apex_disable_pedometer(&imu_dev);

	if (use_lp_config) {
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_25_HZ);
		rc |= inv_imu_apex_set_frequency(&imu_dev, APEX_CONFIG1_DMP_ODR_25Hz);
		dmp_odr_hz = 25;
	} else {
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_50_HZ);
		rc |= inv_imu_apex_set_frequency(&imu_dev, APEX_CONFIG1_DMP_ODR_50Hz);
		dmp_odr_hz = 50;
	}

	/* Set APEX parameters */
	rc |= inv_imu_apex_init_parameters_struct(&imu_dev, &apex_inputs);
	/* Parameters can be modified here if needed */
	apex_inputs.power_save =
	    use_power_save ? APEX_CONFIG0_DMP_POWER_SAVE_EN : APEX_CONFIG0_DMP_POWER_SAVE_DIS;
	rc |= inv_imu_apex_configure_parameters(&imu_dev, &apex_inputs);

	/* If POWER_SAVE mode is enabled, WOM has to be enabled */
	if (use_power_save) {
		/* Configure and enable WOM to wake-up the DMP once it goes in power save mode */
		rc |= inv_imu_configure_wom(&imu_dev, WOM_THRESHOLD, WOM_THRESHOLD, WOM_THRESHOLD,
		                            WOM_CONFIG_WOM_INT_MODE_ANDED, WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_enable_wom(&imu_dev);
	}

	/* Enable Pedometer */
	rc |= inv_imu_apex_enable_pedometer(&imu_dev);

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

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'a': /* Use Low Power or Normal Pedometer configuration */
		use_lp_config = !use_lp_config;
		INV_MSG(INV_MSG_LEVEL_INFO, "Pedometer configured in %s mode.",
		        use_lp_config ? "Low Power" : "Normal");
		rc |= configure_pedometer();
		break;
	case 'p': /* Toggle power-save mode usage */
		use_power_save = !use_power_save;
		INV_MSG(INV_MSG_LEVEL_INFO, "Power-Save mode: %s", use_power_save ? "Enabled" : "Disabled");
		rc |= configure_pedometer();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle Pedometer config (Normal and Low Power)");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'p' : Toggle Power-Save mode usage");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# Pedometer configuration: %s mode",
	        use_lp_config ? "Low Power" : "Normal");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Power-Save: %s", use_power_save ? "Enabled" : "Disabled");
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

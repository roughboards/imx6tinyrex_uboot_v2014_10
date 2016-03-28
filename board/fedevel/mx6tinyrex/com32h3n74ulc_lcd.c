/*
 * com32h3n74ulc.c -- init sequence for Ortustech COM32H3N74ULC
 * adapted from 
 * scf0403.c and spi_display.c from board/boundary/nitrogen6x
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <spi.h>

struct com32h3n74ulc_seq_entry {
	u16 cmd;
	u16 *params;
	int params_len;
	int delay_ms;
};

/* com32h3n74ulc model commands parameters */
// from datasheet power on sequence
static struct com32h3n74ulc_seq_entry en_extended_cmd = {
	0xb9, {0xff, 0x83, 0x63}, 3, 0};
static struct com32h3n74ulc_seq_entry set_power_1_cmd = {
	0xb1, {0x81, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3C, 0x3F, 0x3F}, 12, 0};
static struct com32h3n74ulc_seq_entry sleep_out_cmd = {
	0x11, NULL, 0, 6};
static struct com32h3n74ulc_seq_entry disp_inv_off = {
	0x20, NULL, 0, 0};
static struct com32h3n74ulc_seq_entry mem_acc_ctrl_cmd = {
	0x36, {0x00}, 1, 0};
static struct com32h3n74ulc_seq_entry interface_px_cmd = {
	0x3A, {0x70}, 1, 120};
static struct com32h3n74ulc_seq_entry set_power_2_cmd = {
	0xb1, {0x78, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3C, 0x3F, 0x3F}, 12, 0};
static struct com32h3n74ulc_seq_entry set_rgb_interface_reg_cmd = {
	0xB3, {0x01}, 1, 0};
static struct com32h3n74ulc_seq_entry set_disp_wav_cycle_cmd = {
	0xB4, {0x00, 0x08, 0x56, 0x07, 0x01, 0x01, 0x4D, 0x01, 0x42}, 9, 0}; 
static struct com32h3n74ulc_seq_entry set_panel_cmd = {
	0xCC, {0x0B}, 1, 0};
static struct com32h3n74ulc_seq_entry set_gamma_curve_cmd = {
	0xE0,{0x01, 0x48, 0x4D, 0x4E, 0x58, 0xF6, 0x0B, 0x4E, 0x12, 0xD5, 
		0x15, 0x95, 0x55, 0x8E, 0x11, 0x01, 0x48, 0x4D, 0x55, 0x5F, 
		0xFD, 0x0A, 0x4E, 0x51, 0xD3, 0x17, 0x95, 0x96, 0x4E, 0x11}, 5};
static struct com32h3n74ulc_seq_entry disp_on_cmd = {
	0x29, NULL, 0, 0};

/* com32h3n74ulc init sequence */
static struct com32h3n74ulc_seq_entry com32h3n74ulc_initseq[] = {
	en_extended_cmd,
	set_power_1_cmd,
	sleep_out_cmd,
	disp_inv_off,
	mem_acc_ctrl_cmd,
	interface_px_cmd,
	set_power_2_cmd,
	set_rgb_interface_reg_cmd,
	set_disp_wav_cycle_cmd,
	set_panel_cmd,
	set_gamma_curve_cmd,
	disp_on_cmd,
};

static void com32h3n74ulc_gpio_reset(unsigned int gpio)
{
	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value(gpio, 1);
	mdelay(10);
	gpio_set_value(gpio, 0);
	mdelay(20);
	gpio_set_value(gpio, 1);
	mdelay(50);
}

static int com32h3n74ulc_spi_transfer(struct spi_slave *spi, struct com32h3n74ulc_seq_entry *seq_entry)
{
	int i, error;
	u32 command = seq_entry->cmd;
	u32 msg;

	error = spi_set_wordlen(spi, 9);
	if (error)
		return error;

	// transfer command with DNC = 0
	error = spi_xfer(spi, 9, &command, NULL, SPI_XFER_ONCE);
	if (error)
		return error;

	// transfer parameters with DNC = 1
	for (i = 0; i < seq_entry->params_len; i++) {
		msg = (seq_entry->params[i] | 0x100); // ->DNC = 1
		error = spi_xfer(spi, 9, &msg, NULL, SPI_XFER_ONCE);
		if (error)
			return error;
	}
	
	if (seq_entry->delay>0) mdelay(seq_entry->delay);

	return 0;
}

static void com32h3n74ulc_lcd_init(struct spi_slave *spi)
{
	int i;

	// transfer init sequence
	for (i = 0; i < ARRAY_SIZE(com32h3n74ulc_initseq); i++) {
		if (com32h3n74ulc_spi_transfer(spi, com32h3n74ulc_initseq[i]) < 0)
			puts("SPI transfer failed\n");

		mdelay(priv->init_seq[i].delay_ms);
	}
}


int com32h3n74ulc_init(unsigned reset_gpio, unsigned bus, unsigned cs)
{
	int error;
	struct spi_slave *spi;

	// configure GPIO pin for reset
	if (gpio_is_valid(reset_gpio)) {
		error = gpio_request(gpio, "lcd reset");
		if (error) {
			printf("Failed requesting reset GPIO%d: %d\n",
			       reset_gpio, error);
			return error;
		}
		error = gpio_direction_output(gpio, 0);
		if (error) {
			printf("Failed requesting reset direction GPIO%d: %d\n",
			       reset_gpio, error);
				gpio_free(gpio);
		}
	}

	// init spi
	spi = spi_setup_slave(bus, cs, 1000000, SPI_MODE_2);
	error = spi_claim_bus(spi);
	if (error)
		goto bus_claim_fail;

	/* reset LCD */
	com32h3n74ulc_gpio_reset(reset_gpio);

	/* Start operation by issuing the power on sequence*/
	com32h3n74ulc_lcd_init(spi);

	spi_release_bus(spi);

	return 0;

bus_claim_fail:
	if (gpio_is_valid(reset_gpio))
		gpio_free(reset_gpio);

	return error;
}
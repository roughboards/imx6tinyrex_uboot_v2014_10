/*
 * com32h3n74ulc.c -- init sequence for Ortustech COM32H3N74ULC
 * adapted from 
 * scf0403.c and spi_display.c from board/boundary/nitrogen6x
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <spi.h>

#include "com32h3n74ulc_lcd.h"

#define GPIO_ECSPI2_CS0     IMX_GPIO_NR(2, 26)

struct com32h3n74ulc_cmd {
	u16 cmd;
	u16 *params;
	int params_len;
};

struct com32h3n74ulc_seq_entry {
	struct com32h3n74ulc_cmd cmd;
	int delay_ms;
};

/* com32h3n74ulc model commands parameters */
// from datasheet power on sequence
static u16 en_extended_cmd_params[] = {0xff, 0x83, 0x63};
static u16 set_power_1_cmd_params[] = {0x81, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3C, 0x3F, 0x3F};
static u16 mem_acc_ctrl_cmd_params[] = {0x00};
static u16 interface_px_cmd_params[] = {0x70};
static u16 set_power_2_cmd_params[] = {0x78, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3C, 0x3F, 0x3F};
static u16 set_rgb_interface_reg_cmd_params[] = {0x01};
static u16 set_disp_wav_cycle_cmd_params[] = {0x00, 0x08, 0x56, 0x07, 0x01, 0x01, 0x4D, 0x01, 0x42};
static u16 set_panel_cmd_params[] = {0x0B};
static u16 set_gamma_curve_cmd_params[] = {0x01, 0x48, 0x4D, 0x4E, 0x58, 0xF6, 0x0B, 0x4E, 0x12, 0xD5, 
		0x15, 0x95, 0x55, 0x8E, 0x11, 0x01, 0x48, 0x4D, 0x55, 0x5F, 
		0xFD, 0x0A, 0x4E, 0x51, 0xD3, 0x17, 0x95, 0x96, 0x4E, 0x11};


/*static struct com32h3n74ulc_cmd sleep_out_cmd = {
	0x11, NULL, 0};
static struct com32h3n74ulc_cmd disp_on_cmd = {
	0x29, NULL, 0};
*/

static struct com32h3n74ulc_cmd disp_inv_off = {
	0x20, NULL, 0};
static struct com32h3n74ulc_cmd disp_inv_on = {
	0x21, NULL, 0};

/* com32h3n74ulc init sequence */
static struct com32h3n74ulc_seq_entry com32h3n74ulc_initseq[] = {
	{{0xb9, en_extended_cmd_params, ARRAY_SIZE(en_extended_cmd_params)}, 0},
	{{0xb1, set_power_1_cmd_params, ARRAY_SIZE(set_power_1_cmd_params)}, 0},
	{{0x11, NULL, 0}, 6},
	{{0x20, NULL, 0}, 0},
	{{0x36, mem_acc_ctrl_cmd_params, ARRAY_SIZE(mem_acc_ctrl_cmd_params)}, 0},
	{{0x3A, interface_px_cmd_params, ARRAY_SIZE(interface_px_cmd_params)}, 120},
	{{0xb1, set_power_2_cmd_params, ARRAY_SIZE(set_power_2_cmd_params)}, 0},
	{{0xB3, set_rgb_interface_reg_cmd_params, ARRAY_SIZE(set_rgb_interface_reg_cmd_params)}, 0},
	{{0xB4, set_disp_wav_cycle_cmd_params, ARRAY_SIZE(set_disp_wav_cycle_cmd_params)}, 0},
	{{0xCC, set_panel_cmd_params, ARRAY_SIZE(set_panel_cmd_params)}, 0},
	{{0xE0, set_gamma_curve_cmd_params, ARRAY_SIZE(set_gamma_curve_cmd_params)}, 5},
	{{0x29, NULL, 0}, 0},
};

static int com32h3n74ulc_spi_transfer(struct spi_slave *spi, struct com32h3n74ulc_cmd *cmd)
{
	int i, error;
	u32 command = cmd->cmd;
	u32 msg;

	error = spi_set_wordlen(spi, 9);
	if (error)
		return error;

	// transfer command with DNC = 0
	error = spi_xfer(spi, 9, &command, NULL, SPI_XFER_ONCE);
	if (error)
		return error;

	// transfer parameters with DNC = 1
	for (i = 0; i < cmd->params_len; i++) {
		msg = (cmd->params[i] | 0x100); // ->DNC = 1
		error = spi_xfer(spi, 9, &msg, NULL, SPI_XFER_ONCE);
		if (error)
			return error;
	}
	
	return 0;
}

static void com32h3n74ulc_lcd_init(struct spi_slave *spi)
{
	int i;

	// transfer init sequence
	for (i = 0; i < ARRAY_SIZE(com32h3n74ulc_initseq); i++) {
		if (com32h3n74ulc_spi_transfer(spi, &com32h3n74ulc_initseq[i].cmd) < 0)
			puts("SPI transfer failed\n");

		mdelay(com32h3n74ulc_initseq[i].delay_ms);
	}
}


int com32h3n74ulc_init(unsigned reset_gpio, unsigned bus, unsigned cs)
{
	unsigned cs_gpio = GPIO_ECSPI2_CS0;

	int error;
	struct spi_slave *spi;

	gpio_direction_output(cs_gpio, 1);

	/* reset LCD */
	gpio_direction_output(reset_gpio, 1);
	mdelay(10);
	gpio_direction_output(reset_gpio, 0);
	mdelay(20);
	gpio_direction_output(reset_gpio, 1);
	mdelay(50);

	enable_spi_clk(1, 1);

	// init spi
	spi = spi_setup_slave(bus, cs, 1000000, SPI_MODE_2);
	error = spi_claim_bus(spi);
	if (error)
		return error;


	/* Start operation by issuing the power on sequence*/
	com32h3n74ulc_lcd_init(spi);

	spi_release_bus(spi);

	return 0;
}

static int do_lcdinv(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned cs_gpio = GPIO_ECSPI2_CS0;
	struct spi_slave *spi;
	int ret = 0;
	uint reg;

	if (argc != 2)
		return CMD_RET_USAGE;

	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, 1);

	/* Setup spi_slave */
	// init spi
	spi = spi_setup_slave(1, 0, 1000000, SPI_MODE_2);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	reg = simple_strtoul(argv[1], NULL, 16);
	if (reg == 1) {
		ret = com32h3n74ulc_spi_transfer(spi, &disp_inv_on);
		printf("lcdinv: on");
	} else {
		ret = com32h3n74ulc_spi_transfer(spi, &disp_inv_off);
		printf("lcdinv: off");
	}

	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, 1);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	lcdinv, 70, 0, do_lcdinv,
	"read spi display register",
	"reg16"
);


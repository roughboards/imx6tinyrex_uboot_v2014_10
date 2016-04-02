/*
 * Copyright (C) 2015 Voipac, Inc.
 *
 * Author: support <support@voipac.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <splash.h>

#include "com32h3n74ulc_lcd.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP  |                     \
                        PAD_CTL_SPEED_HIGH  | PAD_CTL_DSE_80ohm | \
                        PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_HYS)

#define SPI_PAD_CTRL   (PAD_CTL_HYS         |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL   (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_HYS         | PAD_CTL_ODE       | \
                        PAD_CTL_SRE_FAST)

#define RST_PAD_CTRL   (PAD_CTL_ODE         | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb_pads[] = {
	IOMUX_PAD_CTRL(EIM_A16__IPU1_DI1_DISP_CLK, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA10__IPU1_DI1_PIN15, RGB_PAD_CTRL),	/* DRDY */
	IOMUX_PAD_CTRL(EIM_DA11__IPU1_DI1_PIN02, RGB_PAD_CTRL),		/* HSYNC */
	IOMUX_PAD_CTRL(EIM_DA12__IPU1_DI1_PIN03, RGB_PAD_CTRL),		/* VSYNC */
	IOMUX_PAD_CTRL(EIM_DA9__IPU1_DISP1_DATA00, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA8__IPU1_DISP1_DATA01, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA7__IPU1_DISP1_DATA02, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA6__IPU1_DISP1_DATA03, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA5__IPU1_DISP1_DATA04, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA4__IPU1_DISP1_DATA05, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA3__IPU1_DISP1_DATA06, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA2__IPU1_DISP1_DATA07, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA1__IPU1_DISP1_DATA08, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_DA0__IPU1_DISP1_DATA09, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_EB1__IPU1_DISP1_DATA10, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_EB0__IPU1_DISP1_DATA11, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A17__IPU1_DISP1_DATA12, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A18__IPU1_DISP1_DATA13, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A19__IPU1_DISP1_DATA14, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A20__IPU1_DISP1_DATA15, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A21__IPU1_DISP1_DATA16, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A22__IPU1_DISP1_DATA17, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A23__IPU1_DISP1_DATA18, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_A24__IPU1_DISP1_DATA19, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D31__IPU1_DISP1_DATA20, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D30__IPU1_DISP1_DATA21, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D26__IPU1_DISP1_DATA22, RGB_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__IPU1_DISP1_DATA23, RGB_PAD_CTRL),
	#define GPIO_LCD_RSTB    IMX_GPIO_NR(2, 11)
	IOMUX_PAD_CTRL(SD4_DAT3__GPIO2_IO11, WEAK_PULLUP),

};

static const iomux_v3_cfg_t rgb_gpio_pads[] = {
	IOMUX_PAD_CTRL(EIM_A16__GPIO2_IO22, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA10__GPIO3_IO10, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA11__GPIO3_IO11, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA12__GPIO3_IO12, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA9__GPIO3_IO09, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA8__GPIO3_IO08, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA7__GPIO3_IO07, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA6__GPIO3_IO06, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA5__GPIO3_IO05, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA4__GPIO3_IO04, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_DA0__GPIO3_IO00, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_EB1__GPIO2_IO29, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_EB0__GPIO2_IO28, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A18__GPIO2_IO20, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A19__GPIO2_IO19, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A20__GPIO2_IO18, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A21__GPIO2_IO17, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A22__GPIO2_IO16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A23__GPIO6_IO06, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_A24__GPIO5_IO04, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_D31__GPIO3_IO31, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_D30__GPIO3_IO30, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_D26__GPIO3_IO26, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_D27__GPIO3_IO27, WEAK_PULLUP),
};
#endif


int dram_init(void)
{
        gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

        return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
        MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads1[] = {
        MX6_PAD_ENET_MDIO__ENET_MDIO       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_ENET_MDC__ENET_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TXC__RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD0__RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD1__RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD2__RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD3__RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RXC__GPIO6_IO30      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* PHYAD2  = 0 */
        MX6_PAD_RGMII_RD0__GPIO6_IO25      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE0  = 1 */
        MX6_PAD_RGMII_RD1__GPIO6_IO27      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE1  = 1 */
        MX6_PAD_RGMII_RD2__GPIO6_IO28      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE2  = 1 */
        MX6_PAD_RGMII_RD3__GPIO6_IO29      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE3  = 1 */
        MX6_PAD_RGMII_RX_CTL__GPIO6_IO24   | MUX_PAD_CTRL(ENET_PAD_CTRL), /* CLK125_EN = 1 */
        /* KSZ9021 PHY Int */
        MX6_PAD_ENET_TX_EN__GPIO1_IO28     | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_ENET_RXD1__GPIO1_IO26      | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* KSZ9021 PHY Reset */
        MX6_PAD_ENET_CRS_DV__GPIO1_IO25    | MUX_PAD_CTRL(RST_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
        MX6_PAD_RGMII_RXC__RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

#define GPIO_ENET_INT_1         IMX_GPIO_NR(1, 28)
#define GPIO_ENET_INT_2         IMX_GPIO_NR(1, 26)
#define GPIO_ENET_RESET         IMX_GPIO_NR(1, 25)
#define GPIO_ENET_CFG_PHYAD2    IMX_GPIO_NR(6, 30)
#define GPIO_ENET_CFG_MODE0     IMX_GPIO_NR(6, 25)
#define GPIO_ENET_CFG_MODE1     IMX_GPIO_NR(6, 27)
#define GPIO_ENET_CFG_MODE2     IMX_GPIO_NR(6, 28)
#define GPIO_ENET_CFG_MODE3     IMX_GPIO_NR(6, 29)
#define GPIO_ENET_CFG_CLK125_EN IMX_GPIO_NR(6, 24)


static void setup_iomux_enet(void)
{
        imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

        /* KSZ9021 PHY Cfg */
        gpio_direction_output(GPIO_ENET_CFG_PHYAD2,    0);
        gpio_direction_output(GPIO_ENET_CFG_MODE0,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE1,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE2,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE3,     1);
        gpio_direction_output(GPIO_ENET_CFG_CLK125_EN, 1);

        /* KSZ9021 PHY Int */
        gpio_direction_input(GPIO_ENET_INT_1);
        gpio_direction_input(GPIO_ENET_INT_2);

        /* KSZ9021 PHY Reset */
        gpio_direction_output(GPIO_ENET_RESET, 0);
        udelay(10000);
        gpio_set_value(GPIO_ENET_RESET, 1);
        udelay(100);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

iomux_v3_cfg_t const usdhc1_pads[] = {
        MX6_PAD_SD1_CLK__SD1_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD1_CMD__SD1_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD1_DAT0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD1_DAT1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD1_DAT2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD1_DAT3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_D0__SD1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_D1__SD1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_D2__SD1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_D3__SD1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_CLK__GPIO1_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
        MX6_PAD_SD2_CMD__GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
};

iomux_v3_cfg_t const usdhc3_pads[] = {
        MX6_PAD_SD3_CLK__SD3_CLK      | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_CMD__SD3_CMD      | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT0__SD3_DATA0   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT1__SD3_DATA1   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT2__SD3_DATA2   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT3__SD3_DATA3   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_RST__GPIO7_IO08   | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
        MX6_PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
        MX6_PAD_SD4_CLK__SD4_CLK      | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_CMD__SD4_CMD      | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT0__SD4_DATA0   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT1__SD4_DATA1   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT2__SD4_DATA2   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT3__SD4_DATA3   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT4__SD4_DATA4   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT5__SD4_DATA5   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT6__SD4_DATA6   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_DAT7__SD4_DATA7   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
        MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
};

iomux_v3_cfg_t const ecspi1_pads[] = {
        MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_EB2__GPIO2_IO30  | MUX_PAD_CTRL(SPI_PAD_CTRL), /* CS0 */
};

static const iomux_v3_cfg_t ecspi2_pads[] = {
	IOMUX_PAD_CTRL(EIM_CS0__GPIO2_IO23, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_OE__GPIO2_IO25, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_CS1__GPIO2_IO24, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT15__GPIO5_IO09, WEAK_PULLUP),

};
// iomux_v3_cfg_t const ecspi2_pads[] = {
//         MX6_PAD_EIM_CS0__ECSPI2_SCLK    | MUX_PAD_CTRL(SPI_PAD_CTRL),
//         MX6_PAD_EIM_OE__ECSPI2_MISO     | MUX_PAD_CTRL(SPI_PAD_CTRL),
//         MX6_PAD_EIM_CS1__ECSPI2_MOSI    | MUX_PAD_CTRL(SPI_PAD_CTRL),
//         MX6_PAD_EIM_RW__GPIO2_IO26      | MUX_PAD_CTRL(SPI_PAD_CTRL), /* CS0 */
//         MX6_PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(SPI_PAD_CTRL), /* CS1 */
// };

#define GPIO_ECSPI1_CS0     IMX_GPIO_NR(2, 30)
#define GPIO_ECSPI2_CS0     IMX_GPIO_NR(2, 26)
#define GPIO_ECSPI2_CS1     IMX_GPIO_NR(5, 9)
#define GPIO_ECSPI2_SCK     IMX_GPIO_NR(2, 23)
#define GPIO_ECSPI2_MOSI     IMX_GPIO_NR(2, 24)


static void setup_spi(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        enable_spi_clk(true, 0);
//        imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
//        enable_spi_clk(true, 1);
		SETUP_IOMUX_PADS(ecspi2_pads);
		gpio_direction_output(GPIO_ECSPI2_CS0, 1);
		gpio_direction_output(GPIO_ECSPI2_CS1, 1);
		gpio_direction_output(GPIO_ECSPI2_SCK, 1);
		gpio_direction_output(GPIO_ECSPI2_MOSI, 0);
		
}

static struct i2c_pads_info i2c0_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(3, 21)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_EIM_D28__I2C1_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(3, 28)
        }
};

static struct i2c_pads_info i2c1_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_KEY_COL3__I2C2_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(4, 12)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_KEY_ROW3__I2C2_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(4, 13)
        }
};

static struct i2c_pads_info i2c2_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_GPIO_5__I2C3_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(1, 5)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_GPIO_16__I2C3_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(7, 11)
        }
};

iomux_v3_cfg_t const pcie_pads[] = {
        MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* WAKE */
        MX6_PAD_EIM_A25__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* RESET */
};

static void setup_pcie(void)
{
        imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM];

#define GPIO_USDHC1_CD     IMX_GPIO_NR(1, 10)
#define GPIO_USDHC3_CD     IMX_GPIO_NR(7, 8)
#define GPIO_USDHC4_CD     IMX_GPIO_NR(6, 11)

int board_mmc_getcd(struct mmc *mmc)
{
        struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
        int ret = 0;

        switch (cfg->esdhc_base) {
        case USDHC1_BASE_ADDR:
                ret = !gpio_get_value(GPIO_USDHC1_CD);
                break;
        case USDHC3_BASE_ADDR:
                //ret = !gpio_get_value(GPIO_USDHC3_CD);
                ret = 1; /* Is always present */
                break;
        case USDHC4_BASE_ADDR:
                ret = !gpio_get_value(GPIO_USDHC4_CD);
                break;
        }

        return ret;
}

int board_mmc_init(bd_t *bis)
{
        s32 status = 0;
        int i;

        /*
         * According to the board_mmc_init() the following map is done:
         * (U-boot device node)    (Physical Port)
         * mmc0                    SD3 Primary
         * mmc1                    SD1
         * mmc2                    SD4
         */
        for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
                switch (i) {
                case 0:
                        imx_iomux_v3_setup_multiple_pads(
                                usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
                        gpio_direction_input(GPIO_USDHC3_CD);
                        usdhc_cfg[0].esdhc_base    = USDHC3_BASE_ADDR;
                        usdhc_cfg[0].sdhc_clk      = mxc_get_clock(MXC_ESDHC3_CLK);
                        usdhc_cfg[0].max_bus_width = 4;
                        break;
                case 1:
                        imx_iomux_v3_setup_multiple_pads(
                                usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
                        gpio_direction_input(GPIO_USDHC1_CD);
                        usdhc_cfg[1].esdhc_base    = USDHC1_BASE_ADDR;
                        usdhc_cfg[1].sdhc_clk      = mxc_get_clock(MXC_ESDHC_CLK);
                        usdhc_cfg[1].max_bus_width = 8;
                        break;
//                 case 2:
//                         imx_iomux_v3_setup_multiple_pads(
//                                 usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
//                         gpio_direction_input(GPIO_USDHC4_CD);
//                         usdhc_cfg[2].esdhc_base    = USDHC4_BASE_ADDR;
//                         usdhc_cfg[2].sdhc_clk      = mxc_get_clock(MXC_ESDHC4_CLK);
//                         usdhc_cfg[2].max_bus_width = 8;
//                         break;
                default:
                        printf("Warning: you configured more USDHC controllers"
                               "(%d) then supported by the board (%d)\n",
                               i + 1, CONFIG_SYS_FSL_USDHC_NUM);
                        return status;
                }

                status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

        return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
        /* min rx data delay */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
        /* min tx data delay */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
        /* max rx/tx clock delay, min rx/tx control */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
        mx6_rgmii_rework(phydev);

        if (phydev->drv->config)
        {
               phydev->drv->config(phydev);
        }

        return 0;
}


#ifdef CONFIG_CMD_FBPANEL
void board_enable_lvds(const struct display_info_t *di, int enable)
{
//	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable) {
		SETUP_IOMUX_PADS(rgb_pads);
		com32h3n74ulc_init(GPIO_LCD_RSTB, 1, 0);
		mdelay(100); /* let panel sync up before enabling backlight */
		// TODO enable backlight here
		
	} else {
		// TODO disable backlight here
		
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	}
}
#define IMX_VD_SPI_QVGA(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= NULL,\
	.enable	= board_enable_lcd,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR | FBF_SPI,\
	.mode	= {\
		.name           = "qvga",\
		.refresh        = 75,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 32552,\
		.left_margin    = 10,\
		.right_margin   = 8,\
		.upper_margin   = 2,\
		.lower_margin   = 2,\
		.hsync_len      = 10,\
		.vsync_len      = 2,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

static const struct display_info_t displays[] = {
	IMX_VD_SPI_QVGA(LCD, 1, 1),
};

#endif /* CONFIG_CMD_FBPANEL */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
        return 1;
}

int board_eth_init(bd_t *bis)
{
        setup_iomux_enet();

        setup_pcie();

        return cpu_eth_init(bis);
}

void clear_pcie_on_wdog_reset(void)
{
        u32 cause;
        struct src *const src_regs = (struct src *)SRC_BASE_ADDR;
        struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

        cause = readl(&src_regs->srsr);
        if (cause == 0x00010)
        {
                clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);
                clrbits_le32(&iomuxc_regs->gpr[12], IOMUXC_GPR12_APPS_LTSSM_ENABLE);
        }
}

int board_early_init_f(void)
{
        setup_iomux_uart();

        clear_pcie_on_wdog_reset();

        return 0;
}

int board_init(void)
{
        /* address of boot parameters */
        gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

        setup_spi();

#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif

        setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c0_pad_info);
        setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c1_pad_info);
        setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c2_pad_info);

#if defined(CONFIG_CMD_SATA) && defined(CONFIG_MX6Q)
        setup_sata();
#endif

        return 0;
}

#ifdef CONFIG_MXC_SPI


int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        int ret = -1;

        if (bus == 0 && cs == 0) ret = GPIO_ECSPI1_CS0;
        if (bus == 1 && cs == 0) ret = GPIO_ECSPI2_CS0;
        if (bus == 1 && cs == 1) ret = GPIO_ECSPI2_CS1;

        return ret;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
        /* 4 bit bus width */
        {"mmc0", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)}, // SD3
        {"mmc1", MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)}, // SD1
        {"mmc2", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)}, // SD4
        {NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
        add_board_boot_modes(board_boot_modes);
#endif

        return 0;
}

int checkboard(void)
{
        puts("Board: MX6 TinyRex - " CONFIG_MODULE_TYPE_POSTFIX "\n");
        return 0;
}

/*
 * Copyright (C) 2015 Voipac, Inc.
 *
 * Author: support <support@voipac.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <micrel.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
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

int dram_init(void)
{
        gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

        return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
        MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
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

iomux_v3_cfg_t const usdhc3_pads[] = {
        MX6_PAD_SD3_CLK__SD3_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_CMD__SD3_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT0__SD3_DATA0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT1__SD3_DATA1  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT2__SD3_DATA2  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD3_DAT3__SD3_DATA3  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const ecspi3_pads[] = {
        MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
//        MX6_PAD_DISP0_DAT5__ECSPI3_SS2  | MUX_PAD_CTRL(SPI_PAD_CTRL), /* CS2 */
        MX6_PAD_DISP0_DAT5__GPIO4_IO26  | MUX_PAD_CTRL(SPI_PAD_CTRL), /* CS2 */
};

static void setup_spi(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM];

#define GPIO_USDHC3_CD     IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
        struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
        int ret = 0;

        switch (cfg->esdhc_base) {
        case USDHC3_BASE_ADDR:
                ret = !gpio_get_value(GPIO_USDHC3_CD);
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

#if defined(CONFIG_VIDEO_IPUV3)
static void enable_hdmi(struct display_info_t const *dev)
{
        imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {
        {
                .bus	= -1,
                .addr	= 0,
                .pixfmt	= IPU_PIX_FMT_RGB24,
                .detect	= detect_hdmi,
                .enable	= enable_hdmi,
                .mode	= {
                        .name           = "HDMI",
                        .refresh        = 60,
                        .xres           = 1024,
                        .yres           = 768,
                        .pixclock       = 15385,
                        .left_margin    = 220,
                        .right_margin   = 40,
                        .upper_margin   = 21,
                        .lower_margin   = 7,
                        .hsync_len      = 60,
                        .vsync_len      = 10,
                        .sync           = FB_SYNC_EXT,
                        .vmode          = FB_VMODE_NONINTERLACED
                }
        }
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
        struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
        int reg;

        enable_ipu_clock();
        imx_setup_hdmi();

        /* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
        reg = readl(&mxc_ccm->CCGR3);
        reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
        writel(reg, &mxc_ccm->CCGR3);

        /* set LDB0, LDB1 clk select to 011/011 */
        reg = readl(&mxc_ccm->cs2cdr);
        reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
                 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
        reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
               | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->cs2cdr);

        reg = readl(&mxc_ccm->cscmr2);
        reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
        writel(reg, &mxc_ccm->cscmr2);

        reg = readl(&mxc_ccm->chsccdr);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0
                << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0
                << MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->chsccdr);

        reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
             | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
             | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
             | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
             | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
             | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
             | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
             | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
             | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
        writel(reg, &iomux->gpr[2]);

        reg = readl(&iomux->gpr[3]);
        reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
                       | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
               | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
               << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
        writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

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

        return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
        setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
        setup_display();
#endif

        return 0;
}

int board_init(void)
{
        /* address of boot parameters */
        gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
        setup_spi();
#endif

#if defined(CONFIG_CMD_SATA) && defined(CONFIG_MX6Q)
        setup_sata();
#endif

        return 0;
}

#ifdef CONFIG_MXC_SPI

#define GPIO_ECSPI3_CS2     IMX_GPIO_NR(4, 26)

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        int ret = -1;

        if (bus == 2 && cs == 2) ret = GPIO_ECSPI3_CS2;

        return ret;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
        /* 4 bit bus width */
        {"mmc0", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)}, // SD3
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
        puts("Board: MX6 Rex\n");
        return 0;
}

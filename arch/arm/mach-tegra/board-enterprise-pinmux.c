/*
 * arch/arm/mach-tegra/board-enterprise-pinmux.c
 *
 * Copyright (C) 2011-2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include "board.h"
#include "board-enterprise.h"
#include "gpio-names.h"

typedef struct {
	char name[16];
	int gpio_nr;
	bool is_gpio;
	bool is_input;
	int value; /* Value if it is output*/
}tegra_gpio_init_pin_info;

static void tegra_pinmux_gpio_init(tegra_gpio_init_pin_info *gpio_init_table, int list_count);

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

static __initdata struct tegra_drive_pingroup_config m470_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */
	SET_DRIVE(DAP2, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(DAP1, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	
	/* UART2 */
	SET_DRIVE(UART2,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* SDMMC1 */
	SET_DRIVE(SDIO1,	DISABLE, DISABLE, DIV_1, 46, 42, FAST, FAST),

	/* DEV3CFG */
	SET_DRIVE(DEV3,  DISABLE, DISABLE, DIV_4, 18, 22, SLOWEST, SLOWEST),
};

struct pin_info_low_power_mode {
	char name[16];
	int gpio_nr;
	bool is_gpio;
	bool is_input;
	int value; /* Value if it is output*/
};

#define PIN_GPIO_INIT(_name, _gpio, _is_input, _value)	\
	{					\
		.name		= _name,	\
		.gpio_nr	= _gpio,	\
		.is_gpio	= true,		\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define CEC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

static __initdata struct tegra_pingroup_config m470_pinmux_common[] = {
	// Audio
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      NORMAL,    NORMAL,     INPUT),
        //DEFAULT_PINMUX(CLK1_REQ,        DAP,             NORMAL,    TRISTATE,     INPUT),        
	DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    NORMAL,     INPUT),	
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,           NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,    NORMAL,  INPUT),
	DEFAULT_PINMUX(SPI1_MISO,       SPI1,            NORMAL, NORMAL,   INPUT),
	DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            NORMAL, NORMAL,   INPUT),
	DEFAULT_PINMUX(SPI1_SCK,        SPI1,            NORMAL, NORMAL,   INPUT),
	DEFAULT_PINMUX(SPI2_CS0_N,      SPI2,            NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            NORMAL,   NORMAL,     OUTPUT),
	DEFAULT_PINMUX(SPI2_CS2_N,      SPI2,            NORMAL,   NORMAL,     INPUT), //gpio CDC_IRQ_N
	DEFAULT_PINMUX(SPI2_MISO,       SPI2,            NORMAL,    NORMAL,  OUTPUT), //gpio CAM_PWR_EN
	DEFAULT_PINMUX(SPI2_MOSI,       SPI2,            NORMAL,    NORMAL,  OUTPUT), //gpio EN_CODEC_PA
	DEFAULT_PINMUX(SPI2_SCK,        SPI2,            NORMAL,    NORMAL,  INPUT), //gpio GYRO_IRQ_N

	//BB
	DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL,    NORMAL,   OUTPUT), //gpio EN_VDD_SDMMC1
	DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            NORMAL,    NORMAL,   OUTPUT), //gpio EN_VDDIO_VID_OC_N
	DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,   OUTPUT),	//gpio GPS_PWN
	DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            NORMAL,    NORMAL,   OUTPUT), //gpio NFC_PWN
	DEFAULT_PINMUX(GPIO_PV0,        RSVD,            NORMAL,   NORMAL,   INPUT), //gpio AP_ONKEY_N
	DEFAULT_PINMUX(GPIO_PV1,        RSVD,            NORMAL,   NORMAL,   OUTPUT), //gpio CAM_AVDD_PWR_EN
	
	DEFAULT_PINMUX(ULPI_CLK,        ULPI,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA0,      ULPI,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA1,      ULPI,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA2,      ULPI,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA3,      ULPI,           NORMAL,    NORMAL,     INPUT),//gpio EARHOOK_DET_N
	DEFAULT_PINMUX(ULPI_DATA4,      SPI2,           NORMAL,    NORMAL,     INPUT),//gpio HP_DET_N
	DEFAULT_PINMUX(ULPI_DATA5,      SPI2,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA6,      SPI2,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA7,      SPI2,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DIR,        ULPI,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(ULPI_NXT,        ULPI,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(ULPI_STP,        ULPI,            NORMAL,    TRISTATE,     OUTPUT),

	//Camera
	I2C_PINMUX(CAM_I2C_SCL,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(CAM_I2C_SDA,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,         NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PBB0,       I2S4,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB3,       VGP3,            NORMAL,    NORMAL,     OUTPUT), //gpio CAM_AF_PWDN_N
	DEFAULT_PINMUX(GPIO_PBB4,       VGP4,            NORMAL,    TRISTATE,   OUTPUT), 
	DEFAULT_PINMUX(GPIO_PBB5,       DISPLAYA,            NORMAL,    NORMAL,     OUTPUT), //gpio FRONT_CAM_PWDN
	DEFAULT_PINMUX(GPIO_PBB6,       DISPLAYA,            NORMAL,    NORMAL,     OUTPUT), //gpio FRONT_CAM_RST
	DEFAULT_PINMUX(GPIO_PBB7,       RSVD1,            NORMAL,    NORMAL,     OUTPUT), //gpio FRONT_CAM_PWDN
	DEFAULT_PINMUX(GPIO_PCC1,       RSVD1,            NORMAL,    NORMAL,   OUTPUT),//gpio CAM_RST_N
	DEFAULT_PINMUX(GPIO_PCC2,       I2S4,            NORMAL,    TRISTATE,     OUTPUT),

	//GMI
	I2C_PINMUX(GEN2_I2C_SCL,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
        DEFAULT_PINMUX(GMI_A16,         UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_A17,         UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_A18,         UARTD,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(GMI_A19,         UARTD,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(GMI_AD0,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD1,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD10,        RSVD2,           NORMAL,    NORMAL,     OUTPUT), //gpio LCD_BL_EN
	DEFAULT_PINMUX(GMI_AD11,        RSVD2,            NORMAL,    NORMAL,  OUTPUT), //gpio EN_VDD_BL
	DEFAULT_PINMUX(GMI_AD12,        RSVD1,           NORMAL,    NORMAL,   OUTPUT), //gpio EN_VDD_FUSE
	DEFAULT_PINMUX(GMI_AD13,        RSVD1,           NORMAL,    NORMAL,   OUTPUT), //gpio EN_LCD_1V8 
	DEFAULT_PINMUX(GMI_AD14,        RSVD1,            NORMAL,    NORMAL,   OUTPUT), //gpio TP_LP0
	DEFAULT_PINMUX(GMI_AD15,        RSVD1,            NORMAL,    TRISTATE,   OUTPUT), 
	DEFAULT_PINMUX(GMI_AD2,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD3,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD4,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD5,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD6,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD7,         GMI,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_AD8,         PWM0,             NORMAL,       NORMAL,  OUTPUT), 
	DEFAULT_PINMUX(GMI_AD9,         GMI,             NORMAL,       TRISTATE,  OUTPUT), 
	DEFAULT_PINMUX(GMI_ADV_N,       RSVD1,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CLK,         RSVD1,             NORMAL,       TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CS0_N,       GMI,             NORMAL,    TRISTATE,  OUTPUT), 
	DEFAULT_PINMUX(GMI_CS1_N,       GMI,             NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CS2_N,       GMI,             NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CS3_N,       GMI,             NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CS4_N,       RSVD1,             NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_CS6_N,       GMI,             NORMAL,    TRISTATE,  OUTPUT), 
	DEFAULT_PINMUX(GMI_CS7_N,       GMI,             NORMAL,    NORMAL,  INPUT), //gpio  PMU_CHRG_DET
	DEFAULT_PINMUX(GMI_DQS,         GMI,           NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GMI_IORDY,       GMI,           NORMAL,   NORMAL,   INPUT), //gpio SDMMC_CD_N
	DEFAULT_PINMUX(GMI_OE_N,       GMI,           NORMAL,   TRISTATE,   OUTPUT), 
	DEFAULT_PINMUX(GMI_RST_N,       GMI,           NORMAL,   TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_WAIT,       GMI,           NORMAL,   TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_WP_N,       RSVD1,           NORMAL,   TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_WR_N,       RSVD1,           NORMAL,   TRISTATE,   OUTPUT),

        //LCD
        DEFAULT_PINMUX(CRT_HSYNC,       CRT,             NORMAL,    TRISTATE,  OUTPUT),
        DEFAULT_PINMUX(CRT_VSYNC,       CRT,             NORMAL,    TRISTATE,  OUTPUT),
        I2C_PINMUX(DDC_SCL,		I2C4,		NORMAL,NORMAL,	INPUT,	DISABLE,	DISABLE),
        I2C_PINMUX(DDC_SDA,		I2C4,		NORMAL,NORMAL,	INPUT,	DISABLE,	DISABLE),
        DEFAULT_PINMUX(HDMI_INT,        RSVD0,           NORMAL,    NORMAL,   INPUT), //gpio HDMI_HPD pullup&down?
        DEFAULT_PINMUX(LCD_CS0_N,       RSVD,        NORMAL,    NORMAL,     OUTPUT), //gpio COMPASS_RST_N
        DEFAULT_PINMUX(LCD_CS1_N,       RSVD2,        NORMAL,       NORMAL,     INPUT), //gpio COMPASS_DRDY pullup&down?
        DEFAULT_PINMUX(LCD_DE,            DISPLAYA,       NORMAL,   NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_HSYNC,    DISPLAYA,       NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_VSYNC,    DISPLAYA,       NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,    NORMAL,  INPUT),
        DEFAULT_PINMUX(LCD_D0,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D1,           DISPLAYA,        NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D2,           DISPLAYA,        NORMAL,    NORMAL,     INPUT),       
        DEFAULT_PINMUX(LCD_D3,           DISPLAYA,        NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D4,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D5,           DISPLAYA,        NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D6,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D7,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D8,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D9,           DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D14,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D15,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,    NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        NORMAL,       NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,       NORMAL,   INPUT), 
        DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL,       NORMAL,   INPUT),
        DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        NORMAL,       NORMAL,   INPUT), 
        DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        NORMAL,       NORMAL,     INPUT),
        DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        NORMAL,       NORMAL,     INPUT), 
        DEFAULT_PINMUX(LCD_DC0,         RSVD1,        NORMAL,       NORMAL,   OUTPUT), //gpio LVDS_SHTDN_N
        DEFAULT_PINMUX(LCD_DC1,         RSVD2,        NORMAL,       NORMAL,   OUTPUT), //gpio BT_REG_ON
        DEFAULT_PINMUX(LCD_M1,          RSVD1,        NORMAL,    NORMAL,     OUTPUT), //gpio EN_VDD_PNL
        DEFAULT_PINMUX(LCD_PWR0,        DISPLAYB,        NORMAL,    NORMAL,     OUTPUT), //gpio RST_CDC
        DEFAULT_PINMUX(LCD_PWR1,        RSVD1,        PULL_DOWN,    NORMAL,     OUTPUT),//gpio PMU_MSECURE
        DEFAULT_PINMUX(LCD_PWR2,        DISPLAYB,        NORMAL,    NORMAL,     OUTPUT),//gpio EN_LCD_3V3
        DEFAULT_PINMUX(LCD_SCK,        DISPLAYA,        NORMAL,    TRISTATE,  OUTPUT),	
        DEFAULT_PINMUX(LCD_SDIN,        RSVD,        NORMAL,    NORMAL,  INPUT),  //gpio ALS_IRQ_N
        DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYB,        PULL_DOWN,    NORMAL,  OUTPUT), //gpio TS_RESET_N
        DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,    NORMAL,  INPUT),//gpio TS_IRQ_N

        //PEX_CTL
        DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,     OUTPUT),
        DEFAULT_PINMUX(PEX_L0_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L0_RST_N,    PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,     OUTPUT),
        DEFAULT_PINMUX(PEX_L1_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L1_RST_N,    PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L2_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(PEX_WAKE_N,      PCIE,            NORMAL,    TRISTATE,     OUTPUT),

	//SDMMC1
	DEFAULT_PINMUX(CLK2_OUT,       RSVD1,       NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(CLK2_REQ,       RSVD1,              NORMAL,    TRISTATE,  OUTPUT),
	DEFAULT_PINMUX(GPIO_PV2,        RSVD1,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           NORMAL,    TRISTATE,     OUTPUT),

	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),

	//SDMMC3
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),

	DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(SDMMC3_DAT5,     SDMMC3,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          NORMAL,    NORMAL,     OUTPUT), //gpio WLAN_RST_N
	DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          NORMAL,    NORMAL,     OUTPUT), //gpio BT_RST_N

        //SDMMC4
        DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,     NORMAL,     INPUT),
        DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_RST_N,    POPSDMMC4,           NORMAL,    NORMAL,     INPUT),

        //SYS
        DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL, NORMAL,   OUTPUT),
        //CLK_32K_IN
        //CORE_PWR_REQ
        //CPU_PWR_REQ
        //DEFAULT_PINMUX(HDMI_CEC,     RSVD1,           PULL_DOWN, NORMAL,   OUTPUT), //used ???????
        DEFAULT_PINMUX(JTAG_RTCK,       RTCK,            NORMAL,    NORMAL,     INPUT),
	//JTAG_TRST_N
	//JTAG_TDO
	//JTAG_TMS
	//JTAG_TCK
	//JTAG_TDI
	DEFAULT_PINMUX(KB_COL0,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_COL1,         KBC,             PULL_UP,   NORMAL,     INPUT), //gpio VOL_UP
	DEFAULT_PINMUX(KB_COL2,         RSVD,             PULL_UP,   NORMAL,     INPUT), //gpio VOL_DOWN
	DEFAULT_PINMUX(KB_COL3,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_COL4,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_COL5,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_COL6,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_COL7,         KBC,             NORMAL,   NORMAL,     OUTPUT), //gpio GPS_RST_N
	DEFAULT_PINMUX(KB_ROW0,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW1,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW10,        KBC,             NORMAL,    TRISTATE,     OUTPUT), 
	DEFAULT_PINMUX(KB_ROW11,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW12,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW13,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW14,         KBC,             NORMAL,   NORMAL,     INPUT), //gpio TEMP_ALERT_N
	DEFAULT_PINMUX(KB_ROW15,         KBC,             NORMAL,   NORMAL,     OUTPUT), //gpio NFC_WAKE
	DEFAULT_PINMUX(KB_ROW2,         KBC,             NORMAL,   NORMAL,     OUTPUT), //gpio CAM_FLASH
	DEFAULT_PINMUX(KB_ROW3,         KBC,             NORMAL,   NORMAL,     OUTPUT), //gpio CAM_TORCH
	DEFAULT_PINMUX(KB_ROW4,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW5,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW6,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW7,         KBC,             NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW8,         KBC,             NORMAL,   NORMAL,     INPUT), //gpio  WLAN_HOST_WAKE
	DEFAULT_PINMUX(KB_ROW9,         KBC,             NORMAL,   NORMAL,     OUTPUT), //gpio BATREMOVAL
	DEFAULT_PINMUX(OWR,             OWR,             NORMAL,    TRISTATE,     OUTPUT),
	//PWR_INT_N
	I2C_PINMUX(PWR_I2C_SCL,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(PWR_I2C_SDA,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          NORMAL,    TRISTATE,     OUTPUT),
	//SYS_RESET_N
	//TEST_MODE_EN

        //UART
        DEFAULT_PINMUX(CLK3_OUT,       EXTPERIPH3,       NORMAL,    NORMAL,  INPUT),
	DEFAULT_PINMUX(CLK3_REQ,       RSVD1,            NORMAL,    TRISTATE,  OUTPUT),
	
	DEFAULT_PINMUX(DAP4_DIN,        I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_DOUT,       I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_FS,         I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_SCLK,       I2S3,            NORMAL,    NORMAL,     INPUT),

	I2C_PINMUX(GEN1_I2C_SCL,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN1_I2C_SDA,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	DEFAULT_PINMUX(GPIO_PU0,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),// gpio NRESWARM
	DEFAULT_PINMUX(GPIO_PU1,        RSVD1,           NORMAL,    NORMAL,       OUTPUT),//gpio HOST_BT_WAKE
	DEFAULT_PINMUX(GPIO_PU2,        UARTA,           NORMAL,    TRISTATE,       OUTPUT),
	DEFAULT_PINMUX(GPIO_PU3,        UARTA,           NORMAL,    NORMAL,     OUTPUT),//gpio FACTORY_LED

	DEFAULT_PINMUX(GPIO_PU4,        RSVD1,           NORMAL,    TRISTATE,     OUTPUT), 
	DEFAULT_PINMUX(GPIO_PU5,        RSVD1,           NORMAL,    NORMAL,     INPUT), //gpio NFC_IRQ
	DEFAULT_PINMUX(GPIO_PU6,        RSVD1,           NORMAL,    NORMAL,     INPUT), //gpio BT_HOST_WAKE

        DEFAULT_PINMUX(UART2_CTS_N,     UARTB,           NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(UART2_RTS_N,     UARTB,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_RXD,       IRDA,            NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(UART2_TXD,       IRDA,            NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,    NORMAL,     INPUT),

	//VI
	DEFAULT_PINMUX(VI_D0,           RSVD1,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D1,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D10,          RSVD1,           NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D11,          RSVD1,           NORMAL,   TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D2,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D3,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D4,           VI,              NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D5,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D6,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D7,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D8,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	DEFAULT_PINMUX(VI_D9,           SDMMC2,          NORMAL,    TRISTATE,     OUTPUT),
	
	VI_PINMUX(VI_HSYNC,        RSVD1,           NORMAL,    TRISTATE,     INPUT,  DISABLE, DISABLE),
	DEFAULT_PINMUX(VI_MCLK,         VI,              NORMAL,   TRISTATE,     INPUT),
	VI_PINMUX(VI_PCLK,         RSVD1,           NORMAL,   TRISTATE,   INPUT,  DISABLE, ENABLE),	
	VI_PINMUX(VI_VSYNC,        RSVD1,           NORMAL,    TRISTATE,     INPUT,  DISABLE, DISABLE),

	//DDR

	//DSI_CSI

	//HDMI

	//HSIC

	//IC_USB

	//PCIe

        //SATA

        //THERM

        //USB

        //VDAC

        //XTAL
        
};

static __initdata struct tegra_pingroup_config m470_unused_pinmux_common[] = {
};

static struct tegra_gpio_table  m470_gpio_table[] = {
};

static __initdata struct pin_info_low_power_mode  m470_unused_gpio_pins[] = {
};

static __initdata tegra_gpio_init_pin_info  m470_gpio_init_table[] = {  
        //PIN_GPIO_INIT("CDC_IRQ_N", TEGRA_GPIO_CDC_IRQ_N, 1, 1),
       	PIN_GPIO_INIT("HP_DET", TEGRA_GPIO_M470_HP_DET, 1, 1),	   
        PIN_GPIO_INIT("KEY_DET", TEGRA_GPIO_M470_KEY_DET, 1, 1),
        PIN_GPIO_INIT("CAM_PWR_EN", TEGRA_GPIO_CAM_PWR_EN, 0, 0),
        PIN_GPIO_INIT("CAM_AVDDPWREN", TEGRA_GPIO_CAM_AVDD_PWR_EN, 0, 0), 
        
        PIN_GPIO_INIT("EN_CODEC_PA", TEGRA_GPIO_EN_CODEC_PA, 0, 0),
        PIN_GPIO_INIT("GYRO_IRQ_N", TEGRA_GPIO_GYRO_IRQ_N, 1, 1), 
        //PIN_GPIO_INIT("EN_VDD_SDMMC1", TEGRA_GPIO_EN_VDD_SDMMC1, 0, 0), //regulator
        PIN_GPIO_INIT("EN_VID_OC_N", TEGRA_GPIO_EN_VDDIO_VID_OC_N, 0, 1), 
        //PIN_GPIO_INIT("GPS_PWN", TEGRA_GPIO_GPS_PWN, 1, 1), 
        PIN_GPIO_INIT("NFC_PWN", TEGRA_GPIO_NFC_PWN, 0, 0), 
        PIN_GPIO_INIT("AP_ONKEY_N", TEGRA_GPIO_AP_ONKEY_N, 1, 1), 
                
        PIN_GPIO_INIT("CAM_AF_PWDN_N", TEGRA_GPIO_CAM_AF_EN, 0, 0), 
        PIN_GPIO_INIT("REAR_CAM_PWDN", TEGRA_GPIO_REAR_CAM_PWDN, 0, 0), 
        PIN_GPIO_INIT("FRONT_CAM_RST", TEGRA_GPIO_FRONT_CAM_RST, 0, 0), 
        PIN_GPIO_INIT("FRONT_CAM_PWDN", TEGRA_GPIO_FRONT_CAM_PWDN, 0, 0), 
        PIN_GPIO_INIT("CAM_RST_N", TEGRA_GPIO_REAR_CAM_RST_N, 0, 0), 
        
        PIN_GPIO_INIT("LCD_BL_EN", TEGRA_GPIO_LCD_BL_EN, 0, 1), 
        PIN_GPIO_INIT("EN_VDD_BL", TEGRA_GPIO_EN_VDD_BL, 0, 1), 
        //PIN_GPIO_INIT("EN_VDD_FUSE", TEGRA_GPIO_EN_VDD_FUSE, 0, 0),  //regulator
        PIN_GPIO_INIT("EN_LCD_1V8", TEGRA_GPIO_EN_LCD_1V8, 0, 1), 
        PIN_GPIO_INIT("TP_LP0", TEGRA_GPIO_TP_LP0, 0, 0), 
        PIN_GPIO_INIT("PMU_CHRG_DET", TEGRA_GPIO_PMU_CHRG_DET, 1, 0), 
        PIN_GPIO_INIT("SDMMC_CD_N", TEGRA_GPIO_SDMMC_CD_N, 1, 1), 
        PIN_GPIO_INIT("HDMI_HPD", TEGRA_GPIO_HDMI_HPD, 1, 1), 
        PIN_GPIO_INIT("COMPASS_RST_N", TEGRA_GPIO_COMPASS_RST_N, 0, 1), 
        PIN_GPIO_INIT("COMPASS_DRDY", TEGRA_GPIO_COMPASS_DRDY, 1, 0), 
        PIN_GPIO_INIT("LVDS_SHTDN_N", TEGRA_GPIO_LVDS_SHTDN_N, 0, 1), 
        PIN_GPIO_INIT("BT_REG_ON", TEGRA_GPIO_BT_REG_ON, 0, 0), 
        PIN_GPIO_INIT("EN_VDD_PNL", TEGRA_GPIO_EN_VDD_PNL, 0, 1), 
        PIN_GPIO_INIT("RST_CDC", TEGRA_GPIO_RST_CDC, 0, 1), 
        PIN_GPIO_INIT("PMU_MSECURE", TEGRA_GPIO_PMU_MSECURE, 0, 0), 
        PIN_GPIO_INIT("EN_LCD_3V3", TEGRA_GPIO_EN_LCD_3V3, 0, 1), 
        PIN_GPIO_INIT("ALS_IRQ_N", TEGRA_GPIO_ALS_IRQ_N, 1, 1), 
        PIN_GPIO_INIT("TS_RESET_N", TEGRA_GPIO_TS_RESET_N, 0, 0), 
        PIN_GPIO_INIT("TS_IRQ_N", TEGRA_GPIO_TS_IRQ_N, 1, 1), 
        PIN_GPIO_INIT("WLAN_RST_N", TEGRA_GPIO_WLAN_RST_N, 0, 0), 
       // PIN_GPIO_INIT("BT_RST_N", TEGRA_GPIO_BT_RST_N, 0, 0), 
        PIN_GPIO_INIT("VOL_UP", TEGRA_GPIO_VOL_UP, 1, 1), 
        PIN_GPIO_INIT("VOL_DOWN", TEGRA_GPIO_VOL_DOWN, 1, 1), 
        //PIN_GPIO_INIT("GPS_RST_N", TEGRA_GPIO_GPS_RST_N, 0, 0), 
        PIN_GPIO_INIT("TEMP_ALERT_N", TEGRA_GPIO_TEMP_ALERT_N, 1, 1), 
        PIN_GPIO_INIT("NFC_WAKE", TEGRA_GPIO_NFC_WAKE, 0, 0), 
        PIN_GPIO_INIT("CAM_FLASH", TEGRA_GPIO_CAM_FLASH_EN, 0, 0), 
        PIN_GPIO_INIT("CAM_TORCH", TEGRA_GPIO_CAM_TORCH_EN, 0, 0), 
        PIN_GPIO_INIT("WLAN_HOST_WAKE", TEGRA_GPIO_WLAN_HOST_WAKE, 1, 1), 
        PIN_GPIO_INIT("BATREMOVAL", TEGRA_GPIO_BATREMOVAL, 1, 1), 
        PIN_GPIO_INIT("NRESWARM", TEGRA_GPIO_NRESWARM, 0, 1), 
        PIN_GPIO_INIT("HOST_BT_WAKE", TEGRA_GPIO_HOST_BT_WAKE, 0, 0), 
        PIN_GPIO_INIT("NFC_IRQ", TEGRA_GPIO_NFC_IRQ, 1, 1), 
        PIN_GPIO_INIT("BT_HOST_WAKE", TEGRA_GPIO_BT_HOST_WAKE, 1, 1), 
};

//gpio_init_pin_info
static void tegra_pinmux_gpio_init(tegra_gpio_init_pin_info *gpio_init_table, int list_count)
{
        int i;
        tegra_gpio_init_pin_info *pin_info;
	int ret;

	for (i = 0; i < list_count; ++i) {
		pin_info = ( tegra_gpio_init_pin_info *)(gpio_init_table + i);
		if (!pin_info->is_gpio)
			continue;

                if(pin_info->is_gpio)
                        tegra_gpio_enable(pin_info->gpio_nr);

		ret = gpio_request(pin_info->gpio_nr, pin_info->name);
		if (ret < 0) {
			pr_err("%s() Error in gpio_request() for gpio %d\n",
					__func__, pin_info->gpio_nr);
			continue;
		}
		if (pin_info->is_input)
			ret = gpio_direction_input(pin_info->gpio_nr);
		else
			ret = gpio_direction_output(pin_info->gpio_nr,
							pin_info->value);
		if (ret < 0) {
			pr_err("%s() Error in setting gpio %d to in/out\n",
				__func__, pin_info->gpio_nr);
			gpio_free(pin_info->gpio_nr);
			continue;
		} 		
	}
}

static void enterprise_set_unused_pin_gpio(struct pin_info_low_power_mode *lpm_pin_info,
		int list_count)
{
	int i;
	struct pin_info_low_power_mode *pin_info;
	int ret;

	for (i = 0; i < list_count; ++i) {
		pin_info = (struct pin_info_low_power_mode *)(lpm_pin_info + i);
		if (!pin_info->is_gpio)
			continue;

		ret = gpio_request(pin_info->gpio_nr, pin_info->name);
		if (ret < 0) {
			pr_err("%s() Error in gpio_request() for gpio %d\n",
					__func__, pin_info->gpio_nr);
			continue;
		}
		if (pin_info->is_input)
			ret = gpio_direction_input(pin_info->gpio_nr);
		else
			ret = gpio_direction_output(pin_info->gpio_nr,
							pin_info->value);
		if (ret < 0) {
			pr_err("%s() Error in setting gpio %d to in/out\n",
				__func__, pin_info->gpio_nr);
			gpio_free(pin_info->gpio_nr);
			continue;
		}
	}
}

int __init enterprise_pinmux_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);

	tegra_pinmux_config_table( m470_pinmux_common, 
                                        ARRAY_SIZE( m470_pinmux_common)); 
        tegra_drive_pinmux_config_table( m470_drive_pinmux,
					ARRAY_SIZE( m470_drive_pinmux));
	tegra_pinmux_config_table( m470_unused_pinmux_common, 
                                        ARRAY_SIZE( m470_unused_pinmux_common)); 				
	tegra_gpio_config( m470_gpio_table, ARRAY_SIZE( m470_gpio_table));	
	enterprise_set_unused_pin_gpio( m470_unused_gpio_pins,
			ARRAY_SIZE( m470_unused_gpio_pins));
	tegra_pinmux_gpio_init( m470_gpio_init_table, ARRAY_SIZE( m470_gpio_init_table));

	return 0;
}

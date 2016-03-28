/*
 * Copyright (c) 2016, 
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef COM32H3N74ULC_LCD_H_
#define COM32H3N74ULC_LCD_H_

#include <spi.h>

int com32h3n74ulc_init(unsigned reset_gpio, unsigned bus, unsigned cs);

#endif

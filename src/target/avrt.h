/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_AVRT_H
#define OPENOCD_TARGET_AVRT_H

#include <jtag/jtag.h>

enum {
	AVR_R0,
	AVR_R1,
	AVR_R2,
	AVR_R3,
	AVR_R4,
	AVR_R5,
	AVR_R6,
	AVR_R7,
	AVR_R8,
	AVR_R9,
	AVR_R10,
	AVR_R11,
	AVR_R12,
	AVR_R13,
	AVR_R14,
	AVR_R15,
	AVR_R16,
	AVR_R17,
	AVR_R18,
	AVR_R19,
	AVR_R20,
	AVR_R21,
	AVR_R22,
	AVR_R23,
	AVR_R24,
	AVR_R25,
	AVR_R26,
	AVR_R27,
	AVR_R28,
	AVR_R29,
	AVR_R30,
	AVR_R31,
	AVR_R32,

	AVR_SREG,
	AVR_SP,
	AVR_PC,

	AVR_REG_W,
	AVR_REG_X,
	AVR_REG_Y,
	AVR_REG_Z,
};

struct mcu_jtag {
	struct jtag_tap *tap;
};

struct avr_common {
	struct mcu_jtag jtag_info;
};

int mcu_execute_queue(void);
int avr_jtag_sendinstr(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out);
int avr_jtag_senddat(struct jtag_tap *tap, uint32_t *dr_in, uint32_t dr_out,
		int len);

#endif /* OPENOCD_TARGET_AVRT_H */

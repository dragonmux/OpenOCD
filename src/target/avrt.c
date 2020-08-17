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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "avrt.h"
#include "target.h"
#include "target_type.h"
#include "register.h"

#define AVR_JTAG_INS_LEN	4

/* forward declarations */
static int avr_target_create(struct target *target, Jim_Interp *interp);
static int avr_init_target(struct command_context *cmd_ctx, struct target *target);

static int avr_arch_state(struct target *target);
static int avr_poll(struct target *target);
static int avr_halt(struct target *target);
static int avr_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution);
static int avr_step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints);

static int avr_assert_reset(struct target *target);
static int avr_deassert_reset(struct target *target);

/* IR and DR functions */
static int mcu_write_ir(struct jtag_tap *tap, uint8_t *ir_in, uint8_t *ir_out, int ir_len, int rti);
static int mcu_write_dr(struct jtag_tap *tap, uint8_t *dr_in, uint8_t *dr_out, int dr_len, int rti);
static int mcu_write_ir_u8(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out, int ir_len, int rti);
static int mcu_write_dr_u32(struct jtag_tap *tap, uint32_t *ir_in, uint32_t ir_out, int dr_len, int rti);

struct target_type avr_target = {
	.name = "avr",

	.poll = avr_poll,
	.arch_state = avr_arch_state,

	.halt = avr_halt,
	.resume = avr_resume,
	.step = avr_step,

	.assert_reset = avr_assert_reset,
	.deassert_reset = avr_deassert_reset,

	.get_gdb_reg_list = avr_get_gdb_reg_list,
/*
	.read_memory = avr_read_memory,
	.write_memory = avr_write_memory,
	.bulk_write_memory = avr_bulk_write_memory,
	.checksum_memory = avr_checksum_memory,
	.blank_check_memory = avr_blank_check_memory,

	.run_algorithm = avr_run_algorithm,

	.add_breakpoint = avr_add_breakpoint,
	.remove_breakpoint = avr_remove_breakpoint,
	.add_watchpoint = avr_add_watchpoint,
	.remove_watchpoint = avr_remove_watchpoint,
*/
	.target_create = avr_target_create,
	.init_target = avr_init_target,
};

static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	enum reg_type type;
	const char *group;
	const char *feature;
} avr_regs[] = {
	{ AVR_R0, "r0", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R1, "r1", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R2, "r2", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R3, "r3", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R4, "r4", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R5, "r5", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R6, "r6", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R7, "r7", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R8, "r8", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R9, "r9", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R10, "r10", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R11, "r11", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R12, "r12", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R13, "r13", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R14, "r14", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R15, "r15", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R16, "r16", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R17, "r17", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R18, "r18", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R19, "r19", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R20, "r20", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R21, "r21", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R22, "r22", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R23, "r23", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R24, "r24", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R25, "r25", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R26, "r26", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R27, "r27", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R28, "r28", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R29, "r29", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R30, "r30", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },
	{ AVR_R31, "r31", 8, REG_TYPE_INT8, "general", "org.gnu.gdb.avr.cpu" },

	{ AVR_SREG, "sreg", 8, REG_TYPE_INT8, "system", "org.gnu.gdb.avr.system" },
	{ AVR_SP, "sp", 16, REG_TYPE_DATA_PTR, "system", "org.gnu.gdb.avr.system" },
	{ AVR_PC, "pc", 32, REG_TYPE_CODE_PTR, "system", "org.gnu.gdb.avr.system" },
	{ AVR_PC, "pc2", 32, REG_TYPE_CODE_PTR, "system", "org.gnu.gdb.avr.system" },
};

static int avr_target_create(struct target *target, Jim_Interp *interp)
{
	struct avr_common *avr = calloc(1, sizeof(struct avr_common));

	avr->jtag_info.tap = target->tap;
	target->arch_info = avr;

	return ERROR_OK;
}

static int avr_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_poll(struct target *target)
{
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
		target->state = TARGET_HALTED;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

int avr_jtag_senddat(struct jtag_tap *tap, uint32_t *dr_in, uint32_t dr_out,
		int len)
{
	return mcu_write_dr_u32(tap, dr_in, dr_out, len, 1);
}

int avr_jtag_sendinstr(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out)
{
	return mcu_write_ir_u8(tap, ir_in, ir_out, AVR_JTAG_INS_LEN, 1);
}

static int avr_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct avr_common *avr = target_to_avr(target);
	int i;

	if (reg_class == REG_CLASS_ALL)
		*reg_list_size = AVR_NUM_ALL_REGS;
	else
		*reg_list_size = AVR_NUM_GP_REGS;
	*reg_list = malloc(sizeof(struct reg *) * *reg_list_size);
	if (*reg_list == NULL)
		return ERROR_FAIL;

	for (i = 0; i < *reg_list_size; i++)
		(*reg_list)[i] = &avr->core_cache->reg_list[i];

	return ERROR_OK;
}

static struct reg_cache *avr_build_reg_cache(struct target *target)
{
	struct avr_common *avr = target_to_avr(target);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(AVR_NUM_ALL_REGS, sizeof(struct reg));
	struct avr_reg *arch_info = calloc(AVR_NUM_ALL_REGS, sizeof(struct avr_reg));
	struct reg_feature *feature;
	struct reg_data_type *data_type;
	int i;

	cache->name = "avr registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = AVR_NUM_ALL_REGS;
	*register_get_last_cache_p(&target->reg_cache) = cache;

	for (i = 0; i < AVR_NUM_ALL_REGS; i++) {
		arch_info[i].num = avr_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].avr = avr;

		reg_list[i].name = avr_regs[i].name;
		reg_list[i].size = avr_regs[i].bits;
		size_t storage_size = DIV_ROUND_UP(avr_regs[i].bits, 8);
		reg_list[i].value = calloc(1, storage_size);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &avr_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = avr_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = avr_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		data_type = calloc(1, sizeof(struct reg_data_type));
		if (data_type) {
			data_type->type = avr_regs[i].type;
			reg_list[i].reg_data_type = data_type;
		} else
			LOG_ERROR("unable to allocate reg type list");
	}

	avr->core_cache = cache;
	return cache;
}

/* IR and DR functions */
static int mcu_write_ir(struct jtag_tap *tap, uint8_t *ir_in, uint8_t *ir_out,
		int ir_len, int rti)
{
	if (!tap) {
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}
	if (ir_len != tap->ir_length) {
		LOG_ERROR("invalid ir_len");
		return ERROR_FAIL;
	}

	{
		jtag_add_plain_ir_scan(tap->ir_length, ir_out, ir_in, TAP_IDLE);
	}

	return ERROR_OK;
}

static int mcu_write_dr(struct jtag_tap *tap, uint8_t *dr_in, uint8_t *dr_out,
		int dr_len, int rti)
{
	if (!tap) {
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}

	{
		jtag_add_plain_dr_scan(dr_len, dr_out, dr_in, TAP_IDLE);
	}

	return ERROR_OK;
}

static int mcu_write_ir_u8(struct jtag_tap *tap, uint8_t *ir_in,
		uint8_t ir_out, int ir_len, int rti)
{
	if (ir_len > 8) {
		LOG_ERROR("ir_len overflow, maximum is 8");
		return ERROR_FAIL;
	}

	mcu_write_ir(tap, ir_in, &ir_out, ir_len, rti);

	return ERROR_OK;
}

static int mcu_write_dr_u32(struct jtag_tap *tap, uint32_t *dr_in,
		uint32_t dr_out, int dr_len, int rti)
{
	if (dr_len > 32) {
		LOG_ERROR("dr_len overflow, maximum is 32");
		return ERROR_FAIL;
	}

	mcu_write_dr(tap, (uint8_t *)dr_in, (uint8_t *)&dr_out, dr_len, rti);

	return ERROR_OK;
}

int mcu_execute_queue(void)
{
	return jtag_execute_queue();
}

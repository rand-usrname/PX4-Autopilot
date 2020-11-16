/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once
#include <stdint.h>

typedef enum  {
	MTD_PARAMETERS  = 1,
	MTD_WAYPOINTS   = 2,
	MTD_CALDATA     = 3,
	MTD_MFT         = 4,
	MTD_ID          = 5,
	MTD_NET         = 6,
} px4_mtd_types_t;


typedef struct  {

	enum px4_bus_type {
		I2C = 0,
		SPI = 1,
	} bus_type;

	uint32_t devid;
} px4_mtd_device_t;

#define PX4_MK_I2C_DEVID(b,a) ((b) << 16 | ((a) & 0xffff))
#define PX4_I2C_DEVID_BUS(d)  (((d) >> 16) & 0xffff)
#define PX4_I2C_DEVID_ADDR(d) ((d) & 0xffff)

typedef struct  {
	const px4_mtd_types_t   type;
	const char              *path;
	const uint32_t          nblocks;
} px4_mtd_part_t;

typedef struct  {
	const px4_mtd_device_t  *device;
	const uint32_t           npart;
	const px4_mtd_part_t     partd[];
} px4_mtd_entry_t;

typedef struct  {
	const uint32_t            nconfigs;
	const px4_mtd_entry_t     *entries[];
} px4_mtd_config_t;


__BEGIN_DECLS
#if defined(BOARD_MANAGED_MTD)
#define CONFIG_AT24XX_MULTI     1
#endif
extern const px4_mtd_config_t board_mtd_config;
/************************************************************************************
 * Name: px4_mtd_config
 *
 * Description:
 *   A board will call this function, to set up the mtd partitions
 *
 * Input Parameters:
 *  list    - A list of GPIO pins to be initialized
 *  count   - Size of the list
 *
 * Returned Value:
 *   Nothing
 *
 ************************************************************************************/

__EXPORT int px4_mtd_config(const px4_mtd_config_t *mtd_list);

__END_DECLS

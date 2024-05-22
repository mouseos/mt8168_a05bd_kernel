/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#ifndef _H_MEMORY_LAYOUT_H_
#define _H_MEMORY_LAYOUT_H_

#define KERN_RAMCONSOLE_BASE		(0x54400000)
#define KERN_RAMCONSOLE_MAX_SIZE	(CONFIG_MTK_RAM_CONSOLE_DRAM_SIZE)
#define KERN_PSTORE_BASE		(CONFIG_PSTORE_MEM_ADDR)
#define KERN_PSTORE_MAX_SIZE		(CONFIG_PSTORE_MEM_SIZE)
#define KERN_MINIDUMP_BASE		(CONFIG_PSTORE_MEM_ADDR + CONFIG_PSTORE_MEM_SIZE)
#define KERN_MINIDUMP_MAX_SIZE		(0x10000)

#endif

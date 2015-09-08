/*
 * Raspberry Pi GPIO - FPGA Interface Driver - Character device registration
 *
 * Authored by Robert Bartle, May 2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
 
#ifndef __AX_CHAR_H
#define __AX_CHAR_H

#define DEV_MAJOR 234
#define DEV_MINOR 1
#define DEV_NAME "ax_fpga"

#include <linux/poll.h>
#include <linux/cdev.h>
 
extern int ax_register_character_device(struct cdev *character_device);

#endif /* __AX_CHAR_H */

/*
 * Raspberry Pi GPIO - FPGA Interface Driver - SPI registration
 *
 * Authored by Robert Bartle, July 2015
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
 
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#ifdef CONFIG_ARCH_BCM2708
#include <mach/platform.h>
#endif
	
#include "ax-spi.h"
	
static struct spi_board_info fpga_spi_board_info[]  = {
	{
		.modalias = "AX-FPGAv1",
		.max_speed_hz = 15625000, //15.625MHz
		.mode = SPI_MODE_0,
		.chip_select = 0,
		.platform_data = NULL,
		.bus_num = 0
	}, {
		.modalias = "AX-FPGAv1",
		.max_speed_hz = 15625000, //15.625MHz
		.mode = SPI_MODE_0,
		.chip_select = 1,
		.platform_data = NULL,
		.bus_num = 0
	}
};
	  
static void spidevices_delete(struct spi_master *master, unsigned cs)
{
	struct device *dev;
	char str[32];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	if (dev)
		device_del(dev);
}

int ax_register_spi_driver(struct spi_driver *driver)
{
	struct spi_master *master;
		
	master = spi_busnum_to_master(0);
	if (master == NULL)
		return -EINVAL;
	
	spidevices_delete(master, 0);
	//spidevices_delete(master, 1);

	//spi_register_board_info cannot be used in modules
	//if (spi_register_board_info(fpga_spi_board_info, ARRAY_SIZE(fpga_spi_board_info)) != 0) {
	if (spi_new_device(master, &fpga_spi_board_info[0]) == NULL) {
		//|| spi_new_device(master, &fpga_spi_board_info[1]) == NULL) {
		return -5; // Input/Output error
	}
	put_device(&master->dev);

	return spi_register_driver(driver);
}
EXPORT_SYMBOL_GPL(ax_register_spi_driver);

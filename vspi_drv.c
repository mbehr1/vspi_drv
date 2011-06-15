/*
 * vspi_drv.c
 *
 *  Created on: Jun 15, 2011
 *      Author: mbehr
 *  (c) M. Behr, 2011
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h> // for e.g. -ENOMEM, ...

// module parameter:
static int param_ber = 0; // module parameter bit error rate: 0 = none
module_param(param_ber, int, S_IRUGO|S_IWUSR); // readable by all users in sysfs (/sys/module/vspi_drv/parameters/), changeable only by root
static unsigned long param_speed_cps = 18000000/8; // module parameter speed in cps.
module_param(param_speed_cps, ulong, S_IRUGO); // only readable


// init function on module load:
static int __init vspi_drv_init(void)
{
	printk( KERN_ALERT "vspi_drv_init (c) M. Behr, 2011\n");
	return 0; // 0 = success
}

// exit function on module unload:
static void __exit vspi_drv_exit(void)
{
	printk( KERN_ALERT "vspi_drv_exit\n");
}

module_init( vspi_drv_init );
module_exit( vspi_drv_exit );

// todo p3 define license MODULE_LICENSE("proprietary");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Matthias Behr");
MODULE_DESCRIPTION("Virtual SPI driver with unrealiability features (patents pending)\n");

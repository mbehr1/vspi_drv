/*
 * vspi_drv.c
 *
 *  Created on: Jun 15, 2011
 *      Author: mbehr
 *  (c) M. Behr, 2011
 *
 *  Based on source code and documentation from the book
 *  "Linux Device Drivers" by Alessandro Rubini and Jonathan Corbet,
 *  published by O'Reilly & Associates.
 *  Thanks a lot to Rubini, Corbet and O'Reilly!
 *
 * todo add GPLv2 lic here?
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h> // kmalloc
#include <linux/fs.h>
#include <linux/errno.h> // for e.g. -ENOMEM, ...
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include "vspi_drv.h"

// module parameter:
int param_major = VSPI_MAJOR;
module_param(param_major, int, S_IRUGO);
int param_minor = 0;
module_param(param_minor, int, S_IRUGO);
int param_ber = 0; // module parameter bit error rate: 0 = none
module_param(param_ber, int, S_IRUGO|S_IWUSR); // readable by all users in sysfs (/sys/module/vspi_drv/parameters/), changeable only by root
static unsigned long param_speed_cps = 18000000/8; // module parameter speed in cps.
module_param(param_speed_cps, ulong, S_IRUGO); // only readable

struct vspi_dev *vspi_devices; // allocated in vspi_drv_init

struct file_operations vspi_fops = {
		.owner = THIS_MODULE,
		.read = vspi_read,
		.write = vspi_write,
		.unlocked_ioctl = vspi_ioctl,
		.open = vspi_open,
		.release = vspi_release
};
// exit function on module unload:
// used from _init as well in case of errors!

static void vspi_exit(void)
{
	int i;
	dev_t devno = MKDEV(param_major, param_minor);
	printk( KERN_NOTICE "vspi_exit\n");

	// get rid of our devices:
	if (vspi_devices){
		for(i=0; i<VSPI_NR_DEVS; i++){
			cdev_del(&vspi_devices[i].cdev);
		}
		kfree(vspi_devices);
		vspi_devices=0;
	}

	unregister_chrdev_region( devno, VSPI_NR_DEVS);
}

// setup the char_dev structure for these devices:
static void vspi_setup_cdev(struct vspi_dev *dev, int index)
{
	int err, devno = MKDEV(param_major, param_minor + index);
	cdev_init(&dev->cdev, &vspi_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &vspi_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding vspi_drv%d", err, index);
}

// init function on module load:
static int __init vspi_init(void)
{
	int retval = 0, i;
	dev_t dev=0;

	printk( KERN_ALERT "vspi_drv_init (c) M. Behr, 2011\n");

	if (param_major){
		dev = MKDEV(param_major, param_minor);
		retval = register_chrdev_region(dev, VSPI_NR_DEVS, "vspi_drv");
	}else{
		retval = alloc_chrdev_region(&dev, param_minor, VSPI_NR_DEVS, "vspi_drv");
		param_major = MAJOR(dev);
	}
	if (retval <0 ){
		printk(KERN_WARNING "vspi_drv: can't get major %d\n", param_major);
		return retval;
	}

	// allocate the devices. We could have them static as well, as we don't change the number at load time
	vspi_devices = kmalloc( VSPI_NR_DEVS * sizeof( struct vspi_dev), GFP_KERNEL);
	if (!vspi_devices){
		retval = -ENOMEM;
		goto fail;
	}
	memset(vspi_devices, 0, VSPI_NR_DEVS * sizeof(struct vspi_dev));

	// init each device:
	for (i=0; i< VSPI_NR_DEVS; i++){
		vspi_devices[i].isMaster = ((i==0) ? 1 : 0); // first one is master
		sema_init(&vspi_devices[i].sem, 1); // 1 = count
		vspi_setup_cdev(&vspi_devices[i], i);
	}

	return 0; // 0 = success,
fail:
	vspi_exit();
	return retval;
}


module_init( vspi_init );
module_exit( vspi_exit );

// open:
int vspi_open(struct inode *inode, struct file *filep)
{
	struct vspi_dev *dev;
	dev = container_of(inode->i_cdev, struct vspi_dev, cdev);
	filep->private_data = dev;

	printk( KERN_ALERT "vspi open\n");

	return 0; // success
}

// close/release:
int vspi_release(struct inode *inode, struct file *filep)
{
	// nothing to cleanup/release right now

	return 0; // success
}
// read:
ssize_t vspi_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t retval = -ENOMEM;
	struct vspi_dev *dev = filep->private_data;


	return retval;
}

// write:
ssize_t vspi_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t retval = -ENOMEM;

	return retval;
}

// the ioctl implementation. This is our main function. read/write will just use this.
// this is used as .unlocked_ioctl!
long vspi_ioctl(struct file *filep,
		unsigned int cmd, unsigned long arg)
{
	long retval = -EFAULT;


	return retval;
}




// todo p3 define license MODULE_LICENSE("proprietary");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Matthias Behr");
MODULE_DESCRIPTION("Virtual SPI driver with unrealiability features (patents pending)\n");

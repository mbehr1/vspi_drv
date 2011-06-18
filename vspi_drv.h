/*
 * vspi_drv.h
 *
 *  Created on: Jun 18, 2011
 *      Author: mbehr
 */

#ifndef VSPI_DRV_H_
#define VSPI_DRV_H_

#ifndef VSPI_MAJOR
#define VSPI_MAJOR 0 // dynamic major by default
#endif

#ifndef VSPI_NR_DEVS
#define VSPI_NR_DEVS 2 // just two devices. One for master one for slave
#endif

struct vspi_dev {
	int isMaster; // indicated whether this device the SPI master or slave
	struct semaphore sem;
	struct cdev cdev;
};

// the configurable parameters
extern int param_major; // major number for char devices
extern int param_ber; // Bit error rate

// prototypes for shared functions:
int vspi_open(struct inode *inode, struct file *filep);
int vspi_release(struct inode *inode, struct file *filep);
ssize_t vspi_read(struct file *filep, char __user *buf, size_t count,
		loff_t *f_pos);
ssize_t vspi_write(struct file * filep, const char __user *buf,
		size_t count, loff_t *f_pos);
long vspi_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);

#endif /* VSPI_DRV_H_ */

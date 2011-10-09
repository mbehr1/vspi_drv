/*
 * vspi_drv.h
 *
 *  Created on: Jun 18, 2011
 *      Author: mbehr
 */

#include <linux/spi/spi.h>

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
	int isOpened; // holds the number of opened clients (restricted to 1)
	char *rp, *wp; // read and write pointer within each ioctl.
	s64 xfer_start_ns; // time in ns of transfer start
	s64 xfer_stop_ns;
	unsigned xfer_len; // in bytes.
	unsigned xfer_actual; // how many bytes have been xfered
	int cs_latched_high; // for slave: CS line went high (just the latch not the CS status)
	struct semaphore sem;
	struct cdev cdev;

	// spi parameters:
	struct spi_master *master;
	struct spi_device *device;
	u32			max_speed_cps; // we store in cps not hz. convert to *8 at rd/wr
	u8			chip_select;
	u8			mode;
	u8			bits_per_word;

};

// the configurable parameters
extern int param_major; // major number for char devices
extern unsigned long param_ber; // Bit error rate

// prototypes for shared functions:
int vspi_open(struct inode *inode, struct file *filep);
int vspi_release(struct inode *inode, struct file *filep);
ssize_t vspi_read(struct file *filep, char __user *buf, size_t count,
		loff_t *f_pos);
ssize_t vspi_write(struct file * filep, const char __user *buf,
		size_t count, loff_t *f_pos);
long vspi_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);

#endif /* VSPI_DRV_H_ */

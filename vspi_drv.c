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
#include <linux/spi/spidev.h> // for spi specific ioctl
#include <asm/uaccess.h> // for access_ok
#include <linux/sched.h> // for TASK_INTERRUPTIBLE
#include <linux/wait.h> // wait queues
#include <linux/time.h>
#include <linux/random.h>
#include "vspi_drv.h"

/*
 * to-do list: (global, for features, enhancements,...
 * todo p2: implement param for idle value on mosi line (for master reading without slave sending)
 * todo p3: check maximum values for .delay_us (16bit) vs. our default value and int overflow handling
 * todo p2: add statistic infos (e.g. bytes transferred, timeouts, bit errors,...)
 */

// module parameter:
int param_major = VSPI_MAJOR;
module_param(param_major, int, S_IRUGO);

int param_minor = 0;
module_param(param_minor, int, S_IRUGO);

unsigned long param_ber = 0; // module parameter bit error rate: 0 = none,
module_param(param_ber, ulong, S_IRUGO|S_IWUSR); // readable by all users in sysfs (/sys/module/vspi_drv/parameters/), changeable only by root
MODULE_PARM_DESC(param_ber, "bit error rate for both devices in read direction (amount of corrupt bytes in 2^32)");

static unsigned long param_speed_cps = 18000000/8; // module parameter speed in cps.
module_param(param_speed_cps, ulong, S_IRUGO); // only readable
MODULE_PARM_DESC(param_speed_cps, "speed in bytes per second");

static unsigned long param_min_speed_cps = 100000/8; // min 100kHz

static unsigned long param_max_bytes_per_ioreq = 16*1024; // todo use page_size constant
module_param(param_max_bytes_per_ioreq, ulong, S_IRUGO);
MODULE_PARM_DESC(param_max_bytes_per_ioreq, "data bytes in biggest supported SPI message");

static unsigned long param_slave_default_delay_us = USEC_PER_SEC; // 1 sec is easier for testing from console
module_param(param_slave_default_delay_us, ulong, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(param_slave_default_delay_us, "default slave delay/timeout in us.");


struct vspi_dev *vspi_devices; // allocated in vspi_drv_init

DEFINE_SEMAPHORE(sem_interchange);
//DECLARE_WAITQUEUE_HEAD(event_master);
wait_queue_head_t event_master;

struct file_operations vspi_fops = {
		.owner = THIS_MODULE,
		.read = vspi_read,
		.write = vspi_write,
		.unlocked_ioctl = vspi_ioctl,
		.open = vspi_open,
		.release = vspi_release
};

#define SPI_MODE_MASK        (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
		| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
		| SPI_NO_CS | SPI_READY)

// exit function on module unload:
// used from _init as well in case of errors!

static void vspi_exit(void)
{
	int i;
	dev_t devno = MKDEV(param_major, param_minor);
	printk( KERN_ALERT "vspi_exit\n");

	// get rid of our devices:
	if (vspi_devices){
		for(i=0; i<VSPI_NR_DEVS; i++){
			cdev_del(&vspi_devices[i].cdev);
			// we don't rely on release being called. So check to free buffers here again:
			if (vspi_devices[i].rp)
				kfree(vspi_devices[i].rp);
			if (vspi_devices[i].wp)
				kfree(vspi_devices[i].wp);
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

	printk( KERN_ALERT "vspi_drv_init (HZ=%d) (c) M. Behr, 2011\n", HZ);

	init_waitqueue_head(&event_master);

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
		vspi_devices[i].max_speed_cps = param_speed_cps;
		// rp and wp will be allocated in open
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

	if (dev->isMaster)
		printk( KERN_NOTICE "vspi open master\n");
	else
		printk( KERN_NOTICE "vspi open slave\n");

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	// allow max 1 user at the same time
	if (dev->isOpened){
		up(&dev->sem);
		return -EUSERS;
	}else
		dev->isOpened+=1;

	// now we're holding/blocking the semaphore.
	if (!dev->rp){
		dev->rp = kmalloc(param_max_bytes_per_ioreq, GFP_KERNEL);
		if (!dev->rp){
			up(&dev->sem);
			return -ENOMEM;
		}
	}
	if (!dev->wp){
		dev->wp = kmalloc(param_max_bytes_per_ioreq, GFP_KERNEL);
		if (!dev->wp){
			up(&dev->sem);
			return -ENOMEM;
		}
	}

	up(&dev->sem);
	// now the semaphore is free again.

	return 0; // success
}

// close/release:
int vspi_release(struct inode *inode, struct file *filep)
{
	struct vspi_dev *dev = filep->private_data;

	if(dev->isMaster)
		printk( KERN_ALERT "vspi_release master\n");
	else
		printk( KERN_ALERT "vspi_release slave\n");

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	if (!dev->isOpened){
		printk( KERN_WARNING "vspi_release called with no one opened\n");
	} else{
		dev->isOpened-=1;
	}
	if (!dev->isOpened){
		// now free the buffers:
		if (dev->wp){
			kfree(dev->wp);
			dev->wp=0;
		}
		if (dev->rp){
			kfree(dev->rp);
			dev->rp = 0;
		}
	}

	up(&dev->sem);

	return 0; // success
}

/*
 * calctimeforxfer_us
 * return the time in ns (10⁻9s) it takes to do the transfer.
 */

static unsigned long calctimeforxfer_ns(unsigned cnt, u32 speed)
{
	return cnt*(NSEC_PER_SEC/speed);
}

/*
 * calc_bytes_transfered
 * input:
 *  time in ns
 *  speed in CPS (=bytes per sec)
 * return the number of bytes transfered within the given timeframe and speed
 */

static unsigned calc_bytes_transfered(s64 time, u32 speed)
{
	u64 tmp;
	if (time<=0)
		return 0;
	tmp=(u64)time;
	tmp*=speed;
	do_div(tmp, NSEC_PER_SEC);
	return tmp;
}

static int vspi_handletransfers(struct vspi_dev *dev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	int retval=-EFAULT;
	unsigned n, total=0;
	struct spi_ioc_transfer *u_tmp;
	unsigned long delay_len;
	struct timespec ts;
	struct vspi_dev *slave;

	slave = &vspi_devices[1];
	// assert !isMaster

	for(n=0, u_tmp=u_xfers; n<n_xfers; n++, u_tmp++){
		unsigned copied_to_slave=0;
		// for each transfer:
		// check max len:
		if (u_tmp->len > param_max_bytes_per_ioreq)
			goto done;
		// check access rights
		if (u_tmp->rx_buf){
			if (!access_ok(VERIFY_WRITE, (u8 __user*)
					(uintptr_t) u_tmp->rx_buf, u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf){
			// copy tx_buf to device wp buf:
			// todo p1: need a semapore here! (but not the interchange one)
			// we can copy to wp as outside this function no transfer is active
			// and the other side checks xfer_len which is secured below.

			if (copy_from_user(dev->wp, (const u8 __user *)
					(uintptr_t) u_tmp->tx_buf, u_tmp->len))
				goto done;
		}
		// ok, now we have the tx-data in the dev->wp.

		// block the other side:
		if (down_interruptible(&sem_interchange))
			return -ERESTARTSYS;

		dev->xfer_len = u_tmp->len;
		dev->xfer_actual = 0;
		// init read to zero
		// todo: use param whether not active is zero or 0xff
		if (dev->rp && dev->xfer_len){
			memset( dev->rp, 0, dev->xfer_len);
		}
		// set start time of xfer:
		ts = CURRENT_TIME;
		dev->xfer_start_ns = timespec_to_ns(&ts);
		// calc finish time of xfer:
		if (dev->isMaster){
			delay_len = calctimeforxfer_ns(dev->xfer_len, dev->max_speed_cps);
			dev->xfer_stop_ns = dev->xfer_start_ns + delay_len;

			if (delay_len<=1000){
				/* lets wait with sem being kept. The time is too short anyhow. */
				ndelay(delay_len);
			}else{
				s64 toWait;
				up(&sem_interchange); // give sem before waiting
				// now wait to simulate blocking io.

				// we need a interruptible wait here otherwise
				// no oneelse (slave) can do anything ... ndelay/udelay are non interruptible!
				// but schedule_timer is only on timer tick granularity???

				ts = CURRENT_TIME;
				toWait = dev->xfer_stop_ns - timespec_to_ns(&ts);

				if (toWait > NSEC_PER_MSEC){
					do_div(toWait, NSEC_PER_MSEC);
					msleep_interruptible(toWait);
					/* printk(KERN_NOTICE "vspi_drv: msleep for %lld ms\n", toWait); */
				}else{
					/* less than a ms but more than a us to wait.
					 * let the schedule see at least whether any other task is available:
					 * the remaining time will be waited below
					*/
					schedule();
				}
				// wait for the remaining time in ns granularity:
				ts = CURRENT_TIME;
				toWait = dev->xfer_stop_ns - timespec_to_ns(&ts);
				if (toWait>0)
					ndelay(toWait);
				if (down_interruptible(&sem_interchange))
					return -ERESTARTSYS;

			}

			/* if there is a slave active, copy data to/from his buffers according
			 * to the proper start times.
			 */

			if (slave->xfer_len){
				unsigned client_start_missed, client_end_missed;
				unsigned needed = slave->xfer_len - slave->xfer_actual;
				/* printk(KERN_NOTICE "vspi master with slave waiting for %d/%d :-)\n",
						needed,
						client->xfer_len);*/
				// copy data:
				// check start times and calc offsets:
				/*
				 * compare slave [xfer_start xfer_stop] with master [xfer_start xfer_stop]
				 * interval:
				 * two checks: 1. check whether slave started later
				 * 2. check whether slave will end earlier due to timeout
				 */
				if ((slave->xfer_start_ns > dev->xfer_stop_ns)||
						(slave->xfer_stop_ns < dev->xfer_start_ns)){
					// missed the transfer completely so do nothing (not even waking up)
					printk(KERN_NOTICE "vspi slave completely missed master\n");
				}else{
					if (slave->xfer_start_ns > dev->xfer_start_ns){
						// client missed the start. Let's calc how many bytes:
						client_start_missed = calc_bytes_transfered(slave->xfer_start_ns - dev->xfer_start_ns,
								dev->max_speed_cps);
						printk(KERN_NOTICE "vspi slave missed %d bytes transfered\n", client_start_missed);
					}else{
						client_start_missed=0;
					}

					if (slave->xfer_stop_ns < dev->xfer_stop_ns){
						// slave will finish earlier due timeout
						client_end_missed = calc_bytes_transfered(dev->xfer_stop_ns - slave->xfer_stop_ns,
								dev->max_speed_cps);
						printk(KERN_NOTICE "vspi slave will finish in between mstr xfer (missing %d)\n",
								client_end_missed);
					}else
						client_end_missed = 0;

					if (client_start_missed > min(needed, dev->xfer_len))
						client_start_missed = min(needed, dev->xfer_len);
					needed -= client_start_missed;

					if ((client_start_missed + needed + client_end_missed) > dev->xfer_len){
						needed= dev->xfer_len - (client_start_missed + client_end_missed);
						printk(KERN_NOTICE "vspi reduced needed to %d\n", needed);
					}

					copied_to_slave = min(needed, dev->xfer_len-client_start_missed);
					if (slave->rp && dev->wp){
						memcpy( slave->rp+slave->xfer_actual, dev->wp+client_start_missed,
							copied_to_slave);
						/* printk(KERN_NOTICE "vspi copied %d bytes to slave read buffer\n",
								min(needed, dev->xfer_len-client_start_missed)); */
					}
					if (slave->wp && dev->rp){
						memcpy( dev->rp+client_start_missed, slave->wp+slave->xfer_actual,
								copied_to_slave);
						/* printk(KERN_NOTICE "vspi copied %d bytes to master read buffer\n",
								min(needed, dev->xfer_len-client_start_missed)); */

					}
					slave->xfer_actual += copied_to_slave;
					// the missed bytes are not put in the buffer. They are lost. Slave will wait for more or timeout or CS change
				}
				wake_up_interruptible(&event_master);
				/*
				 * delay_usecs handling here for master:
				 * simply delay ;-)
				 */
				if (u_tmp->delay_usecs){
					printk(KERN_NOTICE "vspi master udelay %dus\n", u_tmp->delay_usecs);
					udelay( u_tmp->delay_usecs);
				}
				/*
				 * cs handling here:
				 * for master:
				 * if cs_change is set: set cs to high here (and back to low at start of loop)
				 * then wake up slave again
				 */
				if (u_tmp->cs_change){
					slave->cs_latched_high++;
					wake_up_interruptible(&event_master);
				}

			}

			if (copied_to_slave != dev->xfer_len)
				printk(KERN_NOTICE "vspi_xfer master %d/%d bytes took %lu ns from %lld to %lld \n",
					copied_to_slave,
					dev->xfer_len,
					delay_len,
					dev->xfer_start_ns,
					dev->xfer_stop_ns);

		}else{
			// for a slave we send the endtime to starttime + max_span:
			delay_len =((u_tmp->delay_usecs ?
					u_tmp->delay_usecs : param_slave_default_delay_us)*NSEC_PER_USEC);

			// as we wait with wait_event the delay_len has to be at least 1 timer tick:
			if (delay_len < (NSEC_PER_SEC / HZ))
				delay_len = NSEC_PER_SEC/HZ;

			dev->xfer_stop_ns = dev->xfer_start_ns + delay_len;

			/*
			 * cs handling for a slave:
			 * if cs_change set, wake up on cs going high (not at being high)
			 *
			 */
			if (u_tmp->cs_change)
				dev->cs_latched_high = 0; // reset flag here

			up(&sem_interchange);
			// now wait for timeout or master signaling us
			wait_event_interruptible_timeout(event_master,
					(dev->xfer_actual>=dev->xfer_len) ||(u_tmp->cs_change && dev->cs_latched_high),
					((delay_len/NSEC_PER_USEC)*HZ / USEC_PER_SEC)+1 ); // waiting a bit longer is better than shorter!
			if (down_interruptible(&sem_interchange))
				return -ERESTARTSYS;

			ts = CURRENT_TIME;
			delay_len = timespec_to_ns(&ts) - dev->xfer_start_ns;

			if (dev->xfer_actual != dev->xfer_len)
				printk(KERN_NOTICE "vspi_xfer slave %d/%d (%d) bytes took %lu ns from %lld to %lld\n",
					dev->xfer_actual, dev->xfer_len, dev->cs_latched_high,
					delay_len,
					dev->xfer_start_ns,
					dev->xfer_stop_ns);
		}

		// copy any data to rx_buf?
		if (dev->isMaster){
			// master always succeeds in tx and/or rx:
			dev->xfer_actual = dev->xfer_len;
		}else{
		}
		total += dev->xfer_actual;

		if (dev->rp && dev->xfer_actual && u_tmp->rx_buf){
			/* add bit error rate before copying to user space as a simulation
			 * of line/reception noise.
			 * ber is expressed as number of corrupt bytes in 2^32
			 * we do the following approach: (targetting low bit error rates)
			 * for each transfer we change at max. 1 byte by random
			 * (could be improved by e.g. each 256 bytes)
			 */
			if (param_ber>0){
				unsigned long random_number[2]; unsigned long ber_cor;
				ber_cor = param_ber * dev->xfer_actual;
				get_random_bytes(random_number, sizeof(random_number));
				if (random_number[0] <= ber_cor){
					u16 *pos;
					u8 *val;
					pos = (u16*)&random_number[1];
					*pos %= dev->xfer_actual;
					val = (u8*)(pos+1);
					printk(KERN_NOTICE "vspi ber changed %d/%d to %d\n", *pos, dev->xfer_actual, *val);
					// ok, let's change random byte to random value:
					dev->rp[*pos] = *val;
				}
			}

			if (__copy_to_user((u8 __user*)(uintptr_t)u_tmp->rx_buf,
					dev->rp, dev->xfer_actual)){
				retval = -EFAULT;
				up(&sem_interchange);
				goto done;
			}
			/* printk(KERN_NOTICE "vspi copied %d to userspace rx_buf\n", dev->xfer_actual); */
		}
		dev->xfer_len = 0; // no more transfer active
		dev->xfer_actual = 0;

		// allow the other thread to access:
		up(&sem_interchange);
	}
	/*
	 * cs handling part 2:
	 * for master set cs to high here (if not already set)
	 * this is needed as single transactions do this only when cs_change is set to 0
	 */
	if (dev->isMaster){
		// we do this without the semaphore being held, as the event is used for signalling as well and
		// it's just the master who increases the value (and the remaining race cond can happen with real hw as well
		if (!slave->cs_latched_high){
			slave->cs_latched_high++;
			wake_up_interruptible(&event_master);
		}
	}

	retval = total;
done:
	return retval; // return nr of bytes transfered
}

// read:
ssize_t vspi_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spi_ioc_transfer xfer;
	struct vspi_dev *dev = filep->private_data;

	if (count > param_max_bytes_per_ioreq)
		return -EMSGSIZE;

	// put request in a spi_ioc_transfer struct:
	memset( &xfer, 0, sizeof(struct spi_ioc_transfer));
	xfer.rx_buf = (uintptr_t)buf;
	xfer.len = count;
	// keep rest at zero. can see later whether more infos are needed at _handletransfers
	return vspi_handletransfers(dev, &xfer, 1);
}

// write:
ssize_t vspi_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct spi_ioc_transfer xfer;
	struct vspi_dev *dev = filep->private_data;

	if (count > param_max_bytes_per_ioreq){
		printk(" vspi_write tried to write %d bytes\n", count);
		return -EMSGSIZE;
	}
	// put request in a spi_ioc_transfer struct:

	memset( &xfer, 0, sizeof(struct spi_ioc_transfer));
	xfer.tx_buf = (uintptr_t) buf;
	xfer.len = count;
	// keep rest at zero for now.
	return vspi_handletransfers(dev, &xfer, 1);
}




// the ioctl implementation. This is our main function. read/write will just use this.
// this is used as .unlocked_ioctl!
long vspi_ioctl(struct file *filep,
		unsigned int cmd, unsigned long arg)
{
	int err=0;
	long retval = 0;
	u32 tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer *ioc;

	struct vspi_dev *dev = filep->private_data;

	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* check access direction one here:
	 * IOC_DIR is from the user perspective, while access_ok is from the kernel
	 * perspective
	 */

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	// todo guard against device removal while we do ioctl?
	// do we have to use block the semaphore here?? (will take a long time...)

	switch (cmd){
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(dev->mode & SPI_MODE_MASK,
				(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((dev->mode & SPI_LSB_FIRST) ? 1 : 0,
				(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(dev->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(dev->max_speed_cps*8, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0){
			if (tmp & ~SPI_MODE_MASK){
				retval = -EINVAL;
				break;
			}
			tmp |= dev->mode & ~SPI_MODE_MASK;
			dev->mode = (u8)tmp;
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval){
			if (tmp)
				dev->mode |= SPI_LSB_FIRST;
			else
				dev->mode &= ~SPI_LSB_FIRST;
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval){
			dev->bits_per_word = tmp;
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (0 == retval){
			if ((tmp/8) >= param_min_speed_cps ){
				if ((tmp/8) <= param_speed_cps)
					dev->max_speed_cps = (tmp/8);
				else
					dev->max_speed_cps = param_speed_cps;
			}
			printk(KERN_NOTICE "vspi_drv set speed to %dcps (wanted %dHz)\n",
					dev->max_speed_cps, tmp);
		}
		break;
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE){
			// todo bug: we should allow _IOC_READ without _IOC_WRITE
			// as well. But currently spidev behaves the same way!
			retval = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0){
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (0 == n_ioc)
			break;

		/* copy requests: */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc){
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)){
			kfree(ioc);
			retval = -EFAULT;
			break;
		}
		retval = vspi_handletransfers(dev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	return retval;
}




// todo p3 define license MODULE_LICENSE("proprietary");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Matthias Behr");
MODULE_DESCRIPTION("Virtual SPI driver with unreliability features (patents pending)\n");

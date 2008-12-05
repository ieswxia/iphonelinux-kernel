#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/wait.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/loop.h>
#include <linux/compat.h>
#include <linux/suspend.h>
#include <linux/freezer.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>		/* for invalidate_bdev() */
#include <linux/completion.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/splice.h>

#include <asm/uaccess.h>

static struct iphone_nand_device {
	spinlock_t lock;
	struct gendisk* gd;
	struct block_device* bdev;
	struct request_queue* queue;
	int sectorSize;
} Device;

static int iphone_nand_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg)
{
	return -ENOTTY;
}

static struct block_device_operations iphone_nand_fops = {
	.owner =		THIS_MODULE,
	.locked_ioctl =		iphone_nand_ioctl,
};

static void iphone_nand_read(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long sectorCount, char* buffer) {
	memset(buffer, 0, sectorCount * dev->sectorSize);
}

static void iphone_nand_write(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long sectorCount, char* buffer) {

}

static int iphone_nand_make_request(struct request_queue *q, struct bio *bio)
{
	struct request *req;

	while ((req = elv_next_request(q)) != NULL) {
		if (!blk_fs_request(req)) {
			end_request(req, 0);
			continue;
		}
		if(rq_data_dir(req))
		{
			// Write
			iphone_nand_write(&Device, req->sector, req->current_nr_sectors, req->buffer);
		} else {
			// Read
			iphone_nand_read(&Device, req->sector, req->current_nr_sectors, req->buffer);
		}
		end_request(req, 1);
	}

	return 0;
}

static int __init iphone_nand_init(void)
{
	int major_num = 0;
	spin_lock_init(&Device.lock);
	major_num = register_blkdev(major_num, "nand");

	Device.sectorSize = 512;

	if(major_num <= 0)
	{
		printk("iphone_nand: unable to get major number\n");
		goto out;
	}

	Device.gd = alloc_disk(1);
	if(!Device.gd)
		goto out_unregister;

	Device.gd->major = major_num;
	Device.gd->first_minor = 0;
	Device.gd->fops = &iphone_nand_fops;
	Device.gd->private_data = &Device;
	strcpy(Device.gd->disk_name, "nand0");
	Device.queue = blk_alloc_queue(GFP_KERNEL);
	if (!Device.queue)
		goto out_free_queue;

	blk_queue_make_request(Device.queue, iphone_nand_make_request);
	blk_queue_max_sectors(Device.queue, 1024);
	blk_queue_bounce_limit(Device.queue, BLK_BOUNCE_ANY);
	blk_queue_hardsect_size(Device.queue, Device.sectorSize);

	Device.gd->queue = Device.queue;

	set_capacity(Device.gd, 2048);
	add_disk(Device.gd);

	return 0;

out_free_queue:
	blk_cleanup_queue(Device.queue);
out_unregister:
	put_disk(Device.gd);
out:
	return -ENOMEM;
}

static void __exit iphone_nand_exit(void)
{
	blk_cleanup_queue(Device.queue);
	put_disk(Device.gd);
}

module_init(iphone_nand_init);
module_exit(iphone_nand_exit);

MODULE_LICENSE("GPL");

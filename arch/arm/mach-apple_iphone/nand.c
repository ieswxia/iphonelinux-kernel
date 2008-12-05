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

static void iphone_nand_read(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long len, char* buffer) {
	printk("iphone_nand_read: sector %ld, len %ld to %p\n", sectorNum, len, buffer);
	memset(buffer, 0, len);
}

static void iphone_nand_write(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long len, char* buffer) {

}

static int iphone_nand_do_bvec(struct iphone_nand_device* dev, struct page *page,
			unsigned int len, unsigned int off, int rw,
			sector_t sector)
{
	void *mem;
	int err = 0;

	mem = kmap_atomic(page, KM_USER0);
	if (rw == READ) {
		iphone_nand_read(dev, sector, len, mem + off);
		flush_dcache_page(page);
	} else
		iphone_nand_write(dev, sector, len, mem + off);
	kunmap_atomic(mem, KM_USER0);

	return err;
}
static int iphone_nand_make_request(struct request_queue *q, struct bio *bio)
{
	struct block_device* bdev = bio->bi_bdev;
	struct iphone_nand_device* dev = bdev->bd_disk->private_data;
	int rw;
	struct bio_vec *bvec;
	sector_t sector;
	int i;
	int err = -EIO;

	sector = bio->bi_sector;
	if (sector + (bio->bi_size / dev->sectorSize) >
						get_capacity(bdev->bd_disk))
		goto out;

	rw = bio_rw(bio);
	if (rw == READA)
		rw = READ;

	bio_for_each_segment(bvec, bio, i) {
		unsigned int len = bvec->bv_len;
		err = iphone_nand_do_bvec(dev, bvec->bv_page, len,
					bvec->bv_offset, rw, sector);
		if (err)
			break;
		sector += len / dev->sectorSize;
	}

out:
	bio_endio(bio, err);

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

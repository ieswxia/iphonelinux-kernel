/*
 *  arch/arm/mach-apple_iphone/lcd.c
 *
 *  Copyright (C) 2008 Yiduo Wang
 *
 *  Adapted from linux/drivers/video/skeletonfb.c
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

#define DEFAULT_WINDOW_NUM 2
#define LCD IO_ADDRESS(0x38900000)

static void iphone_set_fb_address(int window, void* address) {
	u32 windowBase;
	switch(window) {
		case 1:
			windowBase = LCD + 0x58;
			break;
		case 2:
			windowBase = LCD + 0x70;
			break;
		case 3:
			windowBase = LCD + 0x88;
			break;
		case 4:
			windowBase = LCD + 0xA0;
			break;
		case 5:
			windowBase = LCD + 0xB8;
			break;
		default:
			return;
	}

	__raw_writel((u32)address, windowBase + 8);
}
    /*
     *  This is just simple sample code.
     *
     *  No warranty that it actually compiles.
     *  Even less warranty that it actually works :-)
     */

/*
 * Driver data
 */
static void* framebuffer_virtual_memory __devinitdata;

static struct fb_var_screeninfo iphonefb_var __devinitdata = {
	.xres = 320,
	.yres = 480,
	.xres_virtual = 320,
	.yres_virtual = 480,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 16,
	.grayscale = 0,
	.red = {
		.offset = 11,
		.length = 5,
		.msb_right = 0
	},
	.blue = {
		.offset = 0,
		.length = 5,
		.msb_right = 0
	},
	.green = {
		.offset = 5,
		.length = 6,
		.msb_right = 0
	},
	.width = 320,
	.height = 480,
	.activate = FB_ACTIVATE_NOW
};

/*
 *  If your driver supports multiple boards, you should make the  
 *  below data types arrays, or allocate them dynamically (using kmalloc()). 
 */ 

/* 
 * This structure defines the hardware state of the graphics card. Normally
 * you place this in a header file in linux/include/video. This file usually
 * also includes register information. That allows other driver subsystems
 * and userland applications the ability to use the same header file to 
 * avoid duplicate work and easy porting of software. 
 */
struct iphonefb_par {
	u32 palette[16];
};

/*
 * Here we define the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see iphonefb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well. 
 */
static struct fb_fix_screeninfo iphonefb_fix __devinitdata = {
	.id =		"iphonefb", 
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0, 
	.line_length =	320 * 2, 
	.accel =	FB_ACCEL_NONE,
};

    /*
     * 	Modern graphical hardware not only supports pipelines but some 
     *  also support multiple monitors where each display can have its  
     *  its own unique data. In this case each display could be  
     *  represented by a separate framebuffer device thus a separate 
     *  struct fb_info. Now the struct iphonefb_par represents the graphics
     *  hardware state thus only one exist per card. In this case the 
     *  struct iphonefb_par for each graphics card would be shared between 
     *  every struct fb_info that represents a framebuffer on that card. 
     *  This allows when one display changes it video resolution (info->var) 
     *  the other displays know instantly. Each display can always be
     *  aware of the entire hardware state that affects it because they share
     *  the same iphonefb_par struct. The other side of the coin is multiple
     *  graphics cards that pass data around until it is finally displayed
     *  on one monitor. Such examples are the voodoo 1 cards and high end
     *  NUMA graphics servers. For this case we have a bunch of pars, each
     *  one that represents a graphics state, that belong to one struct 
     *  fb_info. Their you would want to have *par point to a array of device
     *  states and have each struct fb_ops function deal with all those 
     *  states. I hope this covers every possible hardware design. If not
     *  feel free to send your ideas at jsimmons@users.sf.net 
     */

    /*
     *  If your driver supports multiple boards or it supports multiple 
     *  framebuffers, you should make these arrays, or allocate them 
     *  dynamically using framebuffer_alloc() and free them with
     *  framebuffer_release().
     */ 

    /* 
     * Each one represents the state of the hardware. Most hardware have
     * just one hardware state. These here represent the default state(s). 
     */

int iphonefb_init(void);

/**
 *	iphonefb_open - Optional function. Called when the framebuffer is
 *		     first accessed.
 *	@info: frame buffer structure that represents a single frame buffer
 *	@user: tell us if the userland (value=1) or the console is accessing
 *	       the framebuffer. 
 *
 *	This function is the first function called in the framebuffer api.
 *	Usually you don't need to provide this function. The case where it 
 *	is used is to change from a text mode hardware state to a graphics
 * 	mode state. 
 *
 *	Returns negative errno on error, or zero on success.
 */
static int iphonefb_open(struct fb_info *info, int user)
{
    return 0;
}

/**
 *	iphonefb_release - Optional function. Called when the framebuffer 
 *			device is closed. 
 *	@info: frame buffer structure that represents a single frame buffer
 *	@user: tell us if the userland (value=1) or the console is accessing
 *	       the framebuffer. 
 *	
 *	Thus function is called when we close /dev/fb or the framebuffer 
 *	console system is released. Usually you don't need this function.
 *	The case where it is usually used is to go from a graphics state
 *	to a text mode state.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int iphonefb_release(struct fb_info *info, int user)
{
    return 0;
}

static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

static int iphonefb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info) {
	if (regno < 16) {
		u32* pal = (u32*) info->pseudo_palette;
		pal[regno] = convert_bitfield(blue, &info->var.blue) |
			convert_bitfield(green, &info->var.green) |
			convert_bitfield(red, &info->var.red);
		return 0;
	}
	else {
		return 1;
	}

}


    /*
     *  Frame buffer operations
     */

static struct fb_ops iphonefb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= iphonefb_open,
	.fb_read	= fb_sys_read,
	.fb_write	= fb_sys_write,
	.fb_release	= iphonefb_release,
	.fb_setcolreg   = iphonefb_setcolreg,
	.fb_fillrect	= cfb_fillrect, 	/* Needed !!! */
	.fb_copyarea	= cfb_copyarea,	/* Needed !!! */
	.fb_imageblit	= cfb_imageblit,	/* Needed !!! */
};

/* ------------------------------------------------------------------------- */

    /*
     *  Initialization
     */

/* static int __init xxfb_probe (struct platform_device *pdev) -- for platform devs */
static int __init iphonefb_probe(struct platform_device *pdev)
{
    struct fb_info *info;
    struct iphonefb_par *par;
    struct device *device = &pdev->dev; /* or &pdev->dev */

    /*
     * Dynamically allocate info and par
     */
    info = framebuffer_alloc(sizeof(struct iphonefb_par), device);
    framebuffer_virtual_memory = kmalloc(iphonefb_var.xres * iphonefb_var.yres * 2, GFP_KERNEL);
    iphone_set_fb_address(DEFAULT_WINDOW_NUM, framebuffer_virtual_memory);

    if (!info) {
	    /* goto error path */
    }

    par = info->par;

    /* 
     * Here we set the screen_base to the virtual memory address
     * for the framebuffer. Usually we obtain the resource address
     * from the bus layer and then translate it to virtual memory
     * space via ioremap. Consult ioport.h. 
     */
    info->screen_base = framebuffer_virtual_memory;
    info->fbops = &iphonefb_ops;
    info->fix = iphonefb_fix; /* this will be the only time iphonefb_fix will be
			    * used, so mark it as __devinitdata
			    */
    /*
     * Set up flags to indicate what sort of acceleration your
     * driver can provide (pan/wrap/copyarea/etc.) and whether it
     * is a module -- see FBINFO_* in include/linux/fb.h
     *
     * If your hardware can support any of the hardware accelerated functions
     * fbcon performance will improve if info->flags is set properly.
     *
     * FBINFO_HWACCEL_COPYAREA - hardware moves
     * FBINFO_HWACCEL_FILLRECT - hardware fills
     * FBINFO_HWACCEL_IMAGEBLIT - hardware mono->color expansion
     * FBINFO_HWACCEL_YPAN - hardware can pan display in y-axis
     * FBINFO_HWACCEL_YWRAP - hardware can wrap display in y-axis
     * FBINFO_HWACCEL_DISABLED - supports hardware accels, but disabled
     * FBINFO_READS_FAST - if set, prefer moves over mono->color expansion
     * FBINFO_MISC_TILEBLITTING - hardware can do tile blits
     *
     * NOTE: These are for fbcon use only.
     */
    info->flags = FBINFO_DEFAULT;

    /* This has to been done !!! */	
    fb_alloc_cmap(&info->cmap, 256, 0);
    info->pseudo_palette = ((struct iphonefb_par*) info->par)->palette;
    /* 
     * The following is done in the case of having hardware with a static 
     * mode. If we are setting the mode ourselves we don't call this. 
     */	
    info->var = iphonefb_var;

    /*
     * Does a call to fb_set_par() before register_framebuffer needed?  This
     * will depend on you and the hardware.  If you are sure that your driver
     * is the only device in the system, a call to fb_set_par() is safe.
     *
     * Hardware in x86 systems has a VGA core.  Calling set_par() at this
     * point will corrupt the VGA console, so it might be safer to skip a
     * call to set_par here and just allow fbcon to do it for you.
     */
    /* iphonefb_set_par(info); */

    if (register_framebuffer(info) < 0)
	return -EINVAL;
    printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	   info->fix.id);
    platform_set_drvdata(pdev, info);
    return 0;
}

    /*
     *  Cleanup
     */
static int __init iphonefb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	if (info) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		/* ... */
		kfree(framebuffer_virtual_memory);
		framebuffer_release(info);
	}
	return 0;
}

/* for platform devices */

#define iphonefb_suspend NULL
#define iphonefb_resume NULL

static struct platform_driver iphonefb_driver = {
	.probe = iphonefb_probe,
	.remove = iphonefb_remove,
	.suspend = iphonefb_suspend, /* optional but recommended */
	.resume = iphonefb_resume,   /* optional but recommended */
	.driver = {
		.owner = THIS_MODULE,
		.name = "iphonefb",
	},
};

static struct platform_device *iphonefb_device;

    /*
     *  Setup
     */

int __init iphonefb_init(void)
{
	int ret;
	/*
	 *  For kernel boot options (in 'video=iphonefb:<options>' format)
	 */
	char *option = NULL;

	if (fb_get_options("iphonefb", &option))
		return -ENODEV;

	ret = platform_driver_register(&iphonefb_driver);

	if (!ret) {
		iphonefb_device = platform_device_register_simple("iphonefb", 0,
								NULL, 0);

		if (IS_ERR(iphonefb_device)) {
			platform_driver_unregister(&iphonefb_driver);
			ret = PTR_ERR(iphonefb_device);
		}
	}

	return ret;
}

static void __exit iphonefb_exit(void)
{
	platform_device_unregister(iphonefb_device);
	platform_driver_unregister(&iphonefb_driver);
}

module_init(iphonefb_init);
module_exit(iphonefb_exit);

MODULE_LICENSE("GPL")

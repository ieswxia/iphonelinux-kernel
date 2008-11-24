/*
 * linux/drivers/video/skeletonfb.c -- Skeleton for a frame buffer device
 *
 *  Modified to new api Jan 2001 by James Simmons (jsimmons@transvirtual.com)
 *
 *  Created 28 Dec 1997 by Geert Uytterhoeven
 *
 *
 *  I have started rewriting this driver as a example of the upcoming new API
 *  The primary goal is to remove the console code from fbdev and place it
 *  into fbcon.c. This reduces the code and makes writing a new fbdev driver
 *  easy since the author doesn't need to worry about console internals. It
 *  also allows the ability to run fbdev without a console/tty system on top 
 *  of it. 
 *
 *  First the roles of struct fb_info and struct display have changed. Struct
 *  display will go away. The way the new framebuffer console code will
 *  work is that it will act to translate data about the tty/console in 
 *  struct vc_data to data in a device independent way in struct fb_info. Then
 *  various functions in struct fb_ops will be called to store the device 
 *  dependent state in the par field in struct fb_info and to change the 
 *  hardware to that state. This allows a very clean separation of the fbdev
 *  layer from the console layer. It also allows one to use fbdev on its own
 *  which is a bounus for embedded devices. The reason this approach works is  
 *  for each framebuffer device when used as a tty/console device is allocated
 *  a set of virtual terminals to it. Only one virtual terminal can be active 
 *  per framebuffer device. We already have all the data we need in struct 
 *  vc_data so why store a bunch of colormaps and other fbdev specific data
 *  per virtual terminal. 
 *
 *  As you can see doing this makes the con parameter pretty much useless
 *  for struct fb_ops functions, as it should be. Also having struct  
 *  fb_var_screeninfo and other data in fb_info pretty much eliminates the 
 *  need for get_fix and get_var. Once all drivers use the fix, var, and cmap
 *  fbcon can be written around these fields. This will also eliminate the
 *  need to regenerate struct fb_var_screeninfo, struct fb_fix_screeninfo
 *  struct fb_cmap every time get_var, get_fix, get_cmap functions are called
 *  as many drivers do now. 
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
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
	.bits_per_pixel = 32,
	.grayscale = 0,
	.red = {
		.offset = 0,
		.length = 8,
		.msb_right = 0
	},
	.blue = {
		.offset = 8,
		.length = 8,
		.msb_right = 0
	},
	.green = {
		.offset = 16,
		.length = 8,
		.msb_right = 0
	},
	.transp = {
		.offset = 0,
		.length = 0,
		.msb_right = 0
	},
	.width = 50,
	.height = 76,
	.vmode = FB_VMODE_NONINTERLACED
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
struct xxx_par {
	int x;
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
	.accel =	FB_ACCEL_NONE,
};

    /*
     * 	Modern graphical hardware not only supports pipelines but some 
     *  also support multiple monitors where each display can have its  
     *  its own unique data. In this case each display could be  
     *  represented by a separate framebuffer device thus a separate 
     *  struct fb_info. Now the struct xxx_par represents the graphics
     *  hardware state thus only one exist per card. In this case the 
     *  struct xxx_par for each graphics card would be shared between 
     *  every struct fb_info that represents a framebuffer on that card. 
     *  This allows when one display changes it video resolution (info->var) 
     *  the other displays know instantly. Each display can always be
     *  aware of the entire hardware state that affects it because they share
     *  the same xxx_par struct. The other side of the coin is multiple
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

    /*
     *  Frame buffer operations
     */

static struct fb_ops iphonefb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= iphonefb_open,
	.fb_read	= fb_sys_read,
	.fb_write	= fb_sys_write,
	.fb_release	= iphonefb_release,
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
    struct xxx_par *par;
    struct device *device = &pdev->dev; /* or &pdev->dev */
   
    /*
     * Dynamically allocate info and par
     */
    info = framebuffer_alloc(sizeof(struct xxx_par), device);
    framebuffer_virtual_memory = framebuffer_alloc(iphonefb_var.xres * iphonefb_var.yres * 4, device);
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
static int iphonefb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	if (info) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		/* ... */
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


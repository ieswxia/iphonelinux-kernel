#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "iphone-audio.h"

static int iphone_soc_to_wm8758_init(struct snd_soc_codec *codec)
{
	pr_debug("ENTER iphone_soc_to_wm8758_init\n");
	return 0;
}

static struct snd_soc_dai_link iphone_i2s_soc_to_wm8758 = {
	.name           = "WM8758",
	.stream_name    = "WM8758",
	.cpu_dai        = &iphone_i2s_dai,
	.codec_dai      = &iphone_wm8758_dai,
	.init           = iphone_soc_to_wm8758_init,
};

static struct snd_soc_card iphone_snd_soc_card = {
	.name           = "iPhoneSound",
	.platform       = &snd_iphone_soc_platform,
	.dai_link       = &iphone_i2s_soc_to_wm8758,
	.num_links      = 1,
};

static struct snd_soc_device iphone_soc_device = {
	.card           = &iphone_snd_soc_card,
	.codec_dev      = &soc_codec_dev_iphone,
};

static struct platform_device *snd_dev;

static int __init iphone_sound_init(void)
{
	int ret = 0;

	snd_dev = platform_device_alloc("soc-audio", -1);
	if (!snd_dev) {
		printk("failed to alloc soc-audio devicec\n");
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(snd_dev, &iphone_soc_device);
	iphone_soc_device.dev = &snd_dev->dev;

	ret = platform_device_add(snd_dev);
	if (ret) {
		printk("failed to add soc-audio dev\n");
		return ret;
	}

	return ret;
}

static void __exit iphone_sound_exit(void)
{
	platform_device_unregister(snd_dev);
}

module_init(iphone_sound_init);
module_exit(iphone_sound_exit);

MODULE_DESCRIPTION("iPhone SoC sound driver");
MODULE_AUTHOR("Yiduo Wang");
MODULE_LICENSE("GPL");

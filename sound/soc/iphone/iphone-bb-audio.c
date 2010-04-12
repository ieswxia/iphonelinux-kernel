#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

static struct iphone_bb_priv {
	unsigned int sysclk;
	struct snd_soc_codec codec;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	u16 reg_cache[10];
} priv;

static int iphone_bb_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_info(codec->dev, "ENTER iphone_bb_pcm_startup\n");
	return 0;
}


static int iphone_bb_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_info(codec->dev, "ENTER iphone_bb_pcm_hw_params\n");
	return 0;
}

static int iphone_bb_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	dev_info(codec->dev, "ENTER iphone_bb_set_dai_fmt %u\n", fmt);
	return 0;
}

static int iphone_bb_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	dev_info(codec->dev, "ENTER iphone_bb_set_dai_sysclk %d %u %d\n", clk_id, freq, dir);
	return 0;
}

static int iphone_bb_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_info(codec->dev, "ENTER iphone_bb_mute\n");
	return 0;
}

static struct snd_soc_dai_ops iphone_bb_dai_ops = {
	.startup = iphone_bb_pcm_startup,
	.hw_params = iphone_bb_pcm_hw_params,
	.set_fmt = iphone_bb_set_dai_fmt,
	.set_sysclk = iphone_bb_set_dai_sysclk,
	.digital_mute = iphone_bb_mute,
};

struct snd_soc_dai iphone_bb_dai = {
	.name = "Baseband",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	 },
	.ops = &iphone_bb_dai_ops,
	.symmetric_rates = 1,
};

static int iphone_bb_audio_probe(struct platform_device *pdev)
{
	int ret;

	struct snd_soc_codec *codec = &priv.codec;

	pr_debug("ENTER iphone_bb_audio_probe\n");

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->dev = &pdev->dev;
	codec->private_data = &priv;
	codec->name = "Baseband";
	codec->owner = THIS_MODULE;
	codec->dai = &iphone_bb_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(priv.reg_cache);
	codec->reg_cache = &priv.reg_cache;

	iphone_bb_dai.dev = codec->dev;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dai(&iphone_bb_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		goto err_codec;
	}

	dev_info(codec->dev, "DAI and codec registered\n");

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	return ret;
}

static int iphone_bb_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = &priv.codec;

	snd_soc_unregister_dai(&iphone_bb_dai);
	snd_soc_unregister_codec(codec);

	return 0;
}

static int soc_codec_dev_iphone_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_codec *codec;
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	pr_debug("ENTER soc_codec_dev_iphone_probe\n");
	socdev->card->codec = &priv.codec;
	codec = &priv.codec;

	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_debug(KERN_ERR "iphone-bb-audio: failed to create pcms\n");
		goto pcm_err;
	}

	return 0;

pcm_err:
	return ret;
}

static int soc_codec_dev_iphone_remove(struct platform_device *pdev)
{
	pr_debug("ENTER soc_codec_dev_iphone_remove\n");
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_iphone = {
	.probe          = soc_codec_dev_iphone_probe,
	.remove         = soc_codec_dev_iphone_remove,
};

static struct platform_driver iphone_bb_audio_driver = {
	.probe = iphone_bb_audio_probe,
	.remove = iphone_bb_audio_remove,
	.suspend = NULL, /* optional but recommended */
	.resume = NULL,   /* optional but recommended */
	.driver = {
		.owner = THIS_MODULE,
		.name = "iphone-bb-audio",
	},
};

static struct platform_device iphone_bb_audio_dev = {
	.name = "iphone-bb-audio",
	.id = -1,
};

static int __init iphone_bb_audio_init(void)
{
	int ret;

	ret = platform_driver_register(&iphone_bb_audio_driver);

	if (!ret) {
		ret = platform_device_register(&iphone_bb_audio_dev);

		if (ret != 0) {
			platform_driver_unregister(&iphone_bb_audio_driver);
		}
	}
	return ret;
//	return 0;
}

static void __exit iphone_bb_audio_exit(void)
{
	platform_device_unregister(&iphone_bb_audio_dev);
	platform_driver_unregister(&iphone_bb_audio_driver);
}

module_init(iphone_bb_audio_init);
module_exit(iphone_bb_audio_exit);

MODULE_DESCRIPTION("iPhone baseband audio codec driver");
MODULE_AUTHOR("Yiduo Wang");
MODULE_LICENSE("GPL");

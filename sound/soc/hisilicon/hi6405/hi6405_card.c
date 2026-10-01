// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei L410 sound card: ASP DMA PCM <-SLIMbus-> Hi6405 codec, and the
 * codec's I2S4 output feeding two TAS2562 speaker amplifiers.
 *
 * Replaces the vendor da_combine_machine.c ("codec_without_hifi" card).
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "hi6405_drv.h"

/* TAS2562 book 0 / page 0 registers */
#define TAS2562_PB_CFG1		0x03
#define TAS2562_TDM_CFG2	0x08
#define TAS2562_RX_SCFG_MASK	0x30
#define TAS2562_RX_SCFG_LEFT	0x10
#define TAS2562_RX_SCFG_RIGHT	0x20

struct hi6405_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link links[2];
	struct snd_soc_dai_link_component pcm_cpu;
	struct snd_soc_dai_link_component pcm_codec;
	struct snd_soc_dai_link_component pcm_platform;
	struct snd_soc_dai_link_component spk_cpu;
	struct snd_soc_dai_link_component spk_codecs[2];
	struct snd_soc_component *amps[2];
	struct dentry *debugfs;
};

/* I2S4 of the codec: 48 kHz stereo, one PA per slot */
static const struct snd_soc_pcm_stream hi6405_spk_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_dapm_widget hi6405_card_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	/*
	 * The playback SLIMbus track is started by the codec's AUDIO_PLAY_DRV
	 * widget. Keep that widget powered whenever a playback stream runs, even
	 * with no output selected (headphone switch on, nothing plugged in):
	 * otherwise the ASP DMA gets no requests and the stream stalls.
	 */
	SND_SOC_DAPM_SINK("ASP Playback Sink"),
};

static const struct snd_soc_dapm_route hi6405_card_routes[] = {
	{ "Headphone", NULL, "HP_L_OUTPUT" },
	{ "Headphone", NULL, "HP_R_OUTPUT" },
	{ "HSMIC_INPUT", NULL, "Headset Mic" },
	{ "MIC_INPUT", NULL, "Internal Mic" },
	{ "ASP Playback Sink", NULL, "AUDIO_PLAY_DRV" },
	{ "Speaker", NULL, "Left OUT" },
	{ "Speaker", NULL, "Right OUT" },
};

static const struct snd_kcontrol_new hi6405_card_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
};

/*
 * Register values the vendor user space (hwaudioservice) wrote to both
 * amplifiers before first use, restricted to book 0 pages 0-4 (regmap range
 * of the upstream driver): boost current limit and class-H settings.
 */
static const struct {
	unsigned int reg, val;
} tas2562_vendor_init[] = {
	{ 0x38, 0x0c },
	{ 0x40, 0x21 },
	{ 0x3b, 0x38 },
	{ 0x3c, 0x3c },
	{ 0x3e, 0x30 },
};

/*
 * Amplifier status straight from the chips (cache bypassed): power mode,
 * live and latched interrupt flags (read by l410-mainline tests/audio.sh).
 */
static int hi6405_amps_show(struct seq_file *m, void *unused)
{
	static const unsigned int regs[] = { 0x02, 0x03, 0x08, 0x1f, 0x20, 0x24, 0x25 };
	struct hi6405_card *priv = m->private;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(priv->amps); i++) {
		struct regmap *map;

		if (!priv->amps[i])
			continue;
		map = dev_get_regmap(priv->amps[i]->dev, NULL);
		if (!map)
			continue;
		seq_printf(m, "%s:", dev_name(priv->amps[i]->dev));
		regcache_cache_bypass(map, true);
		for (j = 0; j < ARRAY_SIZE(regs); j++) {
			unsigned int val = 0;
			int ret = regmap_read(map, regs[j], &val);

			if (ret)
				seq_printf(m, " %02x=err%d", regs[j], ret);
			else
				seq_printf(m, " %02x=%02x", regs[j], val);
		}
		regcache_cache_bypass(map, false);
		seq_putc(m, '\n');
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hi6405_amps);

static int hi6405_spk_init(struct snd_soc_pcm_runtime *rtd)
{
	struct hi6405_card *priv = container_of(rtd->card, struct hi6405_card, card);
	struct snd_soc_dai *dai;
	int i, j, ret;

	for_each_rtd_codec_dais(rtd, i, dai) {
		struct snd_soc_component *amp = dai->component;

		if (i < ARRAY_SIZE(priv->amps))
			priv->amps[i] = amp;

		/* 32-bit I2S slots, PA 0 takes the left slot, PA 1 the right */
		ret = snd_soc_dai_set_tdm_slot(dai, 0x3, 0x3, 2, 32);
		if (ret)
			return ret;
		snd_soc_component_update_bits(amp, TAS2562_TDM_CFG2,
			TAS2562_RX_SCFG_MASK,
			i ? TAS2562_RX_SCFG_RIGHT : TAS2562_RX_SCFG_LEFT);
		/* amplifier level 14 (same as the vendor "AMP_LEVEL 0x1c") */
		snd_soc_component_update_bits(amp, TAS2562_PB_CFG1, 0x3e, 0x1c);
		for (j = 0; j < ARRAY_SIZE(tas2562_vendor_init); j++)
			snd_soc_component_write(amp, tas2562_vendor_init[j].reg,
				tas2562_vendor_init[j].val);
	}
	return 0;
}

static int hi6405_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct snd_soc_dai_link *link;
	struct hi6405_card *priv;
	int ret;

	if (!slimbus_is_ready())
		return dev_err_probe(dev, -EPROBE_DEFER, "waiting for SLIMbus\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pcm_cpu.of_node = of_parse_phandle(np, "hisilicon,asp-pcm", 0);
	priv->pcm_codec.of_node = of_parse_phandle(np, "hisilicon,codec", 0);
	if (!priv->pcm_cpu.of_node || !priv->pcm_codec.of_node)
		return dev_err_probe(dev, -EINVAL, "missing asp-pcm/codec phandles\n");
	priv->pcm_cpu.dai_name = "slimbus-dai";
	priv->pcm_codec.dai_name = "DA_combine_v5-audio-dai";
	priv->pcm_platform.of_node = priv->pcm_cpu.of_node;

	link = &priv->links[0];
	link->name = "Hi6405";
	link->stream_name = "Hi6405 Audio";
	link->cpus = &priv->pcm_cpu;
	link->num_cpus = 1;
	link->codecs = &priv->pcm_codec;
	link->num_codecs = 1;
	link->platforms = &priv->pcm_platform;
	link->num_platforms = 1;
	/* the ASP PCM does its format conversion and DMA setup in process context */
	link->nonatomic = 1;

	priv->card.num_links = 1;

	priv->spk_codecs[0].of_node = of_parse_phandle(np, "hisilicon,speaker-amps", 0);
	priv->spk_codecs[1].of_node = of_parse_phandle(np, "hisilicon,speaker-amps", 1);
	if (priv->spk_codecs[0].of_node && priv->spk_codecs[1].of_node) {
		priv->spk_cpu.of_node = priv->pcm_codec.of_node;
		priv->spk_cpu.dai_name = "DA_combine_v5-s4-dai";
		priv->spk_codecs[0].dai_name = "tas2562-amplifier";
		priv->spk_codecs[1].dai_name = "tas2562-amplifier";

		link = &priv->links[1];
		link->name = "Hi6405 Speaker";
		link->stream_name = "Hi6405 I2S4";
		link->cpus = &priv->spk_cpu;
		link->num_cpus = 1;
		link->codecs = priv->spk_codecs;
		link->num_codecs = 2;
		link->c2c_params = &hi6405_spk_params;
		link->num_c2c_params = 1;
		/* the codec's I2S4 is the bit/frame clock provider */
		link->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBC_CFC;
		link->init = hi6405_spk_init;
		link->ignore_pmdown_time = 1;
		priv->card.num_links = 2;
	} else {
		dev_warn(dev, "no speaker amplifiers described, speaker disabled\n");
	}

	priv->card.dev = dev;
	priv->card.owner = THIS_MODULE;
	priv->card.name = "hi6405";
	priv->card.driver_name = "hi6405";
	priv->card.dai_link = priv->links;
	priv->card.dapm_widgets = hi6405_card_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(hi6405_card_widgets);
	priv->card.dapm_routes = hi6405_card_routes;
	/* the speaker routes need the TAS2562 widgets */
	priv->card.num_dapm_routes = ARRAY_SIZE(hi6405_card_routes) -
		(priv->card.num_links == 2 ? 0 : 2);
	priv->card.controls = hi6405_card_controls;
	priv->card.num_controls = ARRAY_SIZE(hi6405_card_controls);
	of_property_read_string(np, "model", &priv->card.long_name);

	ret = devm_snd_soc_register_card(dev, &priv->card);
	if (ret)
		return dev_err_probe(dev, ret, "card registration failed\n");

	priv->debugfs = debugfs_create_dir("hi6405-card", NULL);
	debugfs_create_file("amps", 0400, priv->debugfs, priv, &hi6405_amps_fops);
	return 0;
}

static void hi6405_card_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct hi6405_card *priv = container_of(card, struct hi6405_card, card);

	debugfs_remove_recursive(priv->debugfs);
}

static const struct of_device_id hi6405_card_match[] = {
	{ .compatible = "hisilicon,hi3xxx-hi6405", },
	{ }
};
MODULE_DEVICE_TABLE(of, hi6405_card_match);

struct platform_driver hi6405_card_driver = {
	.driver = {
		.name = "hi6405-card",
		.of_match_table = hi6405_card_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = hi6405_card_probe,
	.remove = hi6405_card_remove,
};

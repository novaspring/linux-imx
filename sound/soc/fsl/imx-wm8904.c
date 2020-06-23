/*
 * Copyright (C) 2015-2017 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>
#include "../codecs/wm8904.h"
#include "fsl_sai.h"

#define DAI_NAME_SIZE 32

struct imx_wm8904_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
	struct regmap *gpr;
};

static int imx_wm8904_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_wm8904_data *data = container_of(rtd->card,
						    struct imx_wm8904_data,
						    card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, WM8904_CLK_MCLK,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget imx_wm8904_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_SPK("Line Out Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static int imx_wm8904_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct device_node *gpr_np;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct imx_wm8904_data *data;
	struct snd_soc_dai_link_component *dlc;
	int ret;

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = devm_clk_get(&codec_dev->dev, "mclk");
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	} else {
		data->clk_frequency = clk_get_rate(data->codec_clk);
		clk_prepare_enable(data->codec_clk);
	}

	gpr_np = of_parse_phandle(pdev->dev.of_node, "gpr", 0);
	if (gpr_np) {
		data->gpr = syscon_node_to_regmap(gpr_np);
		if (IS_ERR(data->gpr)) {
			ret = PTR_ERR(data->gpr);
			dev_err(&pdev->dev, "failed to get gpr regmap\n");
			goto fail;
		}
	}

	data->dai.cpus = &dlc[0];
	data->dai.num_cpus = 1;
	data->dai.platforms = &dlc[1];
	data->dai.num_platforms = 1;
	data->dai.codecs = &dlc[2];
	data->dai.num_codecs = 1;

	data->dai.name = "WM8904";
	data->dai.stream_name = "WM8904 PCM";
	data->dai.codecs->dai_name = "wm8904-hifi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platforms->of_node = cpu_np;
	data->dai.init = &imx_wm8904_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
	    SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
	data->card.num_links = 1;
	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	data->card.dapm_widgets = imx_wm8904_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8904_dapm_widgets);

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;

	data->card.dai_link = &data->dai;

	platform_set_drvdata(pdev, &data->card);

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;

	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	return 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8904_dt_ids[] = {
	{.compatible = "fsl,imx-audio-wm8904",},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx_wm8904_dt_ids);

static struct platform_driver imx_wm8904_driver = {
	.driver = {
		   .name = "imx-wm8904",
		   .pm = &snd_soc_pm_ops,
		   .of_match_table = imx_wm8904_dt_ids,
		   },
	.probe = imx_wm8904_probe,
};

module_platform_driver(imx_wm8904_driver);

MODULE_AUTHOR("Yadviga Grigoryeva");
MODULE_DESCRIPTION("Freescale i.MX WM8904 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8904");

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define SN_DEVICE_REV_REG			0x08
#define SN_SOFT_RESET_REG			0x09

#define SN_LVDSPLL_SRC_REG			0x0A
#define  HS_CLK_SRC				BIT(0)
#define  LVDS_CLK_RANGE_MASK			GENMASK(3, 1)
#define  LVDS_CLK_RANGE(x)			((x) << 1)
#define  PLL_EN_STAT				BIT(7)

#define SN_CLK_REG				0x0B
#define  DSI_CLK_DIV_MASK			GENMASK(7, 3)
#define  DSI_CLK_DIV(x)				((x) << 3)
#define  REFCLK_MULT_MASK			GENMASK(1, 0)

#define SN_PLL_ENABLE_REG			0x0D
#define  PLL_EN					BIT(0)

#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define  SOT_ERR_TOL_DIS			BIT(0)

#define SN_DSIA_EQ_REG				0x11
#define  CHA_DSI_DATA_EQ_MASK			GENMASK(7, 6)
#define  CHA_DSI_CLK_EQ_MASK			GENMASK(3, 2)

#define SN_DSIA_CLK_RANGE_REG			0x12

#define SN_LVDS0_REG				0x18
#define  DE_NEG_POL				BIT(7)
#define  HS_NEG_POL				BIT(6)
#define  VS_NEG_POL				BIT(5)
#define  LVDS_LINK_CFG				BIT(4)
#define  CHA_24BPP_MODE				BIT(3)
#define  CHB_24BPP_MODE				BIT(2)
#define  CHA_24BPP_FORMAT1			BIT(1)
#define  CHB_24BPP_FORMAT1			BIT(0)

#define SN_LVDS1_REG				0x19
#define  CHA_LVDS_VOCM				BIT(6)
#define  CHB_LVDS_VOCM				BIT(4)
#define  CHA_LVDS_VOD_SWING_MASK		GENMASK(3, 2)
#define  CHA_LVDS_VOD_SWING(x)			((x) << 2)
#define  CHB_LVDS_VOD_SWING_MASK		GENMASK(1, 0)
#define  CHB_LVDS_VOD_SWING(x)			((x) << 0)

#define SN_LVDS2_REG				0x1A
#define  EVEN_ODD_SWAP				BIT(6)
#define  CHA_REVERSE_LVDS			BIT(5)
#define  CHB_REVERSE_LVDS			BIT(4)
#define  CHA_LVDS_TERM				BIT(1)
#define  CHB_LVDS_TERM				BIT(0)

#define SN_LVDS3_REG				0x1B
#define  CHA_LVDS_CM_ADJUST_MASK		GENMASK(5, 4)
#define  CHA_LVDS_CM_ADJUST(x)			((x) << 4)
#define  CHB_LVDS_CM_ADJUST_MASK		GENMASK(1, 0)

#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_ACTIVE_LINE_LENGTH_HIGH_REG	0x21
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_VERTICAL_DISPLAY_SIZE_HIGH_REG	0x25
#define SN_CHA_SYNC_DELAY_LOW_REG		0x28
#define SN_CHA_SYNC_DELAY_HIGH_REG		0x29
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define SN_CHA_TEST_PATTERN_REG			0x3C

#define SN_IRQ_EN_REG				0xE0
#define SN_IRQ_MASK_REG				0xE1
#define  CHA_SYNCH_ERR_EN			BIT(7)
#define  CHA_CRC_ERR_EN				BIT(6)
#define  CHA_UNC_ECC_ERR_EN			BIT(5)
#define  CHA_COR_ECC_ERR_EN			BIT(4)
#define  CHA_LLP_ERR_EN				BIT(3)
#define  CHA_SOT_BIT_ERR_EN			BIT(2)
#define  PLL_UNLOCK_EN				BIT(0)
#define SN_ERR_REG				0xE5
#define  CHA_SYNCH_ERR				BIT(7)
#define  CHA_CRC_ERR				BIT(6)
#define  CHA_UNC_ECC_ERR			BIT(5)
#define  CHA_COR_ECC_ERR			BIT(4)
#define  CHA_LLP_ERR				BIT(3)
#define  CHA_SOT_BIT_ERR			BIT(2)
#define  PLL_UNLOCK				BIT(0)

#define MIN_DSI_CLK_FREQ_MHZ	40
#define MAX_DSI_CLK_FREQ_MHZ	500
#define MAX_DSI_CLK_FREQ_KHZ	500000

#define SN_REGULATOR_SUPPLY_NUM	4

struct ti_sn_bridge_config {
	u32 sn_dsi_clk_divider;
	u32 sn_dsi_lane_count;
	u8 lvds_de_neg_polarity;
	u32 lvds_channel_count;
	u8 lvds_set_24bpp_mode;
	u8 lvds_set_24bpp_format1;
	u8 lvds_cha_vocm;
	u8 lvds_chb_vocm;
	u32 lvds_cha_vod_swing;
	u32 lvds_chb_vod_swing;
	u8 lvds_even_odd_swap;
	u8 lvds_cha_reverse;
	u8 lvds_chb_reverse;
	u8 lvds_cha_term;
	u8 lvds_chb_term;
	u32 lvds_cha_cm_adjust;
	u32 lvds_chb_cm_adjust;
	u32 sn_sync_delay;
	u8 dsi_burst_mode;
};

struct ti_sn_bridge {
	struct device			*dev;
	struct regmap			*regmap;
	struct drm_bridge		bridge;
	struct drm_connector		connector;
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	struct clk			*refclk;
	struct drm_panel		*panel;
	struct gpio_desc		*enable_gpio;
	struct regulator_bulk_data      supplies[SN_REGULATOR_SUPPLY_NUM];
	struct ti_sn_bridge_config	*config;
};

static const struct regmap_range ti_sn_bridge_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xFF },
};

static const struct regmap_access_table ti_sn_bridge_volatile_table = {
	.yes_ranges = ti_sn_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ti_sn_bridge_volatile_ranges),
};

static const struct regmap_config ti_sn_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &ti_sn_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static void ti_sn_bridge_write_u16(struct ti_sn_bridge *pdata,
				   unsigned int reg, u16 val)
{
	regmap_write(pdata->regmap, reg, val & 0xFF);
	regmap_write(pdata->regmap, reg + 1, val >> 8);
}

static int __maybe_unused ti_sn_bridge_resume(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret) {
		DRM_ERROR("failed to enable supplies %d\n", ret);
		return ret;
	}

	gpiod_set_value(pdata->enable_gpio, 1);

	return ret;
}

static int __maybe_unused ti_sn_bridge_suspend(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	gpiod_set_value(pdata->enable_gpio, 0);

	ret = regulator_bulk_disable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret)
		DRM_ERROR("failed to disable supplies %d\n", ret);

	return ret;
}

static const struct dev_pm_ops ti_sn_bridge_pm_ops = {
	SET_RUNTIME_PM_OPS(ti_sn_bridge_suspend, ti_sn_bridge_resume, NULL)
};

/* Connector funcs */
static struct ti_sn_bridge *
connector_to_ti_sn_bridge(struct drm_connector *connector)
{
	return container_of(connector, struct ti_sn_bridge, connector);
}

static int ti_sn_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);

	return drm_panel_get_modes(pdata->panel);
}

static enum drm_mode_status
ti_sn_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	/* maximum supported resolution is 4K at 60 fps */
	if (mode->clock > MAX_DSI_CLK_FREQ_KHZ)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs ti_sn_bridge_connector_helper_funcs = {
	.get_modes = ti_sn_bridge_connector_get_modes,
	.mode_valid = ti_sn_bridge_connector_mode_valid,
};

static enum drm_connector_status
ti_sn_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	/**
	 * TODO: Currently if drm_panel is present, then always
	 * return the status as connected. Need to add support to detect
	 * device state for hot pluggable scenarios.
	 */
	return connector_status_connected;
}

static const struct drm_connector_funcs ti_sn_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ti_sn_bridge_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct ti_sn_bridge *bridge_to_ti_sn_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ti_sn_bridge, bridge);
}

static int ti_sn_bridge_parse_regulators(struct ti_sn_bridge *pdata)
{
	unsigned int i;
	const char * const ti_sn_bridge_supply_names[] = {
		"vcca", "vcc", "vccio", "vpll",
	};

	for (i = 0; i < SN_REGULATOR_SUPPLY_NUM; i++)
		pdata->supplies[i].supply = ti_sn_bridge_supply_names[i];

	return devm_regulator_bulk_get(pdata->dev, SN_REGULATOR_SUPPLY_NUM,
				       pdata->supplies);
}

static int ti_sn_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "ti_sn_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &ti_sn_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&pdata->connector,
				 &ti_sn_bridge_connector_helper_funcs);
	drm_connector_attach_encoder(&pdata->connector, bridge->encoder);

	/*
	 * TODO: ideally finding host resource and dsi dev registration needs
	 * to be done in bridge probe. But some existing DSI host drivers will
	 * wait for any of the drm_bridge/drm_panel to get added to the global
	 * bridge/panel list, before completing their probe. So if we do the
	 * dsi dev registration part in bridge probe, before populating in
	 * the global bridge list, then it will cause deadlock as dsi host probe
	 * will never complete, neither our bridge probe. So keeping it here
	 * will satisfy most of the existing host drivers. Once the host driver
	 * is fixed we can move the below code to bridge probe safely.
	 */
	host = of_find_mipi_dsi_host_by_node(pdata->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	dsi->lanes = pdata->config->sn_dsi_lane_count;
	dsi->format = MIPI_DSI_FMT_RGB888;

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_HSE;
	if (pdata->config->dsi_burst_mode)
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
	else
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_EOT_PACKET;

	/* check if continuous dsi clock is required or not */
	//pm_runtime_get_sync(pdata->dev);
	//regmap_read(pdata->regmap, SN_LVDSPLL_SRC_REG, &val);
	//pm_runtime_put(pdata->dev);
	//if (!(val & HS_CLK_SRC))
	// TODO DM: No auto detection in DSI84, requires setup.
	//dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	pdata->dsi = dsi;

	/* attach panel to bridge */
	drm_panel_attach(pdata->panel, &pdata->connector);

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_host:
	drm_connector_cleanup(&pdata->connector);
	return ret;
}

static void ti_sn_bridge_disable(struct drm_bridge *bridge)
{
	/*
	 * When the system requires to stop outputting video to the display, it is recommended to use the following
	 * sequence for the SN65DSI84:
	 *	1. Clear the PLL_EN bit to 0 (CSR 0x0D.0)
	 *	2. Stop video streaming on DSI inputs
	 *	3. Drive all DSI data lanes to LP11, but keep the DSI CLK lanes in HS.
	 */

	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	drm_panel_disable(pdata->panel);

	/* Clear PLL_EN bit */
	regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 0);

	drm_panel_unprepare(pdata->panel);
}

static u32 ti_sn_bridge_get_dsi_freq(struct ti_sn_bridge *pdata)
{
        u32 bit_rate_khz, clk_freq_khz;
        struct drm_display_mode *mode =
                &pdata->bridge.encoder->crtc->state->adjusted_mode;

        bit_rate_khz = mode->clock *
                        mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
        clk_freq_khz = bit_rate_khz / (pdata->dsi->lanes * 2);
        return clk_freq_khz;
}

static void ti_sn_bridge_set_lvds_clk_range(struct ti_sn_bridge *pdata)
{
	struct drm_display_mode *mode =
			&pdata->bridge.encoder->crtc->state->adjusted_mode;
	u32 lvds_freq_khz = mode->clock / pdata->config->lvds_channel_count;

	if(lvds_freq_khz < 25000)
		DRM_ERROR("SN65DSI84 LVDS Frequency too low! (%d khz)", lvds_freq_khz);
	else if((lvds_freq_khz >= 25000) && (lvds_freq_khz < 37500))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(0));
	else if((lvds_freq_khz >= 37500) && (lvds_freq_khz < 62500))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(1));
	else if((lvds_freq_khz >= 62500) && (lvds_freq_khz < 87500))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(2));
	else if((lvds_freq_khz >= 87500) && (lvds_freq_khz < 112500))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(3));
	else if((lvds_freq_khz >= 112500) && (lvds_freq_khz < 137500))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(4));
	else if((lvds_freq_khz >= 137500) && (lvds_freq_khz <= 154000))
		regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE(5));
	else
		DRM_ERROR("SN65DSI84 LVDS Frequency too high! (%d khz)", lvds_freq_khz);
}

static void ti_sn_bridge_set_dsi_rate(struct ti_sn_bridge *pdata)
{
	unsigned int clk_freq_mhz;
	unsigned int val;

	/* kHz->mHz conversion to prevent accuracy loss */
	clk_freq_mhz = ti_sn_bridge_get_dsi_freq(pdata) / 1000;

	/* for each increment in val, frequency increases by 5MHz */
	val = (MIN_DSI_CLK_FREQ_MHZ / 5) +
		(((clk_freq_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF);

	regmap_write(pdata->regmap, SN_DSIA_CLK_RANGE_REG, val);
}

// Function prints SN65DSI84 register map in format similar to i2cdump
static void ti_sn_bridge_dump_regmap(struct ti_sn_bridge *pdata){
	unsigned int map[256], i;

	DRM_DEBUG_DRIVER("SN65DSI84 Input DSI frequency is %d kHz", ti_sn_bridge_get_dsi_freq(pdata));

	for(i=0; i<=0xFF; i++){
		regmap_read(pdata->regmap, i, &map[i]);
	}

	DRM_DEBUG_DRIVER("SN65DSI84 Register map dump:");
	DRM_DEBUG_DRIVER("     x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF");
	DRM_DEBUG_DRIVER("     -----------------------------------------------");

	for(i=0; i<=0xF; i++){
			DRM_DEBUG_DRIVER("%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				i,
				map[((i*16)+0)], map[((i*16)+1)], map[((i*16)+2)], map[((i*16)+3)],
				map[((i*16)+4)], map[((i*16)+5)], map[((i*16)+6)], map[((i*16)+7)],
				map[((i*16)+8)], map[((i*16)+9)], map[((i*16)+10)], map[((i*16)+11)],
				map[((i*16)+12)], map[((i*16)+13)], map[((i*16)+14)], map[((i*16)+15)]);
	}
}

static void ti_sn_bridge_configure(struct ti_sn_bridge *pdata){
	unsigned int val;

	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/*
	 * Init seq 1 - power on -> done automatically
	 * Init seq 2 - set DSI clock to continuous mode -> done in attach
	 * Init seq 3 - Set en pin low -> cannot control
	 * Init seq 4 - Set en pin high -> cannot control
	 * Init seq 5 - Initialize all CSR registers:
	 */

	/* Reset and Clock registers */

	/* Setting 0x0A register */
	/* Set refclk source to DSI clock */
	regmap_update_bits(pdata->regmap, SN_LVDSPLL_SRC_REG, HS_CLK_SRC, 1);
	/* Set LVDS clock range */
	ti_sn_bridge_set_lvds_clk_range(pdata);

	/* Setting 0x0B register */
	/* Set dsi clock divider */
	regmap_update_bits(pdata->regmap, SN_CLK_REG, DSI_CLK_DIV_MASK, DSI_CLK_DIV(pdata->config->sn_dsi_clk_divider));
	/* Set REFCLK_MULTIPLIER to 0 */
	regmap_update_bits(pdata->regmap, SN_CLK_REG, REFCLK_MULT_MASK, 0);


	/* DSI registers */

	/* Setting 0x10 register */
	/* Set number of DSIA lanes */
	val = CHA_DSI_LANES(4 - pdata->config->sn_dsi_lane_count);
	regmap_update_bits(pdata->regmap, SN_DSI_LANES_REG,
			   CHA_DSI_LANES_MASK, val);
	/* Set SOT_ERR_TOL_DIS bit to 0 */
	regmap_update_bits(pdata->regmap, SN_DSI_LANES_REG,
			   SOT_ERR_TOL_DIS, 0);

	/* Setting 0x11 register */
	regmap_update_bits(pdata->regmap, SN_DSIA_EQ_REG, CHA_DSI_DATA_EQ_MASK, 0);
	regmap_update_bits(pdata->regmap, SN_DSIA_EQ_REG, CHA_DSI_CLK_EQ_MASK, 0);

	/* Setting 0x12 register */
	/* Set dsi clk frequency range value */
	ti_sn_bridge_set_dsi_rate(pdata);


	/* LVDS registers */

	/* Setting 0x18 register */
	/* Set DE_NEG_POLARITY */
	if (pdata->config->lvds_de_neg_polarity)
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, DE_NEG_POL, DE_NEG_POL);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, DE_NEG_POL, 0);
	/* Set HS_NEG_POLARITY */
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, HS_NEG_POL, 0);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, HS_NEG_POL, HS_NEG_POL);
	/* Set VS_NEG_POLARITY */
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, VS_NEG_POL, 0);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, VS_NEG_POL, VS_NEG_POL);
	/* Set LVDS_LINK_CFG */
	if(pdata->config->lvds_channel_count == 2)
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, LVDS_LINK_CFG, 0);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, LVDS_LINK_CFG, LVDS_LINK_CFG);
	/* Set CHA_24BPP_MODE and CHB_24BPP_MODE*/
	if(pdata->config->lvds_set_24bpp_mode){
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHA_24BPP_MODE, CHA_24BPP_MODE);
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHB_24BPP_MODE, CHB_24BPP_MODE);
	}
	else {
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHA_24BPP_MODE, 0);
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHB_24BPP_MODE, 0);
	}
	/* Set CHA_24BPP_FORMAT1 and CHB_24BPP_FORMAT1*/
	if(pdata->config->lvds_set_24bpp_format1){
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHA_24BPP_FORMAT1, CHA_24BPP_FORMAT1);
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHB_24BPP_FORMAT1, CHB_24BPP_FORMAT1);
	}
	else {
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHA_24BPP_FORMAT1, 0);
		regmap_update_bits(pdata->regmap, SN_LVDS0_REG, CHB_24BPP_FORMAT1, 0);
	}

	/* Setting 0x19 register */
	/* Set CHA_LVDS_VOCM and CHB_LVDS_VOCM */
	if(pdata->config->lvds_cha_vocm)
		regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHA_LVDS_VOCM, CHA_LVDS_VOCM);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHA_LVDS_VOCM, 0);
	if(pdata->config->lvds_chb_vocm)
		regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHB_LVDS_VOCM, CHB_LVDS_VOCM);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHB_LVDS_VOCM, 0);
	/* Set CHA_LVDS_VOD_SWING and CHB_LVDS_VOD_SWING */
	regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHA_LVDS_VOD_SWING_MASK, CHA_LVDS_VOD_SWING(pdata->config->lvds_cha_vod_swing));
	regmap_update_bits(pdata->regmap, SN_LVDS1_REG, CHB_LVDS_VOD_SWING_MASK, CHB_LVDS_VOD_SWING(pdata->config->lvds_chb_vod_swing));

	/* Setting 0x1A register */
	/* Set EVEN_ODD_SWAP */
	if(pdata->config->lvds_even_odd_swap)
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, EVEN_ODD_SWAP, EVEN_ODD_SWAP);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, EVEN_ODD_SWAP, 0);
	/* Set CHA_REVERSE_LVDS */
	if(pdata->config->lvds_cha_reverse)
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHA_REVERSE_LVDS, CHA_REVERSE_LVDS);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHA_REVERSE_LVDS, 0);
	/* Set CHB_REVERSE_LVDS */
	if(pdata->config->lvds_chb_reverse)
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHB_REVERSE_LVDS, CHB_REVERSE_LVDS);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHB_REVERSE_LVDS, 0);
	/* Set CHA_LVDS_TERM */
	if(pdata->config->lvds_cha_term)
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHA_LVDS_TERM, CHA_LVDS_TERM);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHA_LVDS_TERM, 0);
	/* Set CHB_LVDS_TERM */
	if(pdata->config->lvds_chb_term)
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHB_LVDS_TERM, CHB_LVDS_TERM);
	else
		regmap_update_bits(pdata->regmap, SN_LVDS2_REG, CHB_LVDS_TERM, 0);

	/* Setting 0x1B register */
	/* Set CHA_LVDS_CM_ADJUST */
	regmap_update_bits(pdata->regmap, SN_LVDS3_REG, CHA_LVDS_CM_ADJUST_MASK, CHA_LVDS_CM_ADJUST(pdata->config->lvds_cha_cm_adjust));
	/* Set CHB_LVDS_CM_ADJUST */
	regmap_update_bits(pdata->regmap, SN_LVDS3_REG, CHB_LVDS_CM_ADJUST_MASK, pdata->config->lvds_chb_cm_adjust);


	/* VIDEO registers */
	/* Setting 0x20 and 0x21 registers - CHA_ACTIVE_LINE_LENGTH */
	ti_sn_bridge_write_u16(pdata, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,
			       mode->hdisplay);

	/* Setting 0x24 and 0x25 registers - CHA_VERTICAL_DISPLAY_SIZE */
	ti_sn_bridge_write_u16(pdata, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,
			       mode->vdisplay);

	/* Setting 0x28 and 0x29 registers - CHA_SYNC_DELAY */
	ti_sn_bridge_write_u16(pdata, SN_CHA_SYNC_DELAY_LOW_REG,
			       pdata->config->sn_sync_delay);

	/* Setting 0x2C and 0x2D registers - CHA_HSYNC_PULSE_WIDTH */
	ti_sn_bridge_write_u16(pdata, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
			       (mode->hsync_end - mode->hsync_start) / pdata->config->lvds_channel_count);

	/* Setting 0x30 and 0x31 registers - CHA_VSYNC_PULSE_WIDTH */
	ti_sn_bridge_write_u16(pdata, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
			       mode->vsync_end - mode->vsync_start);

	/* Setting 0x34 register - CHA_HORIZONTAL_BACK_PORCH */
	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		     ((mode->htotal - mode->hsync_end) / pdata->config->lvds_channel_count) & 0xFF);


	/* Test Pattern registers */

	/* Setting 0x36 register - CHA_VERTICAL_BACK_PORCH */
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_BACK_PORCH_REG,
		     (mode->vtotal - mode->vsync_end) & 0xFF);

	/* Setting 0x38 register - CHA_HORIZONTAL_FRONT_PORCH */
	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		     ((mode->hsync_start - mode->hdisplay) / pdata->config->lvds_channel_count) & 0xFF);

	/* Setting 0x3A register - CHA_VERTICAL_FRONT_PORCH */
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		     (mode->vsync_start - mode->vdisplay) & 0xFF);

	/* Setting 0x3C register bit 4 - CHA_TEST_PATTERN */
	/* Disable test pattern */
	regmap_write(pdata->regmap, SN_CHA_TEST_PATTERN_REG, 0);

	usleep_range(10000, 10500); /* 10ms delay recommended by spec */
}

static void ti_sn_bridge_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	unsigned int val;
	int ret;

	/* Init seq 6+: Enable PLL, wait 10ms*/
	regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 1);
	msleep(10);

	/* Init seq 7: soft reset, wait 10ms - believed to cause issues*/
	//regmap_write(pdata->regmap, SN_SOFT_RESET_REG, 1);
	//msleep(10);

	ret = regmap_read_poll_timeout(pdata->regmap, SN_LVDSPLL_SRC_REG, val,
					val & PLL_EN_STAT, 1000,
					50 * 1000);
	if (ret) {
		DRM_ERROR("SN65DSI84 LVDS PLL did not lock. (%d)\n", ret);
		return;
	}

	/* Enable the data */
	drm_panel_enable(pdata->panel);
	msleep(5);

	/* Check for errors */
	regmap_write(pdata->regmap, SN_ERR_REG, 0xFF); // 0xFF clear the register.
	msleep(10);
	ret = regmap_read(pdata->regmap, SN_ERR_REG, &val);

	/* TODO: If any error occurs, repeated initialization should be done. */
	if(val != 0){
		if(val & CHA_SYNCH_ERR)
			DRM_ERROR("SN65DSI84 DSI synchronization error.");
		if(val & CHA_CRC_ERR)
			DRM_ERROR("SN65DSI84 DSI CRC error.");
		if(val & CHA_UNC_ECC_ERR)
			DRM_ERROR("SN65DSI84 uncorrectable ECC error.");
		if(val & CHA_COR_ECC_ERR)
			DRM_ERROR("SN65DSI84 correctable ECC error.");
		if(val & CHA_LLP_ERR)
			DRM_ERROR("SN65DSI84 low level protocol error.");
		if(val & CHA_SOT_BIT_ERR)
			DRM_ERROR("SN65DSI84 SoT leader sequence bit error.");
		if(val & PLL_UNLOCK)
			DRM_ERROR("SN65DSI84 lost PLL lock.");
	}
	else {
		DRM_DEBUG_DRIVER("SN65DSI84 enable OK (no errors within 10ms).");
	}

	/* Everything should be running now, dump the regmap for debug */
	ti_sn_bridge_dump_regmap(pdata);

	/* Enable test pattern - debugging purposes*/
//	regmap_write(pdata->regmap, SN_CHA_TEST_PATTERN_REG, BIT(4));
}


static void ti_sn_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	pm_runtime_get_sync(pdata->dev);

	// Set all internal registers except PLL enable
	ti_sn_bridge_configure(pdata);

	drm_panel_prepare(pdata->panel);
}

static void ti_sn_bridge_post_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);

	pm_runtime_put_sync(pdata->dev);
}

static const struct drm_bridge_funcs ti_sn_bridge_funcs = {
	.attach = ti_sn_bridge_attach,
	.pre_enable = ti_sn_bridge_pre_enable,
	.enable = ti_sn_bridge_enable,
	.disable = ti_sn_bridge_disable,
	.post_disable = ti_sn_bridge_post_disable,
};

static int ti_sn_bridge_parse_dsi_host(struct ti_sn_bridge *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}

	return 0;
}

static struct ti_sn_bridge_config* ti_sn_bridge_get_config(struct i2c_client *client)
{
	struct ti_sn_bridge_config *bridge_config;
	struct device_node *bridge_node = client->dev.of_node;
	struct device_node *endpoint;

	endpoint = of_graph_get_next_endpoint(bridge_node, NULL);
	if (!endpoint) {
		DRM_DEV_ERROR(&client->dev, "Cannot find OF endpoint.\n");
		return NULL;
	}

	bridge_config = devm_kzalloc(&client->dev, sizeof(*bridge_config), GFP_KERNEL);
	if (!bridge_config) {
		DRM_DEV_ERROR(&client->dev, "Failed to kzalloc bridge_config.\n");
		of_node_put(endpoint);
		return NULL;
	}

	bridge_config->lvds_de_neg_polarity = of_property_read_bool(bridge_node, "lvds-de-neg-polarity");
	bridge_config->lvds_set_24bpp_mode = of_property_read_bool(bridge_node, "lvds-set-24bpp-mode");
	bridge_config->lvds_set_24bpp_format1 = of_property_read_bool(bridge_node, "lvds-set-24bpp-format1");
	bridge_config->lvds_cha_vocm = of_property_read_bool(bridge_node, "lvds-cha-vocm");
	bridge_config->lvds_chb_vocm = of_property_read_bool(bridge_node, "lvds-chb-vocm");
	bridge_config->lvds_even_odd_swap = of_property_read_bool(bridge_node, "lvds-even-odd-swap");
	bridge_config->lvds_cha_reverse = of_property_read_bool(bridge_node, "lvds-cha-reverse");
	bridge_config->lvds_chb_reverse = of_property_read_bool(bridge_node, "lvds-chb-reverse");
	bridge_config->lvds_cha_term = of_property_read_bool(bridge_node, "lvds-cha-term");
	bridge_config->lvds_chb_term = of_property_read_bool(bridge_node, "lvds-chb-term");
	bridge_config->dsi_burst_mode = of_property_read_bool(bridge_node, "dsi-burst-mode");

	if (!bridge_config->lvds_set_24bpp_mode) {
		bridge_config->lvds_set_24bpp_format1 = 1;
	}

	if (of_property_read_u32(bridge_node, "sn-dsi-lane-count", &bridge_config->sn_dsi_lane_count) < 0) {
		DRM_DEV_INFO(&client->dev, "sn-dsi-lane-count property not found in DTS, setting to 4.\n");
		bridge_config->sn_dsi_lane_count = 4;
	} else {
		if (bridge_config->sn_dsi_lane_count < 1 || bridge_config->sn_dsi_lane_count > 4 ) {
			DRM_DEV_ERROR(&client->dev, "sn-dsi-lane-count property must be 1-4 not %u.\n", bridge_config->sn_dsi_lane_count);
			goto ti_sn_bridge_get_config_dts_fail_out;
		}
	}

	if (of_property_read_u32(bridge_node, "lvds-cha-vod-swing", &bridge_config->lvds_cha_vod_swing) < 0) {
		DRM_DEV_INFO(&client->dev, "lvds-cha-vod-swing property not found in DTS, setting to 1.\n");
		bridge_config->lvds_cha_vod_swing = 1;
	}
	if (of_property_read_u32(bridge_node, "lvds-chb-vod-swing", &bridge_config->lvds_chb_vod_swing) < 0) {
		DRM_DEV_INFO(&client->dev, "lvds-chb-vod-swing property not found in DTS, setting to 1.\n");
		bridge_config->lvds_chb_vod_swing = 1;
	}

	if (bridge_config->lvds_cha_vocm) {
		DRM_DEV_INFO(&client->dev, "lvds_cha_vocm is set in dts, setting lvds_cha_cm_adjust to 1.\n");
		bridge_config->lvds_cha_cm_adjust = 1;
	} else {
		if (of_property_read_u32(bridge_node, "lvds-cha-cm-adjust", &bridge_config->lvds_cha_cm_adjust) < 0) {
			DRM_DEV_INFO(&client->dev, "lvds-cha-cm-adjust property not found in DTS, setting to 0.\n");
			bridge_config->lvds_cha_cm_adjust = 0;
		}
	}
	if (bridge_config->lvds_chb_vocm) {
		DRM_DEV_INFO(&client->dev, "lvds_chb_vocm is set in dts, setting lvds_chb_cm_adjust to 1.\n");
		bridge_config->lvds_chb_cm_adjust = 1;
	} else {
		if (of_property_read_u32(bridge_node, "lvds-chb-cm-adjust", &bridge_config->lvds_chb_cm_adjust) < 0) {
			DRM_DEV_INFO(&client->dev, "lvds-chb-cm-adjust property not found in DTS, setting to 0.\n");
			bridge_config->lvds_chb_cm_adjust = 0;
		}
	}

	if (of_property_read_u32(bridge_node, "sn-sync-delay", &bridge_config->sn_sync_delay) < 0) {
		DRM_DEV_INFO(&client->dev, "sn-sync-delay property not found in DTS, setting to 32.\n");
		bridge_config->sn_sync_delay = 32;
	}

	if (of_property_read_u32(bridge_node, "lvds-channel-count", &bridge_config->lvds_channel_count) < 0) {
		DRM_DEV_INFO(&client->dev, "lvds-channel-count property not found in DTS, setting to 1.\n");
		bridge_config->lvds_channel_count = 1;
	} else {
		if (bridge_config->lvds_channel_count < 1 || bridge_config->lvds_channel_count > 2 ) {
			DRM_DEV_ERROR(&client->dev, "lvds-channel-count property must be 1-2 not %u.", bridge_config->lvds_channel_count);
			goto ti_sn_bridge_get_config_dts_fail_out;
		}
	}

	if (of_property_read_u32(bridge_node, "sn-dsi-clk-divider", &bridge_config->sn_dsi_clk_divider) < 0) {
		if (bridge_config->lvds_channel_count == 1) {
			bridge_config->sn_dsi_clk_divider = 2;
		} else {
			bridge_config->sn_dsi_clk_divider = 5;
		}
		DRM_DEV_INFO(&client->dev, "sn-dsi-clk-divider property not found in DTS, setting register to %u.\n", bridge_config->sn_dsi_clk_divider);
	} else {
		bridge_config->sn_dsi_clk_divider = bridge_config->sn_dsi_clk_divider - 1;
	}

	of_node_put(endpoint);
	return bridge_config;

ti_sn_bridge_get_config_dts_fail_out:
	devm_kfree(&client->dev, bridge_config);
	of_node_put(endpoint);
	return NULL;
}

static int ti_sn_bridge_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct ti_sn_bridge *pdata;
	struct ti_sn_bridge_config *bridge_config;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	bridge_config = ti_sn_bridge_get_config(client);
	if (!bridge_config) {
		DRM_ERROR("Cannot read configuration from dts.\n");
		return-EINVAL;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct ti_sn_bridge),
			     GFP_KERNEL);
	if (!pdata) {
		DRM_ERROR("Cannot allocate memory.\n");
		ret = -ENOMEM;
		goto free_config;
	}

	pdata->regmap = devm_regmap_init_i2c(client,
					     &ti_sn_bridge_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		DRM_ERROR("regmap i2c init failed\n");
		return PTR_ERR(pdata->regmap);
	}

	pdata->dev = &client->dev;
	pdata->config = bridge_config;

	ret = drm_of_find_panel_or_bridge(pdata->dev->of_node, 1, 0,
					  &pdata->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return ret;
	}

	dev_set_drvdata(&client->dev, pdata);

	pdata->enable_gpio = devm_gpiod_get(pdata->dev, "enable",
					    GPIOD_OUT_LOW);
	if (IS_ERR(pdata->enable_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->enable_gpio);
		return ret;
	}

	ret = ti_sn_bridge_parse_regulators(pdata);
	if (ret) {
		DRM_ERROR("failed to parse regulators\n");
		return ret;
	}

	pdata->refclk = devm_clk_get(pdata->dev, "refclk");
	if (IS_ERR(pdata->refclk)) {
		ret = PTR_ERR(pdata->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		DRM_DEBUG_KMS("refclk not found\n");
		pdata->refclk = NULL;
	}

	ret = ti_sn_bridge_parse_dsi_host(pdata);
	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->bridge.funcs = &ti_sn_bridge_funcs;
	pdata->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&pdata->bridge);

	return 0;

free_config:
	kfree(bridge_config);
	return ret;
}

static int ti_sn_bridge_remove(struct i2c_client *client)
{
	struct ti_sn_bridge *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id ti_sn_bridge_id[] = {
	{ "ti,sn65dsi84", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ti_sn_bridge_id);

static const struct of_device_id ti_sn_bridge_match_table[] = {
	{.compatible = "ti,sn65dsi84"},
	{},
};
MODULE_DEVICE_TABLE(of, ti_sn_bridge_match_table);

static struct i2c_driver ti_sn_bridge_driver = {
	.driver = {
		.name = "ti_sn65dsi84",
		.of_match_table = ti_sn_bridge_match_table,
		.pm = &ti_sn_bridge_pm_ops,
	},
	.probe = ti_sn_bridge_probe,
	.remove = ti_sn_bridge_remove,
	.id_table = ti_sn_bridge_id,
};
module_i2c_driver(ti_sn_bridge_driver);

MODULE_AUTHOR("Daniel Machaty <daniel.machaty@kontron.com>");
MODULE_DESCRIPTION("sn65dsi84 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");

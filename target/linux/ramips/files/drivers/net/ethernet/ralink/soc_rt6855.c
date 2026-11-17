// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2009-2015 John Crispin <blogic at openwrt.org>
 * Copyright (C) 2009-2015 Felix Fietkau <nbd at nbd.name>
 * Copyright (C) 2013-2015 Michael Lee <igvtee at gmail.com>
 * Copyright (C) 2026 Nickolay Savchenko <n.savchenko@axioma.lv>
 * Copyright (C) 2026 booqoffsky
 */

#include <linux/module.h>

#include <asm/mach-ralink/ralink_regs.h>

#include "gsw_rt6856.h"
#include "mtk_eth_soc.h"

static int rt6855_mdio_read(struct mii_bus *bus, int phy, int reg)
{
	return rt6856_gsw_mdio_read(phy, reg);
}

static int rt6855_mdio_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	return rt6856_gsw_mdio_write(phy, reg, val);
}

static const u16 rt6855_reg_table[FE_REG_COUNT] = {
	[FE_REG_PDMA_GLO_CFG] = RT5350_PDMA_GLO_CFG,
	[FE_REG_PDMA_RST_CFG] = RT5350_PDMA_RST_CFG,
	[FE_REG_DLY_INT_CFG] = RT5350_DLY_INT_CFG,
	[FE_REG_TX_BASE_PTR0] = RT5350_TX_BASE_PTR0,
	[FE_REG_TX_MAX_CNT0] = RT5350_TX_MAX_CNT0,
	[FE_REG_TX_CTX_IDX0] = RT5350_TX_CTX_IDX0,
	[FE_REG_TX_DTX_IDX0] = RT5350_TX_DTX_IDX0,
	[FE_REG_RX_BASE_PTR0] = RT5350_RX_BASE_PTR0,
	[FE_REG_RX_MAX_CNT0] = RT5350_RX_MAX_CNT0,
	[FE_REG_RX_CALC_IDX0] = RT5350_RX_CALC_IDX0,
	[FE_REG_RX_DRX_IDX0] = RT5350_RX_DRX_IDX0,
	[FE_REG_FE_INT_ENABLE] = RT5350_FE_INT_ENABLE,
	[FE_REG_FE_INT_STATUS] = RT5350_FE_INT_STATUS,
	[FE_REG_FE_RST_GL] = FE_FE_RST_GL,
	[FE_REG_FE_DMA_VID_BASE] = 0,
};

static void rt6855_init_data(struct fe_soc_data *data,
			     struct net_device *netdev)
{
	struct fe_priv *priv = netdev_priv(netdev);
	int err;

	/*
	 * Bring the SoC-internal GSW up on the boards that request it
	 * through the "ralink,gsw-syscon" phandle. This must happen at
	 * probe time, before userspace starts: /etc/init.d/network loads
	 * the swconfig switch config once, before netifd brings eth0 up,
	 * so a switch registered at first interface up would never be
	 * configured. priv->dev is assigned later in fe_probe, so pass the
	 * netdev's parent (SET_NETDEV_DEV already set it to &pdev->dev).
	 */
	err = rt6856_gsw_init(netdev->dev.parent);
	if (err)
		dev_err(netdev->dev.parent,
			"switch bring-up failed: %d\n", err);

	/*
	 * Do not inherit RT3050 ESW flags: boards either use the internal
	 * GSW brought up above or an external switch (the RTL8367RB on
	 * the Keenetic Giga II). Quiesce PDMA across bootloader handoff
	 * and timeout recovery.
	 */
	priv->flags = FE_FLAG_DMA_STOP_WAIT;
	netdev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM;
}

static int rt6855_fwd_config(struct fe_priv *priv)
{
	/*
	 * RT6855/RT6856 uses the non-RT5350 GDMA receive path. Route every
	 * frame class to the CPU explicitly: the forwarding state left by
	 * the bootloader cannot be relied on.
	 */
	fe_fwd_config(priv);
	fe_csum_config(priv);

	return 0;
}

static int rt6855_has_carrier(struct fe_priv *priv)
{
	/*
	 * Only the internal GSW reports a link (its CPU port is forced up
	 * during bring-up); boards with an external switch have no link
	 * information, as before.
	 */
	return rt6856_gsw_has_carrier();
}

static struct fe_soc_data rt6855_data = {
	.init_data = rt6855_init_data,
	.reg_table = rt6855_reg_table,
	.fwd_config = rt6855_fwd_config,
	.has_carrier = rt6855_has_carrier,
	.pdma_glo_cfg = FE_PDMA_SIZE_32DWORDS,
	.checksum_bit = RX_DMA_L4VALID,
	.rx_int = RT5350_RX_DONE_INT,
	.tx_int = RT5350_TX_DONE_INT,
	.mdio_read = rt6855_mdio_read,
	.mdio_write = rt6855_mdio_write,
};

const struct of_device_id of_fe_match[] = {
	{ .compatible = "ralink,rt6855-eth", .data = &rt6855_data },
	{},
};

MODULE_DEVICE_TABLE(of, of_fe_match);

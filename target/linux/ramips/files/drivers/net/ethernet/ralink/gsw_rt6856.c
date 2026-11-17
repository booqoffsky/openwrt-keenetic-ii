// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ralink RT6856 integrated Ethernet switch bring-up.
 *
 * The switch block shares the frame-engine register window: it starts at
 * FE + 0x8000 and exposes an MT7530-compatible layout for the parts the
 * bring-up needs — the per-port VLAN group at 0x2000 + p * 0x100, the
 * port MAC control/status group at 0x3000 + p * 0x100 and the MDIO
 * master/interrupt group at 0x7000. The five internal EPHYs hang off
 * that MDIO master (PIAC at switch + 0x7004).
 *
 * The compatibility is not complete: the indirect table engines (ATC at
 * 0x80, VTCR at 0x90) never complete on this SoC, and a write to switch
 * + 0x3ff0 stalls the bus permanently. Never write to unverified switch
 * registers: with the watchdog disarmed such a stall is unrecoverable.
 *
 * The switch has no device node of its own: the frame-engine driver owns
 * the whole register window and runs this bring-up from its SoC hook
 * when a board requests the internal switch through a
 * "ralink,gsw-syscon" phandle on the ethernet node.
 *
 * The bring-up sequence replicates the vendor initialization, with
 * deviations noted inline. Reset and clock gates of the switch live in
 * the palmbus syscon window at 0x1fb00834 (the "pcie_reset" syscon node
 * in the device tree, whose register window extends to 0x860 to also
 * cover the EPHY clock enable).
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/phy.h>
#include <linux/regmap.h>

#include "mt7530.h"
#include "mtk_eth_soc.h"
#include "gsw_rt6856.h"

/* Switch block offset within the frame-engine window. */
#define RT6856_GSW_OFFSET		0x8000
#define RT6856_GSW_REG(off)		(RT6856_GSW_OFFSET + (off))

/* Miscellany and address table (MT7530-compatible): the ATC busy bit is
 * 15, unlike the VLAN table (VTCR) busy at bit 31. */
#define RT6856_GSW_MFC			0x10
#define RT6856_GSW_ATC			0x80
#define RT6856_GSW_ATC_BUSY		BIT(15)
#define RT6856_GSW_ATC_CMD_FLUSH_ALL	BIT(1)

/* Per-port VLAN and MAC control/status groups. */
#define RT6856_GSW_PMCR(p)		(0x3000 + (p) * 0x100)
#define RT6856_GSW_PMSR(p)		(0x3008 + (p) * 0x100)
#define RT6856_GSW_PCR(p)		(0x2004 + (p) * 0x100)
#define RT6856_GSW_PVC(p)		(0x2010 + (p) * 0x100)

/*
 * Port 0 block registers written by the bring-up beyond PMCR/PMSR; their
 * fields are not decoded, replicate the values verbatim.
 */
#define RT6856_GSW_PORT_E0		0x30e0
#define RT6856_GSW_PORT_F0		0x30f0
#define RT6856_GSW_PORT_E0_CFG		0x1e09
#define RT6856_GSW_PORT_F0_CLEAR	0x30

/*
 * MDIO master, interrupt and PHY-polling registers of the switch block,
 * the same group the MT7620 GSW exposes at its +0x7000: PIAC at 0x7004
 * (MT7620A_GSW_REG_PIAC), IMR at 0x7008, ISR at 0x700c, the MDIO
 * clock/preamble and PHY-polling configuration at 0x7010 and GPC1 at
 * 0x7014. The command encoding matches the GSW_MDIO_* bits of the
 * MT7620 GSW (start bit 16, write bit 18, read bit 19, busy bit 31).
 */
#define RT6856_GSW_PIAC			RT6856_GSW_REG(0x7004)
#define RT6856_GSW_PIAC_BUSY		BIT(31)
#define RT6856_GSW_PIAC_WRITE		0x00050000	/* START | WRITE */
#define RT6856_GSW_PIAC_READ		0x00090000	/* START | READ */
#define RT6856_GSW_IMR			RT6856_GSW_REG(0x7008)
#define RT6856_GSW_ISR			RT6856_GSW_REG(0x700c)
#define RT6856_GSW_PHY_POLLING		RT6856_GSW_REG(0x7010)
#define RT6856_GSW_GPC1			RT6856_GSW_REG(0x7014)

/*
 * EPHY clock gate in the palmbus syscon window that starts at
 * 0x1fb00834 ("pcie_reset" in the device tree): the EPHY clock enable
 * lives at offset 0x2c (0x1fb00860). The reset bits of 0x834 do not
 * latch writes from the CPU, so enabling the clocks is the only
 * effective step. Bit 27 of 0x834 belongs to the PCIe driver, which sets
 * it when the second PCIe lane is absent.
 */
#define RT6856_GATE_CLK_OFF		0x2c
#define RT6856_GATE_CLK_MASK		0x3e00

/* Force mode with speed, duplex and link cleared: port held down. */
#define RT6856_PMCR_LINK_DOWN		0x8000
/* CPU port forced up (1G full duplex, flow control). */
#define RT6856_PMCR_CPU			0x5e33b
/*
 * Hardware-managed MAC configuration for the five EPHY data ports:
 * MAC_MODE | IPG | TX_EN | RX_EN | BACKOFF | BACKPRES | TX_FC | RX_FC
 * — no force bits, so the MAC follows the PHY link state. The ports are
 * brought up with this value only after the EPHY tuning is done.
 */
#define RT6856_PMCR_EPHY_PORT		0x56330

/*
 * Single-shot values of the vendor bring-up sequence, replicated
 * verbatim where the fields are not decoded.
 */
/* MDIO master clock/preamble and PHY polling configuration. This
 * register also gates the table engines: other values wedge ATC/VTCR
 * busy forever. */
#define RT6856_GSW_PHY_POLLING_CFG	0x883e0d01
/* GPC1: TX and RX delay bits, same encoding as the MT7620 GPCx regs. */
#define RT6856_GSW_GPC1_CFG		0xc
/* CPU port PCR: security mode 3, port matrix 0x5f (user ports + self). */
#define RT6856_GSW_PCR_CPU_CFG		0x20df0003
/* CPU port PVC: VLAN function enabled per the vendor value. */
#define RT6856_GSW_PVC_CPU_CFG		0x81000000
/* Flooding masks: the CPU port is bit 6; bits 5 and 7 of the vendor
 * value cover an unused port and a reserved bit. */
#define RT6856_GSW_MFC_CPU_FLOOD	0xe0

#define RT6856_GSW_CPU_PORT		6
#define RT6856_GSW_EPHY_PORTS		5

static void __iomem *rt6856_gsw_base;
static bool rt6856_gsw_done;

/*
 * MDIO transactions that timed out. The EPHY tuning sequence ignores
 * individual errors and reports the total at the end of the bring-up;
 * after that the counter keeps counting but nobody reads it, later
 * callers see the errno directly.
 */
static int rt6856_mdio_failures;

static int rt6856_mdio_wait(void)
{
	u32 val;

	return read_poll_timeout(fe_r32, val, !(val & RT6856_GSW_PIAC_BUSY),
				 10, 100000, false, RT6856_GSW_PIAC);
}

static void rt6856_mdio_start(u32 cmd)
{
	/* Latch the request before raising the busy/start bit. */
	fe_w32(cmd, RT6856_GSW_PIAC);
	fe_w32(cmd | RT6856_GSW_PIAC_BUSY, RT6856_GSW_PIAC);
}

/**
 * rt6856_gsw_mdio_read - read a PHY register through the switch MDIO
 * master
 * @phy: PHY address on the switch MDIO master
 * @reg: register number
 *
 * Returns the 16-bit register value, or a negative errno on timeout.
 */
int rt6856_gsw_mdio_read(int phy, int reg)
{
	u32 cmd;
	int ret;

	ret = rt6856_mdio_wait();
	if (ret)
		goto fail;

	cmd = (phy << 20) | (reg << 25) | RT6856_GSW_PIAC_READ;
	rt6856_mdio_start(cmd);

	ret = rt6856_mdio_wait();
	if (ret)
		goto fail;

	return fe_r32(RT6856_GSW_PIAC) & 0xffff;

fail:
	rt6856_mdio_failures++;
	return ret;
}

/**
 * rt6856_gsw_mdio_write - write a PHY register through the switch MDIO
 * master
 * @phy: PHY address on the switch MDIO master
 * @reg: register number
 * @val: value to write
 *
 * Returns 0, or a negative errno on timeout.
 */
int rt6856_gsw_mdio_write(int phy, int reg, u16 val)
{
	u32 cmd;
	int ret;

	ret = rt6856_mdio_wait();
	if (ret)
		goto fail;

	cmd = (phy << 20) | (reg << 25) | RT6856_GSW_PIAC_WRITE | val;
	rt6856_mdio_start(cmd);

	ret = rt6856_mdio_wait();
	if (ret)
		goto fail;

	return 0;

fail:
	rt6856_mdio_failures++;
	return ret;
}

static void rt6856_gate(struct regmap *gate, u32 off, u32 mask, u32 val)
{
	if (regmap_update_bits(gate, off, mask, val))
		pr_warn("rt6856-gsw: failed to update gate register %#x\n", off);
}

static void rt6856_gsw_clear_mac_table(void)
{
	u32 val;

	/* Address table command: flush all entries. */
	fe_w32(RT6856_GSW_ATC_BUSY | RT6856_GSW_ATC_CMD_FLUSH_ALL,
	       RT6856_GSW_REG(RT6856_GSW_ATC));
	udelay(10);

	if (read_poll_timeout(fe_r32, val, !(val & RT6856_GSW_ATC_BUSY),
			      100, 500000, false,
			      RT6856_GSW_REG(RT6856_GSW_ATC)))
		pr_warn("rt6856-gsw: failed to clear the MAC table\n");
}

/*
 * Switch bring-up: enable the EPHY clocks through the palmbus syscon,
 * configure the port MACs (CPU port forced up, user ports held down
 * while the EPHYs are tuned), apply the internal EPHY tuning sequence
 * and put the CPU port into the flooding masks.
 */
static void rt6856_gsw_hw_init(struct regmap *gate)
{
	u32 val;
	int phyrev;
	int i;

	rt6856_mdio_failures = 0;

	/* Enable the EPHY clocks; the reset bits of 0x834 do not latch
	 * writes (see the gate defines above), so this is the only
	 * effective step. */
	rt6856_gate(gate, RT6856_GATE_CLK_OFF, RT6856_GATE_CLK_MASK,
		    RT6856_GATE_CLK_MASK);

	fe_w32(RT6856_GSW_PHY_POLLING_CFG, RT6856_GSW_PHY_POLLING);
	fe_w32(RT6856_GSW_GPC1_CFG, RT6856_GSW_GPC1);

	/* Hold the five EPHY ports and the unused port 5 down while the
	 * EPHYs are tuned; force the CPU port up. */
	for (i = 0; i < RT6856_GSW_CPU_PORT; i++)
		fe_w32(RT6856_PMCR_LINK_DOWN, RT6856_GSW_REG(RT6856_GSW_PMCR(i)));
	fe_w32(RT6856_PMCR_CPU, RT6856_GSW_REG(RT6856_GSW_PMCR(RT6856_GSW_CPU_PORT)));

	/* Undecoded port 0 block registers; values verbatim. */
	fe_w32(RT6856_GSW_PORT_E0_CFG, RT6856_GSW_REG(RT6856_GSW_PORT_E0));
	val = fe_r32(RT6856_GSW_REG(RT6856_GSW_PORT_F0));
	fe_w32(val & ~RT6856_GSW_PORT_F0_CLEAR, RT6856_GSW_REG(RT6856_GSW_PORT_F0));

	rt6856_gsw_clear_mac_table();

	/* Let the freshly reset EPHY analog section settle before the
	 * tuning sequence. */
	mdelay(10);

	/* Internal EPHY tuning over the switch MDIO master. Pages are
	 * selected on PHY 1, which owns the shared analog and LED
	 * registers; per-port values go to PHYs 0 through 4. Replicate the
	 * sequence verbatim: PHY 1 is written twice on the 0xa000 page and
	 * PHY 4 is not written at all. */
	/* EPHY silicon revision: the tuning differs between revisions. */
	phyrev = rt6856_gsw_mdio_read(0, 0x1f);
	if (phyrev < 0)
		phyrev = 0;

	rt6856_gsw_mdio_write(1, 0x1f, 0x4000);
	rt6856_gsw_mdio_write(1, 0x10, 0xd4cc);
	rt6856_gsw_mdio_write(1, 0x11, 0x7444);
	rt6856_gsw_mdio_write(1, 0x13, 0x112);
	rt6856_gsw_mdio_write(1, 0x15, 0x7160);
	rt6856_gsw_mdio_write(1, 0x16, 0x10cf);
	rt6856_gsw_mdio_write(1, 0x1a, 0x777);

	if (phyrev & 0xf) {
		rt6856_gsw_mdio_write(1, 0x19, 0x212);
		rt6856_gsw_mdio_write(1, 0x1d, 0x4640);
	} else {
		rt6856_gsw_mdio_write(1, 0x19, 0x102);
		rt6856_gsw_mdio_write(1, 0x1d, 0x8641);
	}

	rt6856_gsw_mdio_write(1, 0x1f, 0x2000);
	rt6856_gsw_mdio_write(1, 0x15, 0x655);
	rt6856_gsw_mdio_write(1, 0x16, 0xfd3);
	rt6856_gsw_mdio_write(1, 0x17, 0x3d);
	rt6856_gsw_mdio_write(1, 0x18, 0x96e);
	rt6856_gsw_mdio_write(1, 0x19, 0xfed);
	rt6856_gsw_mdio_write(1, 0x1a, 0xfc4);

	rt6856_gsw_mdio_write(1, 0x1f, 0x1000);
	rt6856_gsw_mdio_write(1, 0x11, 0xe7f8);

	rt6856_gsw_mdio_write(1, 0x1f, 0xa000);
	/* Intentional: PHY 1 twice (0xe0e then 0xf0f), PHY 4 never. */
	rt6856_gsw_mdio_write(1, 0x10, 0xe0e);
	rt6856_gsw_mdio_write(0, 0x10, 0xc0c);
	rt6856_gsw_mdio_write(1, 0x10, 0xf0f);
	rt6856_gsw_mdio_write(2, 0x10, 0x1010);
	rt6856_gsw_mdio_write(3, 0x10, 0x909);

	for (i = 0; i < RT6856_GSW_EPHY_PORTS; i++)
		rt6856_gsw_mdio_write(i, 0x11, 0);

	rt6856_gsw_mdio_write(1, 0x1f, 0x8000);
	/* Turn the ports on: 100M, autoneg, full duplex. BMCR_PDOWN must
	 * stay clear here or the ports never link. */
	for (i = 0; i < RT6856_GSW_EPHY_PORTS; i++)
		rt6856_gsw_mdio_write(i, MII_BMCR,
				      BMCR_SPEED100 | BMCR_ANENABLE |
				      BMCR_FULLDPLX);

	rt6856_gsw_mdio_write(1, 0x1f, 0xb000);
	for (i = 0; i < RT6856_GSW_EPHY_PORTS; i++)
		rt6856_gsw_mdio_write(i, 0x11, 0);

	rt6856_gsw_mdio_write(1, 0x1f, 0x8000);

	if (rt6856_mdio_failures)
		pr_warn("rt6856-gsw: %d EPHY tuning MDIO transactions timed out\n",
			rt6856_mdio_failures);

	/* CPU port VLAN attributes: security mode, admitted frames
	 * filtered on the port VLAN table. */
	fe_w32(RT6856_GSW_PCR_CPU_CFG,
	       RT6856_GSW_REG(RT6856_GSW_PCR(RT6856_GSW_CPU_PORT)));
	fe_w32(RT6856_GSW_PVC_CPU_CFG,
	       RT6856_GSW_REG(RT6856_GSW_PVC(RT6856_GSW_CPU_PORT)));

	/* Flooding masks: include the CPU port (see
	 * RT6856_GSW_MFC_CPU_FLOOD). */
	val = fe_r32(RT6856_GSW_REG(RT6856_GSW_MFC));
	fe_w32(val | RT6856_GSW_MFC_CPU_FLOOD, RT6856_GSW_REG(RT6856_GSW_MFC));

	/* Bit 2 of register 0xc: present in the vendor bring-up sequence,
	 * the field is not decoded. */
	val = fe_r32(RT6856_GSW_REG(0xc));
	fe_w32(val | BIT(2), RT6856_GSW_REG(0xc));

	/* Ack pending switch interrupts and keep them all masked: nothing
	 * handles the GSW interrupt on this SoC. */
	fe_w32(0xffffffff, RT6856_GSW_ISR);
	fe_w32(0, RT6856_GSW_IMR);

	/* Bring the five EPHY ports up: hardware-managed MAC, following
	 * the PHY link state. */
	for (i = 0; i < RT6856_GSW_EPHY_PORTS; i++)
		fe_w32(RT6856_PMCR_EPHY_PORT, RT6856_GSW_REG(RT6856_GSW_PMCR(i)));
}

/**
 * rt6856_gsw_init - bring up the internal GSW and register its swconfig
 * frontend, if the board requests it
 * @dev: the ethernet platform device
 *
 * Boards without the "ralink,gsw-syscon" phandle (an external switch,
 * like the Keenetic Giga II) are skipped. Runs at frame-engine probe
 * time, before userspace loads the swconfig switch configuration.
 *
 * The done flag makes this run exactly once per boot: re-running the
 * sequence (a failed probe retried, a driver rebind) would drop the
 * EPHY links for nothing, and the MT7530 swconfig frontend has no
 * unregister path, so a second registration would accumulate.
 */
int rt6856_gsw_init(struct device *dev)
{
	struct regmap *gate;

	if (rt6856_gsw_done)
		return 0;

	gate = syscon_regmap_lookup_by_phandle_optional(dev->of_node,
							"ralink,gsw-syscon");
	if (!gate)
		return 0;

	rt6856_gsw_base = fe_get_base() + RT6856_GSW_OFFSET;

	rt6856_gsw_hw_init(gate);
	rt6856_gsw_done = true;

	/*
	 * The switch block is register-compatible with the MT7620 GSW:
	 * reuse the MT7530 swconfig frontend for the VLAN table and port
	 * configuration (CPU port 6, seven ports, sixteen VLANs). If the
	 * frontend fails to register, the hardware stays up as brought up
	 * here: a flat, untagged network.
	 */
	return mt7530_probe(dev, rt6856_gsw_base, NULL, 1);
}

/**
 * rt6856_gsw_has_carrier - report the CPU port link status for eth0
 *
 * The CPU port is forced up during the bring-up, so a brought-up
 * internal GSW always reports carrier; boards without the internal
 * switch report none (their eth0 then runs with NO-CARRIER, as on the
 * Keenetic Giga II).
 */
int rt6856_gsw_has_carrier(void)
{
	if (!rt6856_gsw_base)
		return 0;

	return fe_r32(RT6856_GSW_REG(RT6856_GSW_PMSR(RT6856_GSW_CPU_PORT))) & 1;
}

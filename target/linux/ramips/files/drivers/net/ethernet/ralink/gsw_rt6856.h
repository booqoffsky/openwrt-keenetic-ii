// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ralink RT6856 integrated Ethernet switch bring-up.
 *
 * The frame-engine driver calls into this from its SoC hook when the
 * board asks for the internal switch through a "ralink,gsw-syscon"
 * phandle on the ethernet node.
 */

#ifndef _GSW_RT6856_H__
#define _GSW_RT6856_H__

#include <linux/types.h>

struct device;

int rt6856_gsw_init(struct device *dev);
int rt6856_gsw_has_carrier(void);

/*
 * Low-level access to the switch MDIO master (PIAC). Returns a negative
 * errno on timeout, the register value on read. Also used by the SoC
 * code for its mii_bus callbacks.
 */
int rt6856_gsw_mdio_read(int phy, int reg);
int rt6856_gsw_mdio_write(int phy, int reg, u16 val);

#endif

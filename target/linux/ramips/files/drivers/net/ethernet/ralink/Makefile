#
# Makefile for the Ralink SoCs built-in ethernet macs
#

ralink-eth-y					+= mtk_eth_soc.o ethtool.o

ralink-eth-$(CONFIG_NET_RALINK_MDIO)		+= mdio.o
ralink-eth-$(CONFIG_NET_RALINK_MDIO_RT2880)	+= mdio_rt2880.o
ralink-eth-$(CONFIG_NET_RALINK_MDIO_MT7620)	+= mdio_mt7620.o

ralink-eth-$(CONFIG_NET_RALINK_RT2880)	+= soc_rt2880.o
ralink-eth-$(CONFIG_NET_RALINK_RT3050)	+= soc_rt3050.o
ralink-eth-$(CONFIG_NET_RALINK_RT3883)	+= soc_rt3883.o
ralink-eth-$(CONFIG_NET_RALINK_MT7620)	+= soc_mt7620.o
ralink-eth-$(CONFIG_NET_RALINK_RT6855)	+= soc_rt6855.o gsw_rt6856.o
# mt7530.o doubles as the swconfig frontend of the RT6856 switch, but the
# MT7620 GSW build links the same object separately below — take it here
# only when that config is off, or it ends up in vmlinux twice.
ifndef CONFIG_NET_RALINK_GSW_MT7620
ralink-eth-$(CONFIG_NET_RALINK_RT6855)	+= mt7530.o
endif

obj-$(CONFIG_NET_RALINK_ESW_RT3050)		+= esw_rt3050.o
obj-$(CONFIG_NET_RALINK_GSW_MT7620)		+= gsw_mt7620.o mt7530.o
obj-$(CONFIG_NET_DSA_MT7620)			+= gsw_mt7620_dsa.o
obj-$(CONFIG_NET_RALINK_SOC)			+= ralink-eth.o

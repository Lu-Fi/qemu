system_ss.add(when: 'CONFIG_DP8393X', if_true: files('dp8393x.c'))
system_ss.add(when: 'CONFIG_XEN_BUS', if_true: files('xen_nic.c'))
system_ss.add(when: 'CONFIG_NE2000_COMMON', if_true: files('ne2000.c'))

# PCI network cards
system_ss.add(when: 'CONFIG_NE2000_PCI', if_true: files('ne2000-pci.c'))
system_ss.add(when: 'CONFIG_EEPRO100_PCI', if_true: files('eepro100.c'))
system_ss.add(when: 'CONFIG_PCNET_PCI', if_true: files('pcnet-pci.c'))
system_ss.add(when: 'CONFIG_PCNET_COMMON', if_true: files('pcnet.c'))
system_ss.add(when: 'CONFIG_E1000_PCI', if_true: files('e1000.c', 'e1000x_common.c'))
system_ss.add(when: 'CONFIG_E1000E_PCI_EXPRESS', if_true: files('net_tx_pkt.c', 'net_rx_pkt.c'))
system_ss.add(when: 'CONFIG_E1000E_PCI_EXPRESS', if_true: files('e1000e.c', 'e1000e_core.c', 'e1000x_common.c'))
system_ss.add(when: 'CONFIG_IGB_PCI_EXPRESS', if_true: files('net_tx_pkt.c', 'net_rx_pkt.c'))
system_ss.add(when: 'CONFIG_IGB_PCI_EXPRESS', if_true: files('igb.c', 'igbvf.c', 'igb_core.c'))
system_ss.add(when: 'CONFIG_RTL8139_PCI', if_true: files('rtl8139.c'))
system_ss.add(when: 'CONFIG_TULIP', if_true: files('tulip.c'))
system_ss.add(when: 'CONFIG_VMXNET3_PCI', if_true: files('net_tx_pkt.c', 'net_rx_pkt.c'))
system_ss.add(when: 'CONFIG_VMXNET3_PCI', if_true: files('vmxnet3.c'))

system_ss.add(when: 'CONFIG_SMC91C111', if_true: files('smc91c111.c'))
system_ss.add(when: 'CONFIG_LAN9118', if_true: files('lan9118.c'))
system_ss.add(when: 'CONFIG_LAN9118_PHY', if_true: files('lan9118_phy.c'))
system_ss.add(when: 'CONFIG_NE2000_ISA', if_true: files('ne2000-isa.c'))
system_ss.add(when: 'CONFIG_OPENCORES_ETH', if_true: files('opencores_eth.c'))
system_ss.add(when: 'CONFIG_XILINX_AXI', if_true: files('xilinx_axienet.c'))
system_ss.add(when: 'CONFIG_ALLWINNER_EMAC', if_true: files('allwinner_emac.c'))
system_ss.add(when: 'CONFIG_ALLWINNER_SUN8I_EMAC', if_true: files('allwinner-sun8i-emac.c'))
system_ss.add(when: 'CONFIG_IMX_FEC', if_true: files('imx_fec.c'))
system_ss.add(when: 'CONFIG_MSF2', if_true: files('msf2-emac.c'))

system_ss.add(when: 'CONFIG_CADENCE', if_true: files('cadence_gem.c'))
system_ss.add(when: 'CONFIG_STELLARIS_ENET', if_true: files('stellaris_enet.c'))
system_ss.add(when: 'CONFIG_LANCE', if_true: files('lance.c'))
system_ss.add(when: 'CONFIG_LASI_82596', if_true: files('lasi_i82596.c'))
system_ss.add(when: 'CONFIG_I82596_COMMON', if_true: files('i82596.c'))
system_ss.add(when: 'CONFIG_SUNHME', if_true: files('sunhme.c'))
system_ss.add(when: 'CONFIG_FTGMAC100', if_true: files('ftgmac100.c'))
system_ss.add(when: 'CONFIG_SUNGEM', if_true: files('sungem.c'))
system_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx_emc.c', 'npcm_gmac.c'))
system_ss.add(when: 'CONFIG_NPCM8XX', if_true: files('npcm_pcs.c'))

system_ss.add(when: 'CONFIG_COLDFIRE', if_true: files('mcf_fec.c'))
specific_ss.add(when: 'CONFIG_PSERIES', if_true: files('spapr_llan.c'))
system_ss.add(when: 'CONFIG_XILINX_ETHLITE', if_true: files('xilinx_ethlite.c'))

system_ss.add(when: 'CONFIG_VIRTIO_NET', if_true: files('net_rx_pkt.c'))
system_ss.add(when: 'CONFIG_VIRTIO_NET', if_true: files('virtio-net.c'))

if have_vhost_net
  system_ss.add(when: 'CONFIG_VIRTIO_NET', if_true: files('vhost_net.c'))
endif
stub_ss.add(files('vhost_net-stub.c'))

system_ss.add(when: 'CONFIG_ETSEC', if_true: files(
  'fsl_etsec/etsec.c',
  'fsl_etsec/miim.c',
  'fsl_etsec/registers.c',
  'fsl_etsec/rings.c',
))

system_ss.add(when: 'CONFIG_ROCKER', if_true: files(
  'rocker/rocker.c',
  'rocker/rocker_desc.c',
  'rocker/rocker_fp.c',
  'rocker/rocker_of_dpa.c',
  'rocker/rocker_world.c',
))
stub_ss.add(files('rocker/rocker-stubs.c'))
if have_hmp
  system_ss.add(files('rocker/rocker-hmp-cmds.c'))
endif

system_ss.add(when: 'CONFIG_INGENIC_GMAC', if_true: files('ingenic-gmac.c'))
system_ss.add(when: 'CONFIG_AIC_WIFI', if_true: files('aic-wifi.c', 'aic-sdio.c'))
system_ss.add(when: ['CONFIG_AIC_WIFI', 'CONFIG_USB'], if_true: files('aic-usb.c'))
system_ss.add(when: 'CONFIG_ATBM_WIFI', if_true: files('atbm-wifi.c', 'atbm-sdio.c'))
system_ss.add(when: ['CONFIG_ATBM_WIFI', 'CONFIG_USB'], if_true: files('atbm-usb.c'))
system_ss.add(when: 'CONFIG_BCM_WIFI', if_true: files('bcm-wifi.c', 'bcm-sdio.c'))
system_ss.add(when: 'CONFIG_HI3881_WIFI', if_true: files('hi3881.c'))
system_ss.add(when: 'CONFIG_RTL_WIFI', if_true: files('rtl-wifi.c', 'rtl-sdio.c'))
system_ss.add(when: ['CONFIG_RTL_WIFI', 'CONFIG_USB'], if_true: files('rtl-usb.c'))
system_ss.add(when: ['CONFIG_MT7601_WIFI', 'CONFIG_USB'], if_true: files('mt7601u.c'))
system_ss.add(when: ['CONFIG_TXW901U_WIFI', 'CONFIG_USB'], if_true: files('txw901u.c'))
system_ss.add(when: ['CONFIG_WQ9001_WIFI', 'CONFIG_USB'], if_true: files('wq9001.c'))
system_ss.add(when: ['CONFIG_WS73_WIFI', 'CONFIG_USB'], if_true: files('ws73.c'))
system_ss.add(when: 'CONFIG_SSV_WIFI', if_true: files('ssv-wifi.c', 'ssv-fmac.c',
                                                     'ssv-sdio.c'))
system_ss.add(when: ['CONFIG_SSV_WIFI', 'CONFIG_USB'], if_true: files('ssv-usb.c'))
system_ss.add(when: 'CONFIG_WIFI_AP', if_true: files('wifi-ap.c'))
system_ss.add(when: 'CONFIG_WIFI_CLIENT', if_true: files('wifi-client.c'))
system_ss.add(when: 'CONFIG_WIFI_CCMP', if_true: files('wifi-ccmp.c'))
system_ss.add(when: 'CONFIG_WIFI_RSN', if_true: files('wifi-rsn.c'))
system_ss.add(when: 'CONFIG_INGENIC_A1_XGMAC', if_true: files('ingenic-a1-xgmac.c'))

subdir('can')

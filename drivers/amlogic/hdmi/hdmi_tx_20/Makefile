obj-$(CONFIG_AML_HDMI_TX_20)		+= hdmitx20.o

hdmitx20-objs := hdmi_tx_main.o hdmi_tx_video.o hdmi_tx_audio.o hdmi_tx_edid.o \
		 hdmi_tx_audio.o hdmi_tx_hdcp.o hdmi_tx_scdc.o

ifdef CONFIG_AML_HDMI_TX_NEW_CEC_DRIVER
  hdmitx20-objs += amlogic_cec.o
else
  hdmitx20-objs += hdmi_cec_key.o hdmi_tx_cec.o
endif
obj-$(CONFIG_AML_HDMI_TX_20) += hw/

#EXTRA_CFLAGS += -O2

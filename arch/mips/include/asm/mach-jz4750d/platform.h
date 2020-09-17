#ifndef __JZ4750D_PLATFORM_H__
#define __JZ4750D_PLATFORM_H__

/* msc */
#define CARD_INSERTED 1
#define CARD_REMOVED 0

struct jz_mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	unsigned char status_irq;
	unsigned char support_sdio;
	unsigned char bus_width;
	unsigned int max_bus_width;
	unsigned int detect_pin;

	unsigned char msc_irq;
	unsigned char dma_rxid;
	unsigned char dma_txid;

	void *driver_data;

	void (*init) (struct device *);
	void (*power_on) (struct device *);
	void (*power_off) (struct device *);
	void (*cpm_start) (struct device *);
	unsigned int (*status) (struct device *);
	unsigned int (*write_protect) (struct device *);
	void (*plug_change) (int);
};

#endif /* __JZ4750D_PLATFORM_H__ */

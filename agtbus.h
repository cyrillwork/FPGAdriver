#ifndef DGT_AGTBUS_H
#define DGT_AGTBUS_H

#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/version.h>

#define AXI4_INTERRUPT_REG  0x00000000 // 32 input Interrupt Controller

#define AXI4_UART0_REG      0x00020000 // UART #0
#define AXI4_UART1_REG      0x00022000 // UART #1
#define AXI4_UART2_REG      0x00024000 // UART #2
#define AXI4_UART3_REG      0x00026000 // UART #3
#define AXI4_UART4_REG      0x00028000 // UART #4
#define AXI4_UART5_REG      0x0002A000 // UART #5
#define AXI4_UART6_REG      0x0002C000 // UART #6
#define AXI4_UART7_REG      0x0002E000 // UART #7
//#define AXI4_UART8_REG      0x00030000 // UART #8. Optional
#define AXI4_UART9_REG      0x00032000 // UART #9. Logging Processor

#define AXI4_FPGA_GMIO0_REG 0x00100000 // FPGA GMIO
#define AXI4_FPGA_GMIOI_REG 0x00100004 // FPGA GMIO_INPUT register

#define AXI4_FPGA_GMIO1_REG 0x00200000 // FPGA GMIO. Reserved
#define AXI4_FPGA_GMIO2_REG 0x00300000 // FPGA GMIO. Reserved
#define AXI4_FPGA_GMIO3_REG 0x00400000 // FPGA GMIO. Reserved

#define AXI4_MRAM_BANK0_REG 0x04000000 // MRAM Bank #0
#define AXI4_MRAM_BANK1_REG 0x04400000 // MRAM Bank #1
#define AXI4_MRAM_BANKC_REG 0x04800000 // MRAM Bank #0 and MRAM bank #1 compare region

#define AXI4_I2C_REG 0x00130000 // work with I2C interface

#define AXI4_CLEARINT_TIMER 0x24

typedef struct dgt_xpdev_t dgt_xpdev_t;

struct dgt_xpdev_t 
{
  struct pci_dev* pcidev;
  unsigned long base_start;
  unsigned long base_len;
  void __iomem* iobase;
  int irq;
  spinlock_t lock;
};

static inline unsigned char dgt_xpdev_readb(struct dgt_xpdev_t* xpdev, int reg) { return readb(xpdev->iobase + reg); }
static inline u32 dgt_xpdev_readl(struct dgt_xpdev_t* xpdev, int reg) { return readl(xpdev->iobase + reg); }
static inline u64 dgt_xpdev_readq(struct dgt_xpdev_t* xpdev, int reg) { return readq(xpdev->iobase + reg); }

static inline void dgt_xpdev_writeb(struct dgt_xpdev_t* xpdev, int reg, unsigned char val) { writeb(val, xpdev->iobase + reg); }
static inline void dgt_xpdev_writel(struct dgt_xpdev_t* xpdev, int reg, u32 val) { writel(val, xpdev->iobase + reg); }
static inline void dgt_xpdev_writeq(struct dgt_xpdev_t* xpdev, int reg, u64 val) { writeq(val, xpdev->iobase + reg); }

#endif /* DGT_AGTBUS_H */


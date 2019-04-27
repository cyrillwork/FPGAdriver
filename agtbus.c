#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci_ids.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <linux/serial.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>

#include <uapi/linux/serial_reg.h>


#include "agtbus.h"
#include "agt_version.h"


#define SUCCESS 0
#define PREFIX "[agtbus]: "
#define PCI_DEVICE_ID_XILINX 0x7013

#define DGT_BUF_SIZE (size_t)(sizeof(struct dgt_xpdev_t))

struct dgt_xpdev_t drv_access;
struct dgt_xpdev_t *drv_access2 = NULL;

static int isIRQ = 0;


unsigned int AXI_UART_REG[] = 
{
	AXI4_UART0_REG,
	AXI4_UART1_REG,
	AXI4_UART2_REG,
	AXI4_UART3_REG,
	AXI4_UART4_REG,
	AXI4_UART5_REG,
	AXI4_UART6_REG,
	AXI4_UART7_REG,
	AXI4_UART9_REG
};

#define COUNT_METERS		8

typedef struct
{
    int array_meters[COUNT_METERS];
    int array_crc[COUNT_METERS];
    unsigned int turn;
    unsigned int update;
    u64 mask;
} meters_data;

meters_data g_meters_data;



#include "nvram.c"

#include "uart_agt.c"

#include "buttons.c"


/*************** PCIe Main functions  ****************************/


// IRQ handler function
static irqreturn_t dgt_xpdev_irq(int irq, void* data)
{
	return IRQ_HANDLED;
}

static int dgt_xpdev_create(struct pci_dev* pcidev, struct dgt_xpdev_t** result)
{
	struct dgt_xpdev_t* xpdev;

	xpdev = kmalloc(sizeof(dgt_xpdev_t), GFP_KERNEL);
	if (xpdev == NULL)
		return -ENOMEM;

	memset(xpdev, 0, sizeof(dgt_xpdev_t));
	
	spin_lock_init(&xpdev->lock);
	xpdev->pcidev = pcidev;
	pci_set_drvdata(pcidev, xpdev);

	*result = xpdev;
	return SUCCESS;
}

static int agtbus_probe(struct pci_dev* dev, const struct pci_device_id* id);
static void agtbus_remove(struct pci_dev* dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
DEFINE_PCI_DEVICE_TABLE(agtbus_id_table)=
#else
static const struct pci_device_id agtbus_id_table[] =
#endif
{
  { PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX) },
  { 0, 0 }
};


static struct pci_driver agtbus_pci_driver = 
{
  .name = KBUILD_MODNAME,
  .id_table = agtbus_id_table,
  .probe    = agtbus_probe,
  .remove   = agtbus_remove
};


static int agtbus_probe(struct pci_dev* pcidev, const struct pci_device_id* id)
{
	int bar;
	int res;
	u16 vendor, device;
	unsigned char myirq = 0;
	unsigned char irq_pin = 0;
	struct dgt_xpdev_t* xpdev;

	printk(KERN_DEBUG PREFIX "!!!! agtbus_probe\n");
	res = pci_enable_device(pcidev);

	if(res)
	{
		printk(KERN_DEBUG PREFIX "Couldn't enable the device\n");
		return res;
	}

	pci_read_config_word(pcidev, PCI_VENDOR_ID, &vendor);
	pci_read_config_word(pcidev, PCI_DEVICE_ID, &device);
	
	res = pci_read_config_byte(pcidev, PCI_INTERRUPT_LINE, &myirq);
	if(res)
	{
		printk(PREFIX "Error read IRQ from FPGA\n");
		return res;
	}
	printk(PREFIX "Device vendor: 0x%X device_id: 0x%X\n", vendor, device);

	printk(PREFIX "IRQ: irq: %u\n", myirq);
	
	
	pci_read_config_byte(pcidev, PCI_INTERRUPT_PIN, &irq_pin);
	if(res)
	{
		printk(PREFIX "Error read IRQ PIN\n");
		return res;
	}
	printk(PREFIX "IRQ PIN: %u\n", irq_pin);


	res = dgt_xpdev_create(pcidev, &xpdev);
	if (res != SUCCESS)
	{
		printk(KERN_ERR PREFIX "Coudln't create xpdev.\n");
		return res;
	}

	xpdev->base_start = pci_resource_start(pcidev, 0);
	xpdev->base_len   = pci_resource_len(pcidev, 0);

	bar = pci_select_bars(pcidev, IORESOURCE_MEM);
	printk(PREFIX "Device available MEM BAR are 0x%x\n", bar);
	
	res = pci_enable_device_mem(pcidev);
	if(res != SUCCESS)
	{
		printk(KERN_ERR PREFIX "Failed to enable device memory, err: %i\n", res);
		return res;
	}

	res = pci_request_region(pcidev, bar, "agtbus");
  
	if (res != SUCCESS) 
	{
		printk(KERN_ERR PREFIX "Coudln't pci_request_region.\n");
		return res;
	}

	xpdev->iobase = ioremap(xpdev->base_start, xpdev->base_len);
	
	if (!xpdev->iobase)
	{
		printk(KERN_ERR PREFIX "Coudln't map the iobase in the system.\n");
		
		//release_device(pcidev);
		pci_release_region(pcidev, pci_select_bars(pcidev, IORESOURCE_MEM));
		pci_disable_device(pcidev);
		
		return -ENOMEM;
	}
	
	
	{
		if(isIRQ)
		{
			int res;
			res = request_irq(pcidev->irq, dgt_xpdev_irq, IRQF_SHARED, "agtbus", xpdev);
			if (res != SUCCESS)
			{
				printk(KERN_ERR PREFIX "Unable to request IRQ %d\n", pcidev->irq);
				return res;
			}
		}
	
		printk(KERN_ERR PREFIX "Set handler to IRQ %d\n", pcidev->irq);
		
		xpdev->irq = pcidev->irq;
	}

	// set MRAM enable
	{
		dgt_xpdev_writeb(xpdev, AXI4_FPGA_GMIO0_REG + 16,  1);
	}

	{
		dgt_xpdev_writeb(xpdev, AXI4_FPGA_GMIO0_REG + 13,  1);
	}

	{
		dgt_xpdev_writeb(xpdev, AXI4_FPGA_GMIO0_REG + 14,  1);
	}



    {
	    // mask interrupt
	    unsigned int int_mask = 0x02FF0000;
	    dgt_xpdev_writel(xpdev, AXI4_INTERRUPT_REG + 0x20,  int_mask);
    }

	// set timer
	
	{
		dgt_xpdev_writeb(xpdev, AXI4_FPGA_GMIO0_REG + 15,  0);
		
		udelay(1000);
		
		dgt_xpdev_writeb(xpdev, AXI4_FPGA_GMIO0_REG + 15,  1);
	}
	
    { 	//IREDR
	    unsigned int int_mask = 0x00000027;
	    dgt_xpdev_writel(xpdev, AXI4_INTERRUPT_REG + 0x24,  int_mask);
    }

	drv_access = *xpdev;
	drv_access2 = xpdev;

	//create NVRAM device
	create_mramdev();
	
	//create button device
	create_but();

	//create UART device
	create_uart();

	return SUCCESS;
}


static void agtbus_remove(struct pci_dev* dev)
{
	struct dgt_xpdev_t* xpdev = pci_get_drvdata(dev);
	printk(KERN_DEBUG PREFIX "!!!! agtbus_remove\n");

	if(xpdev)
	{
		
		if(isIRQ)
		{
			if (xpdev->irq > 0)
			{
				free_irq(xpdev->irq, xpdev);
			}
		}
		
		
		if (xpdev->iobase != NULL)
		{
			iounmap(xpdev->iobase);
		}

		kfree(xpdev);
	}
	
	pci_disable_device(dev);
}

static int __init agtbus_init(void)
{
	int ret;

	printk(KERN_DEBUG PREFIX "AGT_MODULE_VERSION %s\n", AGT__MODULE_VERSION);

	ret = pci_register_driver(&agtbus_pci_driver);
  
	if(ret != SUCCESS) 
	{
		printk(KERN_ERR PREFIX "Couldn't register the pcidev\n");
		return ret;
	}

	printk(KERN_DEBUG PREFIX "Enable the device\n");
	return SUCCESS;
}

static void __exit agtbus_exit(void)
{
	printk(KERN_DEBUG PREFIX "Exit\n");

	// remove NVRAM device
	remove_mramdev();

	// remove buttons device
	remove_but();
	
	// remove UART device
	remove_uart();

	pci_unregister_driver(&agtbus_pci_driver);
}


module_init(agtbus_init);
module_exit(agtbus_exit);

MODULE_DESCRIPTION("Xilinx AGT bus PCIe driver");
MODULE_AUTHOR("cyrillwork");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");
MODULE_ALIAS("agtbus_pcie");

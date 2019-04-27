/*************** Buttons back-plane I/O  ****************************/
#include <linux/timer.h>
#include <linux/delay.h>

//timer for read changes from agt_spi1
struct timer_list but_timer;
//timer for writes meters changes
struct timer_list meter_timer;

#define BUT_NAME "agt_spi"
#define BUT_NUMB 2

#define CSUM_ADDRESS		0x3ff100
#define TURN_ADDRESS		0x3ff200
#define MASK_ADDRESS		0x3ff300

// 4*1024*1023
#define METERS_ADDRESS		0x3ff000


static int rs_dev_major = 0;

static int is_set_clear_meters	= 0;
static int is_work_meters		= 0;

static struct class *but_class = NULL;

struct rs_device_private
{
	uint8_t chnum;
	struct dgt_xpdev_t *drv;
};

struct rs_device_data
{
	struct device *rsdev;
	struct cdev cdev;
};

static struct rs_device_data but_dev_data[BUT_NUMB];

static int but_open_spi0(struct inode *inode, struct file *file);
static int but_open_spi1(struct inode *inode, struct file *file);

static int but_release_spi0(struct inode *inode, struct file *file);
static int but_release_spi1(struct inode *inode, struct file *file);

static ssize_t but_read_spi1(struct file *file, char __user *buf, size_t count, loff_t *offset);
static ssize_t but_write_spi0(struct file *file, const char __user *buf, size_t count, loff_t *offset);
static long but_ioctl_spi0(struct file *file, unsigned int call, unsigned long size);
static long but_ioctl_spi1(struct file *file, unsigned int call, unsigned long size);

static loff_t but_lseek (struct file *file, loff_t offset, int orig);

u64 output_reg64 = 0;
u64 output_reg64_user = 0;

static struct file_operations but_dev_foops[] = 
{
{ // SPI0
    .owner          = THIS_MODULE,
    .read           = NULL,
    .write          = but_write_spi0,
    .open           = but_open_spi0,
    .llseek         = but_lseek,
    .unlocked_ioctl = but_ioctl_spi0,
    .release        = but_release_spi0,
    .mmap           = NULL,

},
{ //SPI1
    .owner          = THIS_MODULE,
    .read           = but_read_spi1,
    .write          = NULL,
    .open           = but_open_spi1,
    .llseek         = but_lseek,
    .unlocked_ioctl = but_ioctl_spi1,
    .release        = but_release_spi1,
    .mmap           = NULL,

}
};

/* Delay sleep routine loop 1ms*/
#define BUT_DELAY (unsigned long)(HZ*0.001)

/* Delay sleep routine loop 1ms*/
#define METER_DELAY (unsigned long)(HZ*0.025)

/* Register Input SPI1 56bits have means*/
static unsigned long long _reg64 = 0;

static DEFINE_MUTEX(_reg64_mutex);

static DEFINE_MUTEX(meter_mutex);

static unsigned int checksum(unsigned int var)
{
    int i;
    unsigned int res = 0;
    unsigned char *ptr1 = (unsigned char *)(&var);

    for(i = 0; i < sizeof(unsigned int); i++)
    {
        res ^= *ptr1++;
    }

    return res;
}

static void need_write_reg64(u64 reg)
{
	writeq(reg, drv_access.iobase + 0x00110000);
	udelay(6); //wait 6 uS
	//Wait intil bit #1 reg status == 0
	while(1)
	{
		unsigned int status_reg = readl(drv_access.iobase + AXI4_FPGA_GMIO0_REG);
		if((status_reg & 1) == 0)
		{
			break;
		}
		udelay(25);
	}
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
static void timer_func_meter(unsigned long func_param)
#else
static void timer_func_meter(struct timer_list *unused)
#endif
{
	int need_write = 0;
	int iii;
	u64 mask1 = 0;
	meters_data l_meters_data;

	if(is_set_clear_meters == 1)
	{
		unsigned int null1 = 0x0;

		memset(&g_meters_data, 0, sizeof(meters_data));

		for(iii = 0; iii < COUNT_METERS; iii++)
		{

			writel(null1, drv_access.iobase + AXI4_MRAM_BANK0_REG + METERS_ADDRESS + 4*iii);
			writel(null1, drv_access.iobase + AXI4_MRAM_BANK1_REG + METERS_ADDRESS + 4*iii);

			writel(null1, drv_access.iobase + AXI4_MRAM_BANK0_REG + CSUM_ADDRESS + 4*iii);
			writel(null1, drv_access.iobase + AXI4_MRAM_BANK1_REG + CSUM_ADDRESS + 4*iii);
		}

		writel(null1, drv_access.iobase + AXI4_MRAM_BANK0_REG + TURN_ADDRESS);
		writel(null1, drv_access.iobase + AXI4_MRAM_BANK1_REG + TURN_ADDRESS);

		writeq(null1, drv_access.iobase + AXI4_MRAM_BANK0_REG + MASK_ADDRESS);
		writeq(null1, drv_access.iobase + AXI4_MRAM_BANK1_REG + MASK_ADDRESS);

		is_set_clear_meters = 0;
	}


	//check if need write new state lamps
	if(output_reg64_user != 0)
	{
		int j;
		u64 mask2 = 0;
		u64 res1  = output_reg64;
		u64 output_reg64_temp = output_reg64_user;

		need_write = 1;

		output_reg64_user = 0;

		for(j = 8; j < 39; ++j)
		{
			if(output_reg64_temp & (1 << j))
			{
				mask2 |= (1 << j);
			}
		}

		if(!(res1 & mask2))
		{
			res1 |= mask2;
		}
		else
		{
			res1 &= ~mask2;
		}

		output_reg64 = res1;
	}

	//printk(KERN_INFO "!!!!!! timer_func_meter output_reg64=%lld\n", output_reg64);

	if(is_work_meters == 1)
	{
		mutex_lock(&meter_mutex);
		memcpy( &l_meters_data, &g_meters_data, sizeof(meters_data) );
		mutex_unlock(&meter_mutex);

		if(l_meters_data.turn)
		{ //if data.turn we need to turn meters one more with mask
			u64 res1  = output_reg64;
			res1 |= l_meters_data.mask;
			output_reg64 = res1;
			
			//printk(KERN_INFO "!!!!!! turn res1=%lld mask=%lld\n", res1, l_meters_data.mask);

			need_write_reg64(res1);

			mutex_lock(&meter_mutex);
			g_meters_data.mask   = 0;
			g_meters_data.turn   = 0;
			g_meters_data.update = 2;
			mutex_unlock(&meter_mutex);
		}
		else
		{
			for(iii = 0; iii < COUNT_METERS; iii++)
			{
				unsigned int get_last = l_meters_data.array_meters[iii]; 

				if(get_last != 0)
				{ //check if exist some meters number
					mask1 |= 1 << iii;
					--get_last;
					l_meters_data.array_meters[iii] = get_last;
					//printk(KERN_INFO "!!!!!! work meter=%d mask=%lld\n", get_last, mask1);
				}
			}

			if(mask1 != 0)
			{ // need write data for turn meters
				u64 res1 = output_reg64;
				res1		&= ~mask1;
				output_reg64 = res1;

				need_write_reg64(res1);

				//calc crc sum of meters
				for(iii = 0; iii < COUNT_METERS; iii++)
				{
					unsigned int get_meter = l_meters_data.array_meters[iii];
					unsigned int sum1 =  checksum(get_meter);

					l_meters_data.array_crc[iii] = sum1;
				}

				l_meters_data.update	= 1;
				l_meters_data.turn		= 1;
				l_meters_data.mask		= mask1;

				mutex_lock(&meter_mutex);
				memcpy( &g_meters_data, &l_meters_data, sizeof(meters_data) );
				mutex_unlock(&meter_mutex);
			}
			else
			if(need_write == 1)
			{
				need_write_reg64(output_reg64);
			}
		}

		if(g_meters_data.update == 1)
		{
			//printk(KERN_INFO "!1 update meter1=%d\n", g_meters_data.array_meters[0]);

			for(iii = 0; iii < COUNT_METERS/2; iii++)
			{
				u64 reg1 = *((u64 *)g_meters_data.array_meters + iii);
				u64 reg2 = *((u64 *)g_meters_data.array_crc + iii);

				writeq(reg1, drv_access.iobase + AXI4_MRAM_BANK0_REG + METERS_ADDRESS + 8*iii);
				writeq(reg1, drv_access.iobase + AXI4_MRAM_BANK1_REG + METERS_ADDRESS + 8*iii);

				writeq(reg2, drv_access.iobase + AXI4_MRAM_BANK0_REG + CSUM_ADDRESS + 8*iii);
				writeq(reg2, drv_access.iobase + AXI4_MRAM_BANK1_REG + CSUM_ADDRESS + 8*iii);
			}

			writel(g_meters_data.turn, drv_access.iobase + AXI4_MRAM_BANK0_REG + TURN_ADDRESS);
			writel(g_meters_data.turn, drv_access.iobase + AXI4_MRAM_BANK1_REG + TURN_ADDRESS);

			writeq(g_meters_data.mask, drv_access.iobase + AXI4_MRAM_BANK0_REG + MASK_ADDRESS);
			writeq(g_meters_data.mask, drv_access.iobase + AXI4_MRAM_BANK1_REG + MASK_ADDRESS);

			g_meters_data.update = 0;
		}
		else
		if(g_meters_data.update == 2)
		{
			//printk(KERN_INFO "!2 update meter1=%d\n", g_meters_data.array_meters[0]);

			writel(g_meters_data.turn, drv_access.iobase + AXI4_MRAM_BANK0_REG + TURN_ADDRESS);
			writel(g_meters_data.turn, drv_access.iobase + AXI4_MRAM_BANK1_REG + TURN_ADDRESS);

			writeq(g_meters_data.mask, drv_access.iobase + AXI4_MRAM_BANK0_REG + MASK_ADDRESS);
			writeq(g_meters_data.mask, drv_access.iobase + AXI4_MRAM_BANK1_REG + MASK_ADDRESS);

			g_meters_data.update = 0;
		}
	}
	else
	if(need_write == 1)
	{
		need_write_reg64(output_reg64);
	}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
	init_timer(&meter_timer);
	meter_timer.expires = jiffies + METER_DELAY;
	meter_timer.data = func_param;
	meter_timer.function = timer_func_meter;
	add_timer(&meter_timer);
#else
    mod_timer(&meter_timer, jiffies + METER_DELAY);
#endif

}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
static void timer_func_buttons(unsigned long func_param)
#else
static void timer_func_buttons(struct timer_list *unused)
#endif
{
	//Trigger FPGA to init shift data
	{
		writeq(0xffffffffffffffff, drv_access.iobase + 0x00120000);
		udelay(6); //wait 6 uS
	}
	
	//Wait intil bit #1 reg status == 0
	while(1)
	{
		unsigned int status_reg = readl(drv_access.iobase + 0x00100000);
		if((status_reg & 2) == 0)
		{
			break;
		}
		//printk(KERN_INFO "!!!!!! stat_reg\n");
		udelay(100);
	}

	//Read input data from FPGA and print if changed
	{
		unsigned long long reg64 = readq(drv_access.iobase + 0x00120008);

		if((_reg64 != reg64))
		{
			printk(KERN_INFO "!!!!!! %llx\n", reg64);

			mutex_lock(&_reg64_mutex);

			_reg64 = reg64;
			
			mutex_unlock(&_reg64_mutex);
		}
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
	init_timer(&but_timer);
	but_timer.expires = jiffies + BUT_DELAY;
	but_timer.data = func_param;
	but_timer.function = timer_func_buttons;
	add_timer(&but_timer);
#else
    mod_timer(&but_timer, jiffies + BUT_DELAY);
#endif

}


static int but_open_spi0(struct inode *inode, struct file *file)
{
	struct rs_device_private *rs_priv;
	unsigned int minor = iminor(inode);
	
	printk(KERN_DEBUG PREFIX "!!!! open Button SPI0\n");

	rs_priv = kmalloc(sizeof(struct rs_device_private), GFP_KERNEL);
	if (rs_priv == NULL)
	{
		return -ENOMEM;
	}
	
	rs_priv->chnum = minor;
	rs_priv->drv = drv_access2;
	//rs_priv->drv = &drv_access;
	
	file->private_data = rs_priv;
	
	return SUCCESS;
}

static int but_open_spi1(struct inode *inode, struct file *file)
{
	struct rs_device_private *rs_priv;
	unsigned int minor = iminor(inode);
	
	printk(KERN_DEBUG PREFIX "!!!! open Button SPI1\n");

	rs_priv = kmalloc(sizeof(struct rs_device_private), GFP_KERNEL);
	if (rs_priv == NULL)
	{
		return -ENOMEM;
	}
	
	rs_priv->chnum = minor;
	rs_priv->drv = drv_access2;
	//rs_priv->drv = &drv_access;
	
	file->private_data = rs_priv;
	
	return SUCCESS;
}

static int but_release_spi0(struct inode *inode, struct file *file)
{
	struct rs_device_private *rs_priv = file->private_data;
	printk(KERN_DEBUG PREFIX "!!!! close SPI0\n");
	
	kfree(rs_priv);
	
	rs_priv = NULL;
	
	return SUCCESS;
}

static int but_release_spi1(struct inode *inode, struct file *file)
{
	struct rs_device_private *rs_priv = file->private_data;
	printk(KERN_DEBUG PREFIX "!!!! close SPI1\n");
	
	kfree(rs_priv);
	
	rs_priv = NULL;
	
	return SUCCESS;
}

static loff_t but_lseek (struct file *file, loff_t offset, int orig)
{
    loff_t testpos = 0;

    printk(KERN_DEBUG PREFIX "!!!! but_lseek orig=%d\n", orig);

    switch(orig)
    {
        case SEEK_SET:
            testpos = offset;
        break;

        case SEEK_CUR:
            testpos = file->f_pos + offset;
        break;

        case SEEK_END:
            testpos = IO_BUF_SIZE + offset;
        break;

        default:
        return ~EINVAL;
    }

    testpos = testpos < IO_BUF_SIZE ? testpos : IO_BUF_SIZE;
    testpos = testpos >= 0 ? testpos : 0;
    file->f_pos = testpos;

    //printk(KERN_INFO "!!! seeking to %ld position\n", (long)testpos);
    return testpos;
}

#define RS_BUFF_RW 256
#define BUFF_SPI1 8

static ssize_t but_read_spi1(struct file *file, char __user *buf, size_t lbuf, loff_t *offset)
{
    dgt_xpdev_t *kbuf;
    u64 data;
    struct rs_device_private *rs_priv = file->private_data;

    kbuf = (dgt_xpdev_t *)rs_priv->drv;
    if(lbuf > RS_BUFF_RW)
    {
        printk(KERN_INFO "!!! Error read_spi1 lbuf=%ld\n", lbuf);
        return -EFAULT;
    }
    
    mutex_lock(&_reg64_mutex);

    data = _reg64;

    mutex_unlock(&_reg64_mutex);

    if(copy_to_user(buf, (unsigned char*)&data, lbuf))
    {
        printk(KERN_INFO "!!! Error copy data\n");
        return -EFAULT;
    }

    return lbuf;
}

//This function set LED stripes
//For meters clear (call == 255 and size == 255)
//Start work meters(call == 255 and size == 254)
//Stop work meters (call == 255 and size == 255)
static long but_ioctl_spi0(struct file *file, unsigned int call, unsigned long size)
{
	dgt_xpdev_t *kbuf;
	long result = 0;
	struct rs_device_private *rs_priv = file->private_data;

	kbuf = (dgt_xpdev_t *)rs_priv->drv;

	if((call == 255) && (size == 255))
	{
		printk(KERN_INFO "!!! CLEAR METERS\n");
		is_set_clear_meters = 1;
		return result;
	}

	if((call == 255) && (size == 254))
	{
		printk(KERN_INFO "!!! START WORK METERS\n");
		is_work_meters = 1;
		return result;
	}

	if((call == 255) && (size == 253))
	{
		printk(KERN_INFO "!!! STOP WORK METERS\n");
		is_work_meters = 0;
		return result;
	}


	//printk(KERN_INFO "!!! but_ioctl_spi0 call=%d size=%ld\n", call, size);
	{
		int iii;
		unsigned int color_reg = call;
		unsigned int speed_reg = size;

		if(speed_reg == 0)
		{
			speed_reg = 0x0000030B;
		}

		//write light for I2C
		for(iii = 0; iii < 3; iii++)
		{
			unsigned int i2c_reg = 0x0;
			if(iii == 0)
			{
				i2c_reg |= 0x000005A8;
			}
			else
			if(iii == 1)
			{
				i2c_reg |= color_reg | (0x100); //0x00000111;
			}
			else
			if(iii == 2)
			{
				i2c_reg |= speed_reg | (0x300);
			}

			dgt_xpdev_writel(kbuf, AXI4_I2C_REG, i2c_reg);

			while(1)
			{
				unsigned int status_i2c = dgt_xpdev_readl(kbuf, AXI4_FPGA_GMIO0_REG);
				unsigned int bool1 = (status_i2c & (1<<2));

				//printk(KERN_INFO "!!!!!# stat_i2c_reg iii=%d bool1=%d reg=%x\n",
				//	   iii, bool1, i2c_reg);

				if(bool1 == 0)
				{
					break;
				}

				udelay(10);
			}
			udelay(5000);
		}
	}

	return result;
}


//Get state switchers and buttons (number button set call)
static long but_ioctl_spi1(struct file *file, unsigned int call, unsigned long size)
{
	long result = 0;
	dgt_xpdev_t *kbuf;
	struct rs_device_private *rs_priv;

	//printk(KERN_INFO "!!!!!# but_ioctl_spi1\n");

	rs_priv= file->private_data;
	kbuf = (dgt_xpdev_t *)rs_priv->drv;

	//printk(KERN_INFO "!!!!!# SW butt=%d\n", call);

	if((10 <= call) && (call <= 16))
	{
		unsigned int SX_DOUT = readl(drv_access.iobase + AXI4_FPGA_GMIOI_REG); //dgt_xpdev_readb(kbuf, arrayAddrSwitch[call - 1]);
		//printk(KERN_INFO "!!!!!# SX_DOUT=%x\n", SX_DOUT);

		if( SX_DOUT & (1<< (call - 10)) )
		{
			result = 1;
		}
		else
		{
			result = 0;
		}
	}

	return result;
}


static ssize_t but_write_spi0(struct file *file, const char __user *buf, size_t lbuf, loff_t *offset)
{
	dgt_xpdev_t *kbuf;
	int nbytes;
	unsigned char data_ch = 0;
	unsigned char data[RS_BUFF_RW];
	
	int COUNT_WRITE = 1;

	struct rs_device_private *rs_priv = file->private_data;
	
	if(lbuf > RS_BUFF_RW)
	{
		printk(KERN_INFO "!!! Error write_spi0 very big data lbuf=%ld\n", lbuf);
		return -EFAULT;
	}


	kbuf = (dgt_xpdev_t *)rs_priv->drv;
	spin_lock(&kbuf->lock);
	copy_from_user(data, buf, lbuf);
	nbytes = lbuf;

	data_ch = *(data + 0) - 0x30;
	
	if (!( (0 <= data_ch) && ( data_ch <= 9) ))
	{
		printk(KERN_DEBUG PREFIX "!!!! Error input first symbol(0..9)=%c\n", *(data + 0));
		return nbytes;
	}


	if(lbuf > 1)
	{
		if( (0x30 <= *(data + 1)) && (*(data + 1) <= 0x39) )
		{
			data_ch = 10*data_ch + ( *(data + 1) - 0x30 );
		}
		
		if( *(data + 1) == '|' )
		{
			int d1 = 0;
			int d2 = 0;
			
			sscanf(data, "%d|%d", &d1, &d2);
			//printk(KERN_DEBUG PREFIX "!!!! d1=%d d2=%d\n", d1, d2);
			COUNT_WRITE = d2*2;
			//return nbytes;
		}
	}

	if(0 <= data_ch && data_ch <= 7)
	{
		//for meters, write in nvram
		mutex_lock(&meter_mutex);

		g_meters_data.array_meters[data_ch] += (COUNT_WRITE >> 1);

		mutex_unlock(&meter_mutex);

	}
	else
		if(8 <= data_ch && data_ch <= 39)
		{
			output_reg64_user |= 1 << data_ch;
		}

	*offset += nbytes;
	spin_unlock(&kbuf->lock);

	//printk(KERN_INFO "!!! write device=%s nbytes=%d ppos=%d\n\n", MRAM_NAME, nbytes, (int)*ppos);
	return nbytes;
}


static int create_but(void)
{
    int i;
    int err;
    dev_t dev;

    printk(KERN_DEBUG PREFIX "!!!! create Button dev\n");

    err = alloc_chrdev_region(&dev, 0, BUT_NUMB, BUT_NAME);

    rs_dev_major = MAJOR(dev);
    but_class = class_create(THIS_MODULE, BUT_NAME);

    for (i = 0; i < BUT_NUMB; i++)
    {
        cdev_init( &but_dev_data[i].cdev, &(but_dev_foops[i]) );
        but_dev_data[i].cdev.owner = THIS_MODULE;
        cdev_add(&but_dev_data[i].cdev, MKDEV(rs_dev_major, i), 1);

        but_dev_data[i].rsdev = device_create(but_class, NULL, MKDEV(rs_dev_major, i), NULL, "agt_spi%d", i);
    }


    {
        int iii;

        meters_data l_meters_data;
        memset(&l_meters_data, 0, sizeof(meters_data));

        //check meters csum
		{
			unsigned int get_turn = readl(drv_access.iobase + AXI4_MRAM_BANK0_REG + TURN_ADDRESS);
			u64 get_mask = readq(drv_access.iobase + AXI4_MRAM_BANK0_REG + MASK_ADDRESS);

			if((get_turn != 0) && (get_turn != 1))
			{
				printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! init turn and mask\n");
				writel(0, drv_access.iobase + AXI4_MRAM_BANK0_REG + TURN_ADDRESS);
				writel(0, drv_access.iobase + AXI4_MRAM_BANK1_REG + TURN_ADDRESS);

				writeq(0, drv_access.iobase + AXI4_MRAM_BANK0_REG + MASK_ADDRESS);
				writeq(0, drv_access.iobase + AXI4_MRAM_BANK1_REG + MASK_ADDRESS);
				get_turn = 0;
				get_mask = 0;
			}
			l_meters_data.turn = get_turn;
			l_meters_data.mask = get_mask;
		}

        for(iii = 0; iii < COUNT_METERS; iii++)
        {
            unsigned int get_meter = readl(drv_access.iobase + AXI4_MRAM_BANK0_REG + METERS_ADDRESS + 4*iii);
            unsigned int sum1 = checksum(get_meter);
            unsigned int sum2 = readl(drv_access.iobase + AXI4_MRAM_BANK0_REG + CSUM_ADDRESS + 4*iii);
            unsigned int sum3 = checksum(get_meter + 1);

            if(sum1 != 0)
            {
                printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not null csumm meter=%d\n", iii);
            }

            if(sum1 != sum2)
            {
                if(sum3 == sum2)
                {
                    printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! set meter=%d pluse one\n", iii);

                    writel(get_meter + 1, drv_access.iobase + AXI4_MRAM_BANK0_REG + METERS_ADDRESS + 4*iii);
                    writel(get_meter + 1, drv_access.iobase + AXI4_MRAM_BANK1_REG + METERS_ADDRESS + 4*iii);

                    l_meters_data.array_meters[iii]     = get_meter + 1;
                    l_meters_data.array_crc[iii]        = sum2;

                }
                else
                {
                    unsigned int null_int = 0x0;

                    printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! reset csumm meter=%d\n", iii);

                    writel(null_int, drv_access.iobase + AXI4_MRAM_BANK0_REG + CSUM_ADDRESS + 4*iii);
                    writel(null_int, drv_access.iobase + AXI4_MRAM_BANK1_REG + CSUM_ADDRESS + 4*iii);

                    writel(null_int, drv_access.iobase + AXI4_MRAM_BANK0_REG + METERS_ADDRESS + 4*iii);
                    writel(null_int, drv_access.iobase + AXI4_MRAM_BANK1_REG + METERS_ADDRESS + 4*iii);
                }
            }
            else
            {
                l_meters_data.array_meters[iii] = get_meter;
            }
        }

		mutex_lock(&meter_mutex);
		memcpy(&g_meters_data, &l_meters_data, sizeof(meters_data) );
		mutex_unlock(&meter_mutex);

		if(g_meters_data.turn == 1)
		{
    		output_reg64 = 0x0000000000003000;
		}
		else
		{
    		output_reg64 = 0x00000000000030FF;
		}

    }

    //init timer for check buttons state
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
    init_timer(&but_timer);
    but_timer.expires = jiffies + BUT_DELAY;
    but_timer.function = timer_func_buttons;
    but_timer.data = 42;
#else
    timer_setup(&but_timer, timer_func_buttons, 0);
    but_timer.expires = jiffies + BUT_DELAY;
#endif
    add_timer(&but_timer);

    //init timer meters
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
    init_timer(&meter_timer);
    meter_timer.expires = jiffies + METER_DELAY;
    meter_timer.function = timer_func_meter;
    meter_timer.data = 42;
#else
    timer_setup(&meter_timer, timer_func_meter, 0);
    meter_timer.expires = jiffies + METER_DELAY;
#endif
    add_timer(&meter_timer);

    return 0;
}

static int remove_but(void)
{
    int i;
    printk(KERN_DEBUG PREFIX "!!!! remove Button dev\n");

    del_timer(&but_timer);

    del_timer(&meter_timer);

    for(i = 0; i < BUT_NUMB; i++)
    {
        device_destroy(but_class, MKDEV(rs_dev_major, i));
    }
    class_unregister(but_class);
    class_destroy(but_class);

    unregister_chrdev_region(MKDEV(rs_dev_major, 0), MINORMASK);

    return 0;
}

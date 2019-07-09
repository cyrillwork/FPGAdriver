/*************** NVRAM data ****************************/

#define MRAM_NAME "MRAM"
#define MRAM_NUMB 1

static struct cdev mram_cdev;

static int mram_dev_major = 0;

static struct class *mram_class = NULL;
struct device *mram_device = NULL;

//static int mram_major = 289, mram_minor = 0;

//static dev_t dev_mram;
static int counter_mram = 0;

#define IO_BUF_SIZE (4*1024*1024)
#define IO_BUF_SIZE_METERS (4*1024*1023)

#define RW_BLOCK_SIZE 8

static void *io_buf = NULL;

/*************** NVRAM functions  ****************************/

static int open_mram(struct inode *inode, struct file *file)
{
	char *kbuf = kmalloc(DGT_BUF_SIZE, GFP_KERNEL);
	memcpy(kbuf, &drv_access, DGT_BUF_SIZE);

	file->private_data = kbuf;
	printk(KERN_INFO "!!! open /dev/%s buf_size=%ld\n", MRAM_NAME, DGT_BUF_SIZE);
	counter_mram++;

	printk(KERN_INFO "!!! counter: %d\n\n", counter_mram);
	printk(KERN_INFO "!!! module refcounter: %d\n\n", module_refcount(THIS_MODULE));
	return 0;
}

static int release_mram(struct inode *inode, struct file *file)
{
	char *kbuf;
	printk(KERN_INFO "!!! release /dev/%s\n", MRAM_NAME);
	
	kbuf = file->private_data;
	printk(KERN_INFO "!!! free buffer \n\n");

	if(kbuf)
	{
		kfree(kbuf);
	}
	
	file->private_data = NULL;
	return 0;
}

static ssize_t read_mram(struct file *file, char __user *buf, size_t lbuf, loff_t *ppos)
{
	dgt_xpdev_t *kbuf;
	int nbytes;
	
	if(lbuf > IO_BUF_SIZE)
	{
		printk(KERN_INFO "!!! Read Error too big request block=%ld MAX_SIZE=%d\n", lbuf, IO_BUF_SIZE);
		return -EFAULT;
	}

	if(*ppos > IO_BUF_SIZE)
	{
		printk(KERN_INFO "!!! Read Error too big request offset=%lld MAX_SIZE=%d\n", *ppos, IO_BUF_SIZE);
		return -ENOSPC;
	}
	
	//printk(KERN_INFO "!!! read mram lbuf=%ld ppos=%d\n", lbuf, (int)*ppos);
	kbuf = (dgt_xpdev_t *)file->private_data;

        memcpy_fromio(io_buf, kbuf->iobase + AXI4_MRAM_BANK0_REG + *ppos, lbuf);

//	{
//              int i;
//		unsigned int lCycl, lrest;
		
//		lCycl = lbuf/RW_BLOCK_SIZE;
//		lrest = lbuf%RW_BLOCK_SIZE;

//		for(i = 0; i < lCycl; i++)
//		{
//			*((u64*)io_buf + i) = dgt_xpdev_readq(kbuf, AXI4_MRAM_BANK0_REG + *ppos + i*RW_BLOCK_SIZE);
//		}
//		for(i = 0; i < lrest; i++)
//		{
//			*((u8*)io_buf + i + lCycl*RW_BLOCK_SIZE) = dgt_xpdev_readb(kbuf, AXI4_MRAM_BANK0_REG + *ppos + lCycl*RW_BLOCK_SIZE + i);
//		}
//	}

	if(copy_to_user(buf, io_buf, lbuf))
	{
		printk(KERN_INFO "!!! copy data\n");
		return -EFAULT;
	}
	
	nbytes = lbuf;
	*ppos += nbytes;
	
	//printk(KERN_INFO "!!! read device=%s lbuf=%ld nbytes=%d ppos=%d\n\n", MRAM_NAME, lbuf, nbytes, (int)*ppos);
	return nbytes;
}

static ssize_t write_mram(struct file *file, const char __user *buf, size_t lbuf, loff_t *ppos)
{
	dgt_xpdev_t *kbuf;
	//unsigned char data[MAX_RW_SIZE];
	int nbytes;

	if(*ppos > IO_BUF_SIZE)
	{
		printk(KERN_INFO "!!! Write Error too big request offset=%lld MAX_SIZE=%d\n", *ppos, IO_BUF_SIZE);
		return -ENOSPC;
	}

	if ((IO_BUF_SIZE_METERS <= *ppos) && (*ppos < IO_BUF_SIZE))
	{
		printk(KERN_INFO "!!! Try write in meters area offset=%lld\n", *ppos);
		return lbuf;
	}
	
	kbuf = (dgt_xpdev_t *)file->private_data;
	copy_from_user((unsigned char*)io_buf, buf, lbuf);

        memcpy_toio(kbuf->iobase + AXI4_MRAM_BANK0_REG + *ppos, io_buf, lbuf);
        memcpy_toio(kbuf->iobase + AXI4_MRAM_BANK1_REG + *ppos, io_buf, lbuf);

	
//	{
//              int i;
//		unsigned int lCycl, lrest;
		
//		lCycl = lbuf/RW_BLOCK_SIZE;
//		lrest = lbuf%RW_BLOCK_SIZE;

//		for(i = 0; i < lCycl; i++)
//		{
//			dgt_xpdev_writeq(kbuf, AXI4_MRAM_BANK0_REG + *ppos + i*RW_BLOCK_SIZE,  *((u64*)io_buf + i));
//			dgt_xpdev_writeq(kbuf, AXI4_MRAM_BANK1_REG + *ppos + i*RW_BLOCK_SIZE,  *((u64*)io_buf + i));
//		}

//		for(i = 0; i < lrest; i++)
//		{
//			dgt_xpdev_writeb(kbuf, AXI4_MRAM_BANK0_REG + *ppos + lCycl*RW_BLOCK_SIZE + i,  *((u8*)io_buf + i));
//			dgt_xpdev_writeb(kbuf, AXI4_MRAM_BANK1_REG + *ppos + lCycl*RW_BLOCK_SIZE + i,  *((u8*)io_buf + i));
//		}
//	}

        nbytes = lbuf;
        *ppos += nbytes;

	//printk(KERN_INFO "!!! write device=%s nbytes=%d ppos=%d\n\n", MRAM_NAME, nbytes, (int)*ppos);
	
	return nbytes;
}

static loff_t lseek_mram (struct file *file, loff_t offset, int orig)
{
	loff_t testpos = 0;

	//printk(KERN_INFO "!!! lseek request offset=%lld MAX_SIZE=%d\n", offset, IO_BUF_SIZE);

	if(offset > IO_BUF_SIZE)
	{
		printk(KERN_INFO "!!! lseek Error too big request offset=%lld MAX_SIZE=%d\n", offset, IO_BUF_SIZE);
		return -ENOSPC;
	}

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
	//printk(KERN_INFO "!!! seeking to %ld position orig=%d\n", (long)testpos, orig);
	return testpos;
}

static struct file_operations mram_foops = 
{
  .owner          = THIS_MODULE,
  .read           = read_mram,
  .write          = write_mram,
  .open           = open_mram,
  .llseek         = lseek_mram,
  .unlocked_ioctl = NULL,
  .release        = release_mram,
  .mmap           = NULL,
};

static void create_mramdev(void)
{
	int i = 0;
	int err;
	dev_t dev;
	
	printk(KERN_DEBUG PREFIX "!!!! create NVRAM\n");
	
	err = alloc_chrdev_region(&dev, 0, MRAM_NUMB, MRAM_NAME);
	
	mram_dev_major = MAJOR(dev);

	printk(KERN_DEBUG PREFIX "!!!! create NVRAM mram_dev_major=%d\n", mram_dev_major);

	mram_class = class_create(THIS_MODULE, MRAM_NAME);
	
	cdev_init(&mram_cdev, &mram_foops);
	mram_cdev.owner = THIS_MODULE;
	
	cdev_add(&mram_cdev, MKDEV(mram_dev_major, 0), 1);
	
	mram_device = device_create(mram_class, NULL, MKDEV(mram_dev_major, 0), NULL, "nvram%d", i);
	
	if (IS_ERR(mram_device)) 
	{
		printk(KERN_DEBUG PREFIX "!!!! Error create NVRAM\n");
	}
	else
	{
		printk(KERN_DEBUG PREFIX "!!!! create device OK\n");
	}

	
	io_buf = kmalloc(IO_BUF_SIZE, GFP_USER);
	if(io_buf == NULL)
	{
		printk(KERN_DEBUG PREFIX "!!!! Error alloc io_buf NVRAM\n");
	}

	printk(KERN_DEBUG PREFIX "!!!! create_mramdev OK\n");
}

static void remove_mramdev(void)
{
	printk(KERN_INFO PREFIX "!!! Remove mram_chrdev\n");
	if(io_buf)
	{
		kfree(io_buf);
		io_buf = NULL;
	}
	
	device_destroy(mram_class, MKDEV(mram_dev_major, 0));
	
	class_unregister(mram_class);
	class_destroy(mram_class);
	
	unregister_chrdev_region(MKDEV(mram_dev_major, 0), MINORMASK);
}


//static struct class *uart_class = NULL;

#define UART_NAME				"ttyPS"
#define UART_AGT_NAME			"uartagt"

#define UART_MAJOR				204
#define UART_MINOR				187

#define AGT_NR_UARTS			9

#define ULITE_RX				0x00
#define ULITE_TX				0x04
#define ULITE_STATUS			0x08
#define ULITE_CONTROL			0x0c

#define ULITE_REGION			16

#define ULITE_STATUS_RXVALID	0x01
#define ULITE_STATUS_RXFULL		0x02
#define ULITE_STATUS_TXEMPTY	0x04
#define ULITE_STATUS_TXFULL		0x08
#define ULITE_STATUS_IE			0x10
#define ULITE_STATUS_OVERRUN	0x20
#define ULITE_STATUS_FRAME		0x40
#define ULITE_STATUS_PARITY		0x80

#define ULITE_CONTROL_RST_TX	0x01
#define ULITE_CONTROL_RST_RX	0x02
#define ULITE_CONTROL_IE		0x10

struct platform_device *array_platform_dev[AGT_NR_UARTS];

static struct uart_driver ulite_uart_driver;

static int ulite_assign(struct device *dev, int id, u32 base, int irq);


//in (0 or 1) meaning LCR 7 bit set or not
//RO Receiver Buffer Register (0)
#define AXI_RBR 0x1000
//WO Transmitter Holding Register (0)
#define AXI_THR 0x1000
//R/W Interrupt Enable Register (0)
#define AXI_IER 0x1004
//RO Interrupt Identification Register (x)
#define AXI_IIR 0x1008
//R/W FIFO Control Register (x)
#define AXI_FCR 0x1008
//R/W Line Control Register (x)
#define AXI_LCR 0x100C
//R/W Modem Control Register (x)
#define AXI_MCR 0x1010
//R/W Line Status Register (x)
#define AXI_LSR 0x1014
//R/W Modem Status Register (x)
#define AXI_MSR 0x1018
//R/W Scratch Register (x)
#define AXI_SCR 0x101C
//R/W Divisor Latch (Least Significant Byte) Register (1)
#define AXI_DLL 0x1000
//R/W Divisor Latch (Most Significant Byte) Register (1)
#define AXI_DLM 0x1004

//Modem Control Register (MCR)
//Data Terminal Ready
#define AXI_MCR_DTR		0x00000001
//Request To Send
#define AXI_MCR_RTS		0x00000002
//User Output 1
#define AXI_MCR_OUT1	0x00000004
//User Output 2
#define AXI_MCR_OUT2	0x00000008
//Loop back
#define AXI_MCR_LOOP	0x00000010

//Modem Status Register (MSR)
//Delta Clear To Send
#define AXI_MSR_DCTS	0x00000001
//Delta Data Set Ready
#define AXI_MSR_DDSR	0x00000002
//Trailing Edge Ring Indicator
#define AXI_MSR_TERI	0x00000004
//Delta Data Carrier Detect
#define AXI_MSR_DDCD	0x00000008
//Clear To Send
#define AXI_MSR_CTS		0x00000010
//Data Set Ready
#define AXI_MSR_DSR		0x00000020
//Ring Indicator
#define AXI_MSR_RI		0x00000040
//Data Carrier Detect
#define AXI_MSR_DCD		0x00000080

#define DEFAULT_SPEED 9600

static unsigned int AXI_BaudTable[] = 
{
    0x0CB7,	// 2400
    0x065B,	// 4800
    0x032D,	// 9600
    0x0196,	// 19200
    0x00AB,	// 38400
    0x0087,	// 57600
    0x0043,	// 115200
};

typedef struct AXI_PortParam AXI_PortParam;

struct AXI_PortParam
{
	unsigned int baud;		/* speed default 9600 */
	unsigned int parity;	/* 1 - enable, 0 - none */
	unsigned int even;		/* 1 - even, 0 - odd */
	unsigned int stop;		/* 0 - 1 stop bit, 1 - 2 or 1.5*/
	unsigned int bits;		/* 3 - 8bits, 2 - 7bits, 1 - 6bits, 0 - 5bits*/
	unsigned int is9th;
};

//AGT_NR_UARTS
static AXI_PortParam AXI_SpeedTable[AGT_NR_UARTS] = 
{
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 },
	{ DEFAULT_SPEED, 0, 0, 0, 3, 0 }
};

/* UART_IER Masks */
#define ERBFI                    0x01  /* Enable Receive Buffer Full Interrupt */
#define ETBEI                    0x02  /* Enable Transmit Buffer Empty Interrupt */
#define ELSI                     0x04  /* Enable RX Status Interrupt */
#define EDSSI                    0x08  /* Enable Modem Status Interrupt */
#define EDTPTI                   0x10  /* Enable DMA Transmit PIRQ Interrupt */
#define ETFI                     0x20  /* Enable Transmission Finished Interrupt */
#define ERFCI                    0x40  /* Enable Receive FIFO Count Interrupt */

#define LSR_TX_BUFFER_EMPTY		0x00000020

#define IER_MODEM_STATUS        0x00000008
#define IER_RX_LINE             0x00000004
#define IER_TX_EMPTY            0x00000002
#define IER_RX_DATA             0x00000001

#define GET_TEMP(LSR) (LSR & 0x00000040)
#define GET_THRE(LSR) (LSR & 0x00000020)
#define GET_DR(LSR)   (LSR & 0x00000001)

#define AXI_UartClock	125000000

static struct uart_port ulite_ports[AGT_NR_UARTS];

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

static int axi_receive(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;

	//printk(PREFIX "!!!!!!!! ulite_receive\n");


	//if ((stat & (ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN
	//	     | ULITE_STATUS_FRAME)) == 0)
	//	return 0;

	/* stats */
	//if (stat & ULITE_STATUS_RXVALID) 
	{
		port->icount.rx++;
		//ch = uart_in32(ULITE_RX, port);
		ch = readb(port->membase + AXI_RBR);

		//dgt_xpdev_writeb(&drv_access, AXI4_INTERRUPT_REG + 0x10,  0x0);
		//dgt_xpdev_writeb(&drv_access, AXI4_INTERRUPT_REG + 0x10,  0x1);
	
		//if (stat & ULITE_STATUS_PARITY)
		//	port->icount.parity++;
		//printk(PREFIX "!!!!!!!! ulite_receive ch=%u\n", ch);

		tty_insert_flip_char(tport, ch, flag);

	}
	
	
	/*
	if (stat & ULITE_STATUS_OVERRUN)
		port->icount.overrun++;

	if (stat & ULITE_STATUS_FRAME)
		port->icount.frame++;
	*/

	/* drop byte with parity error if IGNPAR specificed */
	//if (stat & port->ignore_status_mask & ULITE_STATUS_PARITY)
	//	stat &= ~ULITE_STATUS_RXVALID;

	//stat &= port->read_status_mask;

	//if (stat & ULITE_STATUS_PARITY)
	//	flag = TTY_PARITY;


	//stat &= ~port->ignore_status_mask;

	//if (stat & ULITE_STATUS_RXVALID)
	//	tty_insert_flip_char(tport, ch, flag);

	//if (stat & ULITE_STATUS_FRAME)
	//	tty_insert_flip_char(tport, 0, TTY_FRAME);

	//if (stat & ULITE_STATUS_OVERRUN)
	//	tty_insert_flip_char(tport, 0, TTY_OVERRUN);

	return 1;
}

static void ulite_stop_tx(struct uart_port *port);

static int axi_transmit(struct uart_port *port, unsigned int LSR)
{
	struct circ_buf *xmit  = &port->state->xmit;
	//printk(PREFIX "!!!!!!!! ulite_transmit\n");
	if (LSR & LSR_TX_BUFFER_EMPTY)
	{
		int count1;
		//printk(PREFIX "!!!!!!!! ulite_transmit x_char=%x port->icount.tx=%d xmit->tail=%d\n", 
		//	port->x_char, port->icount.tx, xmit->tail);

		if (port->x_char) 
		{
			//printk(PREFIX "_%hhx\n", port->x_char);
			writeb(port->x_char, port->membase + AXI_THR);
			port->x_char = 0;
			port->icount.tx++;
			return 1;
		}

		if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
				ulite_stop_tx(port);
				return 0;
		}

		count1 = port->fifosize >> 1;
		do {
			//printk(PREFIX "__%hhx\n", xmit->buf[xmit->tail]);
			writeb(xmit->buf[xmit->tail], port->membase + AXI_THR);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
			port->icount.tx++;

			if ( uart_circ_empty(xmit) )
				break;

		} while(--count1 > 0);


		// wake up
		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		{
			uart_write_wakeup(port);
		}

		if (uart_circ_empty(xmit))
		{
			ulite_stop_tx(port);
		}

	}
	return 1;
}




static irqreturn_t ulite_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct uart_port *port = dev_id;
	//printk(PREFIX "!!!!!!!! irq port=%d\n", port->line);
	{
		// read Rerister IRQ, check irq my or not
		unsigned int reg_irq = dgt_xpdev_readl(&drv_access, AXI4_INTERRUPT_REG);
		//unsigned int _IIR    = dgt_xpdev_readl(&drv_access, AXI4_INTERRUPT_REG + 0x04);

		unsigned int _shift = port->line;
		unsigned int my_irq = 0;
		
		if(port->line == 8) //UART9 door logger
		{
			_shift++;
		}
		
		my_irq = (reg_irq >> 16) & (1 << _shift);
		
		//printk(PREFIX "!!!!!!!!!! reg_irq=%x _IIR=%x id=%d my_irq=%x\n", reg_irq, _IIR, port->line, my_irq);
		
		if(my_irq == 0)
		{
			return IRQ_NONE;
		}
	}
	
	spin_lock_irqsave(&port->lock, flags);
	

	{

		unsigned int IIR = readl(port->membase + AXI_IIR) & 0x0000000F;//olny ints
		unsigned int LSR = readl(port->membase + AXI_LSR);

		//printk(PREFIX "!!!!!!!! irq port=%d IIR=%x LSR=%x TEMP=%d, THRE=%d, DR=%d, _IIR=%x\n", port->line, IIR, LSR, TEMP, THRE, DR, _IIR);
		if(IIR)
		{
			//We need to read IIR for reset interrupts
		}

		if(GET_DR(LSR))
		{
			//printk(PREFIX "!!!!!!!! irq ulite_receive\n");
			axi_receive(port);
			tty_flip_buffer_push(&port->state->port);
		}

		if(GET_TEMP(LSR) || GET_THRE(LSR)) //cyrill
		{
				//printk(PREFIX "!!!!!!!! irq ulite_transmit\n");
			axi_transmit(port, LSR);
		}
	}
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}


static unsigned int ulite_tx_empty(struct uart_port *port)
{
	unsigned int LSR = readl(port->membase + AXI_LSR);
	//printk(PREFIX "!!!!!!!! ulite_tx_empty\n");	
	return (GET_TEMP(LSR) > 0) ? TIOCSER_TEMT : 0;
}

static unsigned int ulite_get_mctrl(struct uart_port *port)
{
	u32 ret = 0;
	u32 MSR = readl(port->membase + AXI_MSR);
	
	if(MSR & AXI_MSR_DCD)
		ret |= TIOCM_CD;
		
	if(MSR & AXI_MSR_CTS)
		ret |= TIOCM_CTS;
		
	if(MSR & AXI_MSR_DSR)
		ret |= TIOCM_DSR;
		
	if(MSR & AXI_MSR_RI)
		ret |= TIOCM_RI;
		
	//printk(PREFIX "!!!!!!!! get_mcrtl ret %x\n", ret);

	return ret;
	
//	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ulite_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 val;
	
	//printk(PREFIX "!!!!!!!! ulite_set_mctrl\n");
	
	val = readl(port->membase + AXI_MCR);

	//printk(PREFIX "!!!!!!!!1 set_mcrtl MCR %x\n", val);
	
	if(mctrl & TIOCM_RTS)
		val |= AXI_MCR_RTS;
		
	if(mctrl & TIOCM_DTR)
		val |= AXI_MCR_DTR;

	//printk(PREFIX "!!!!!!!!2 set_mcrtl MCR %x\n", val);
	
	writel(val, port->membase + AXI_MCR);
}

static void ulite_stop_tx(struct uart_port *port)
{
	//printk(PREFIX "!!!!!!!! ulite_stop_tx begin\n");	
	while(1)
	{
		unsigned int LSR = readl(port->membase + AXI_LSR);
		//printk(PREFIX "!!!!!!!! ulite_stop_tx %d\n", count1);
		if( GET_THRE(LSR) > 0) //THRE
			break;
		//udelay(1000);
		cpu_relax();
	}

	{	//CLEAR ETBEI
		unsigned int CLEAR_ETBEI = 0xd;
		unsigned int IER = readl(port->membase + AXI_IER);
		IER &= CLEAR_ETBEI;
		//IER |= ERBFI;
		writel(IER, port->membase + AXI_IER);
	}
	//printk(PREFIX "!!!!!!!! ulite_stop_tx end\n");
}

static void ulite_start_tx(struct uart_port *port)
{
	unsigned int LSR = readl(port->membase + AXI_LSR);
	unsigned int IER = readl(port->membase + AXI_IER);

	//printk(PREFIX "!!!!!!! ulite_start_tx port=%d LSR=%x\n", port->line, LSR);
	//set ETBEI
	IER |= ETBEI;
	//IER &= ~ERBFI;
	writel(IER, port->membase + AXI_IER);

	axi_transmit(port, LSR);
}

static void ulite_stop_rx(struct uart_port *port)
{
	unsigned int CLEAR_ERBFI = 0xe;
	unsigned int IER = readl(port->membase + AXI_IER);

	//printk(PREFIX "!!!!!!!! ulite_stop_rx\n");
	//writel(0x00000001, port->membase + AXI_IER);

	//clear ERBFI
	IER &= CLEAR_ERBFI;
	writel(IER, port->membase + AXI_IER);
}

static void ulite_break_ctl(struct uart_port *port, int ctl)
{
	/* N/A */
	//printk(PREFIX "!!!!!!!! ulite_break_ctl\n");
}

static void set_port_param(struct uart_port *port)
{
	//cyrill
	//printk(PREFIX "!!!!!!!!!!!!!!!!!!!111 set_port_param\n");
	//printk(PREFIX "!!!!!!!!!!!!!!!!!!!111 baud=%d\n", AXI_SpeedTable[port->line].baud);

	{ //read divisor for test
		//unsigned int res;
		unsigned int write_reg = 0;
		unsigned int DLL = 0x0000002D;
		unsigned int DLM = 0x00000003;

		//write 7 bit in LCR (DLAB bit)
		//writel(0x00000080, drv_access.iobase + AXI4_UART4_REG + AXI_LCR);
		writel(0x00000080, port->membase + AXI_LCR);
		
		switch(AXI_SpeedTable[port->line].baud)
		{
			case 2400:
    			write_reg = AXI_BaudTable[0];
			break;
			case 4800:
    			write_reg = AXI_BaudTable[1];
			break;
			case 9600:
    			write_reg = AXI_BaudTable[2];
			break;
			case 19200:
    			write_reg = AXI_BaudTable[3];
			break;
			case 38400:
    			write_reg = AXI_BaudTable[4];
			break;
			case 57600:
    			write_reg = AXI_BaudTable[5];
			break;
			case 115200:
    			write_reg = AXI_BaudTable[6];
			break;
			default:
    			write_reg = AXI_BaudTable[2];//9600
			break;
		}
		
		DLL = write_reg & 0x000000ff;
		DLM = write_reg >> 8;

		writel(DLL, port->membase + AXI_DLL);
		writel(DLM, port->membase + AXI_DLM);

		//res = readl(port->membase + AXI_DLM);
		//res = readl(drv_access.iobase + AXI4_UART4_REG + AXI_DLM);
		//printk(PREFIX "!!!!!!!! DLM=%x\n", DLM);
		//res = readl(port->membase + AXI_DLL);
		//res = readl(drv_access.iobase + AXI4_UART4_REG + AXI_DLL);
		//printk(PREFIX "!!!!!!!! DLL=%x\n", DLL);

		//res = readl(port->membase + AXI_FCR);
		//res = readl(drv_access.iobase + AXI4_UART4_REG + AXI_FCR);
		//printk(PREFIX "!!!!!!!! FCR=%x\n", res);
	}

	{ //set default value (8bits, 1stop, pairity NONE)
		unsigned int wr_lcr =0x00000000;
		//unsigned int wr_lcr =0x00000003;
		wr_lcr |= AXI_SpeedTable[port->line].bits;

		
		if(AXI_SpeedTable[port->line].parity)
		{
			if(AXI_SpeedTable[port->line].even)
			{
				//printk(PREFIX "!!!!!!!! Pairity even\n");
				wr_lcr = 0x00000018;
			}
			else
			{
				//printk(PREFIX "!!!!!!!! Pairity odd\n");
				wr_lcr = 0x00000008;
			}
			wr_lcr |= AXI_SpeedTable[port->line].bits;
		}
		
		
		if(AXI_SpeedTable[port->line].is9th)
		{
			//printk(PREFIX "!!!!!!!! Pairity Even\n");
			//wr_lcr = 0x0000003B;
			//printk(PREFIX "!!!!!!!! stick parity\n");
			wr_lcr |= (1<<5);
		}

		if(AXI_SpeedTable[port->line].stop)
		{
			//printk(PREFIX "!!!!!!!! 2 stop bits\n");
			wr_lcr |= (1<<2);
		}
		
		//printk(PREFIX "!!!!!!!! w_lcr=%x\n", wr_lcr);
		
		writel(wr_lcr, port->membase + AXI_LCR);

		//write IER enable trans holder register empty int and recv data int
		writel(0x00000003, port->membase + AXI_IER);
		//only receive
		//writel(0x00000001, port->membase + AXI_IER);

		//write FCR enable FIFOs, reset and set MSB trigger level
		writel(0x00000007, port->membase + AXI_FCR);
		//writel(0x00000047, port->membase + AXI_FCR);
	
	}
}

static int ulite_startup(struct uart_port *port)
{
	int ret;
	//printk(PREFIX "!!!!!!!! ulite_startup\n");

	ret = request_irq(port->irq, ulite_isr, IRQF_SHARED, "uartagt", port);
	if (ret)
	{
		return ret;
	}
	
	set_port_param(port);
	return 0;
}

static void ulite_shutdown(struct uart_port *port)
{
	//printk(PREFIX "!!!!!!!! ulite_shutdown\n");

	{
		unsigned int IER = 0x00000000;
		writel(IER, port->membase + AXI_IER);
	}

	//printk(PREFIX "!!!!!!!! disables int port=%d\n", port->line);

	free_irq(port->irq, port);
}

static void ulite_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned int parity = 0;
	unsigned int even   = 0;
	
	unsigned long flags;
	unsigned int baud;
	unsigned int bits = 3;

	//printk(PREFIX "!!!!!!!! ulite_set_termios line=%d\n", port->line);

	{
		//struct circ_buf *xmit = &port->state->xmit;
		//int state_circ = uart_circ_empty(xmit);
		//printk(PREFIX "!!!!!!!! state_circ=%d\n",  state_circ);
	
		udelay(1000);

	}


	spin_lock_irqsave(&port->lock, flags);


	{
		switch (termios->c_cflag & CSIZE)
		{
			case CS5:
				//printk(PREFIX "!!!!!!!! CS5\n");
				bits = 0;
			break;
			case CS6:
				//printk(PREFIX "!!!!!!!! CS6\n");
				bits = 1;
			break;
			case CS7:
				//printk(PREFIX "!!!!!!!! CS7\n");
				bits = 2;
			break;
			case CS8:
				//printk(PREFIX "!!!!!!!! CS8\n");
				bits = 3;
			break;
		}
		//printk(PREFIX "!!!!!!!! bits=%d\n", bits);
	}


	if (termios->c_cflag & PARENB)
	{
		//printk(PREFIX "!!!!!!!! ulite_set_termios PARENB set\n");
		parity = 1;
		even = 1;
	}

	if (termios->c_cflag & PARODD)
	{
		//printk(PREFIX "!!!!!!!! ulite_set_termios PARODD set\n");
		parity = 1;
		even = 0;
	}

	if (termios->c_cflag & CMSPAR)
	{
		//printk(PREFIX "!!!!!!!! ulite_set_termios 9bit set set\n");
		AXI_SpeedTable[port->line].is9th = 1;
	}
	else
	{
		AXI_SpeedTable[port->line].is9th = 0;
	}

	if (termios->c_cflag & CSTOPB)
	{
		//printk(PREFIX "!!!!!!!! ulite_set_termios send two stop bits\n");
		AXI_SpeedTable[port->line].stop = 1;
	}
	else
	{
		AXI_SpeedTable[port->line].stop = 0;
	}

	//port->read_status_mask = ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN| ULITE_STATUS_TXFULL;
	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;

	if (termios->c_iflag & INPCK)
	{
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	}

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
	{
		//port->ignore_status_mask |= ULITE_STATUS_PARITY | ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;
		port->ignore_status_mask |= UART_LSR_FE | UART_LSR_PE;
	}
	if (termios->c_iflag & IGNBRK)
	{
		port->ignore_status_mask |= UART_LSR_BI;
		if(termios->c_iflag & IGNPAR)
		{
			port->ignore_status_mask |= UART_LSR_OE;
		}
	}
	
	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
	{
		//port->ignore_status_mask |= ULITE_STATUS_RXVALID | ULITE_STATUS_PARITY | ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;
		port->ignore_status_mask |= UART_LSR_DR;
	}
	
	/* update timeout */
	baud = uart_get_baud_rate(port, termios, old, 0, 460800);
	uart_update_timeout(port, termios->c_cflag, baud);


	AXI_SpeedTable[port->line].baud		= baud;
	AXI_SpeedTable[port->line].parity	= parity;
	AXI_SpeedTable[port->line].even		= even;
	AXI_SpeedTable[port->line].bits		= bits;

	//printk(PREFIX "!!!!!!!! ulite_set_termios baud=%d\n", baud);
	
	set_port_param(port);
	

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ulite_type(struct uart_port *port)
{
	//printk(PREFIX "!!!!!!!! ulite_type\n");
	return port->type == PORT_UARTLITE ? UART_AGT_NAME : NULL;
}

static void ulite_release_port(struct uart_port *port)
{
	printk(PREFIX "!!!!!!!! ulite_release_port\n");
	//release_mem_region(port->mapbase, ULITE_REGION);
	//iounmap(port->membase);
	port->membase = NULL;
}

///!!!!!!!!!!!!!!!!!!!!!!!!! cyrill
static int ulite_request_port(struct uart_port *port)
{
	//int ret;

	printk(PREFIX "!!!!!!!! ulite_request_port console: port->mapbase=%llx\n",
		 (unsigned long long) port->mapbase);

/*
	if (!request_mem_region(port->mapbase, ULITE_REGION, "uartlite")) 
	{
		//dev_err(port->dev, "Memory region busy\n");
		printk(PREFIX "!!!!!!!! Memory region busy ulite\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, ULITE_REGION);
	
	if (!port->membase) 
	{
		printk(PREFIX "!!!!!!!! Unable to map registers ulite\n");
		//dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, ULITE_REGION);
		return -EBUSY;
	}
*/
	//port->private_data = &uartlite_be;

/*
	printk(PREFIX "!!!!!!!! ulite_request_port 1\n");
	
	ret = uart_in32(ULITE_CONTROL, port);
	
	uart_out32(ULITE_CONTROL_RST_TX, ULITE_CONTROL, port);

	printk(PREFIX "!!!!!!!! ulite_request_port 2\n");
	
	ret = uart_in32(ULITE_STATUS, port);
	
	
	if ((ret & ULITE_STATUS_TXEMPTY) != ULITE_STATUS_TXEMPTY)
		port->private_data = &uartlite_le;
*/

	//printk(PREFIX "!!!!!!!! ulite_request_port 3\n");

	return 0;
}

static void ulite_config_port(struct uart_port *port, int flags)
{
	printk(PREFIX "!!!!!!!! ulite_config_port\n");
	
	if (!ulite_request_port(port))
		port->type = PORT_UARTLITE;
}

static int ulite_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk(PREFIX "!!!!!!!! ulite_verity_port\n");

	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static struct uart_ops ulite_ops = 
{
	.tx_empty			= ulite_tx_empty,
	.set_mctrl			= ulite_set_mctrl,
	.get_mctrl			= ulite_get_mctrl,
	.stop_tx			= ulite_stop_tx,
	.start_tx			= ulite_start_tx,
	.stop_rx			= ulite_stop_rx,
	.break_ctl			= ulite_break_ctl,
	.startup			= ulite_startup,
	.shutdown			= ulite_shutdown,
	.set_termios		= ulite_set_termios,
	.type				= ulite_type,
	.release_port		= ulite_release_port,
	.request_port		= ulite_request_port,
	.config_port		= ulite_config_port,
	.verify_port		= ulite_verify_port,
};

static int ulite_release(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	int rc = 0;

	if (port) 
	{
		rc = uart_remove_one_port(&ulite_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rc;
}


static int ulite_probe(struct platform_device *pdev)
{
//	struct resource *res;
//	int irq;
	int id = pdev->id;

	//printk(PREFIX "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! platform ulite_probe pdev->id=%d\n", pdev->id);

/*
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
	{
		printk(PREFIX "!!! Error platform_get_resource\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
	{
		printk(PREFIX "!!! Error platform_get_irq\n");
		return -ENXIO;
	}
*/

	return ulite_assign(&pdev->dev, id, 0/*res->start*/, drv_access.irq);
}

static int ulite_remove(struct platform_device *pdev)
{
	printk(PREFIX "!!! platform_ulite remove\n");
	return ulite_release(&pdev->dev);
}


static struct uart_driver ulite_uart_driver = 
{
	.owner			= THIS_MODULE,
	.driver_name	= UART_AGT_NAME,
	.dev_name		= UART_NAME,
	.major			= UART_MAJOR,
	.minor			= UART_MINOR,
	.nr		    	= AGT_NR_UARTS,
};

static struct platform_driver ulite_platform_driver = 
{
	.probe = ulite_probe,
	.remove = ulite_remove,
	.driver = 
	{
		.owner = THIS_MODULE,
		.name  = UART_AGT_NAME,
		//.of_match_table = of_match_ptr(ulite_of_match),
	},
};


/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

/** ulite_assign: register a uartlite device with the driver
 *
 * @dev: pointer to device structure
 * @id: requested id number.  Pass -1 for automatic port assignment
 * @base: base address of uartlite registers
 * @irq: irq number for uartlite
 *
 * Returns: 0 on success, <0 otherwise
 */
static int ulite_assign(struct device *dev, int id, u32 base, int irq)
{
	struct uart_port *port;
	int rc;
	
	printk(PREFIX "uartagt: assign id=%d\n", id);

	/* if id = -1; then scan for a free id and use that */
	if (id < 0) 
	{
		for (id = 0; id < AGT_NR_UARTS; id++)
			if (ulite_ports[id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= AGT_NR_UARTS) 
	{
		//dev_err(dev, "%s%i too large\n", ULITE_NAME, id);
		return -EINVAL;
	}

//	printk(PREFIX "uartlite: calling uart_register_driver() 1\n");

/*
	if ((ulite_ports[id].mapbase) && (ulite_ports[id].mapbase != base)) 
	{
		//dev_err(dev, "cannot assign to %s%i; it is already in use\n", ULITE_NAME, id);
		return -EBUSY;
	}
*/
	port = &ulite_ports[id];

	spin_lock_init(&port->lock);

//cyrill
	port->fifosize		= 16;
	port->regshift		= 2;
	
	//port->iotype		= UPIO_MEM;
	port->iobase		= id + 1;

	port->membase = drv_access.iobase + AXI_UART_REG[id];//AXI4_UART4_REG;
	port->mapbase = drv_access.base_start;
	
	//printk(PREFIX "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! addr=%x\n", AXI_UART_REG[id]);
	

	port->ops   = &ulite_ops;
	port->irq   = irq;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev   = dev;
	port->type  = PORT_16550;
	port->line  = id;
	
	//port->uartclk = 460800 * 16;//125000000; 
	//clk_get_rate(cdns_uart_data->uartclk);
	
	/*
	{
		struct clk *clk_my = clk_get(&platform_dev->dev, "c_axi_aclk");
		
		if (IS_ERR(clk_my)) 
		{
			printk(PREFIX "uartlite: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  ERROR c_axi_aclk\n");
		}
		else
		{
			printk(PREFIX "uartlite: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  OK get clk\n");
		}
	}
	*/

	//printk(PREFIX "uartlite: calling uart_register_driver() 2\n");
	dev_set_drvdata(dev, port);

	/* Register the port */
	rc = uart_add_one_port(&ulite_uart_driver, port);
	
	if (rc) 
	{
		//dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		printk(PREFIX "Error  uart_add_one_port(&ulite_uart_driver, port)\n");
		port->mapbase = 0;
		//dev_set_drvdata(dev, NULL);
		return rc;
	}
	
	
	if(id == 8)
	{
		u32 val = 0x0;
		
		printk(PREFIX "!!!!!!!! Add init UART9\n");
		
		val |= AXI_MCR_OUT1 | AXI_MCR_OUT2;
		
		writel(val, port->membase + AXI_MCR);
		
		printk(PREFIX "!!!!!!!! end UART9\n");
	}
	
	
	return 0;
}


/** ulite_release: register a uartlite device with the driver
 *
 * @dev: pointer to device structure
 */
static int remove_uart(void)
{
	int i;
	int rc = 0;
	//int id = 0;
	
	/*
	struct uart_port *port = &ulite_ports[id];;
	printk(PREFIX "uartlite: ulite_release %d\n", id);
	
	rc = 0;

	if (port) 
	{
		rc = uart_remove_one_port(&ulite_uart_driver, port);
		//dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}
	
	printk(PREFIX "uartlite: uart_unregister_driver\n");
	*/
	platform_driver_unregister(&ulite_platform_driver);
	

	for(i = 0; i < AGT_NR_UARTS; i++)
	{
		platform_device_unregister(array_platform_dev[i]);
	}
	
	uart_unregister_driver(&ulite_uart_driver);
	printk(PREFIX "uartlite: device destroy\n");
	//device_destroy(uart_class, MKDEV(UART_MAJOR, 0));
	//printk(PREFIX "uartlite: class_unregister\n");
	//class_unregister(uart_class);
	//class_destroy(uart_class);
	
	printk(PREFIX "uartlite: end release\n");
	return rc;
}

static int create_uart(void)
{
	int i;
	int rc;
	//struct platform_device *pdev;
	
	printk(PREFIX "uartlite: calling uart_register_driver()\n");
	
	rc = uart_register_driver(&ulite_uart_driver);
	if (rc) 
	{
		printk(PREFIX "!!! Eror uart_register_driver\n");
		return rc;
	}

	for(i = 0; i < AGT_NR_UARTS; i++)
	{
		//printk(PREFIX "uartlite: create device platform register\n");
		array_platform_dev[i] = platform_device_register_simple(UART_AGT_NAME, i, NULL, 0);
		
		if(IS_ERR(array_platform_dev[i]))
		{
			printk(PREFIX "!!! Eror platform_device register simple %d\n", i);
			return 1;
		}
	}

	rc = platform_driver_register(&ulite_platform_driver);
	if (rc)
	{
		printk(PREFIX "!!! Eror platform_driver_register\n");
		return rc;
	}
	//printk(PREFIX "uartlite: calling uart_register_driver() 3\n");
	return 0;
}


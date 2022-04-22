#include <linux/init.h>                        
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/unistd.h>
#include <uapi/asm/unistd.h>
#include <linux/termios.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/serial.h>
#include "wk2xxx.h"
#include <linux/device.h>
#include <linux/io.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/acpi.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <asm/byteorder.h>


//#define DEBUG_WK2XXX
#define WK_FIFO_FUNCTION

#define CONFIG_DEVFS_FS

#define DBG_PORT	1 

#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0
#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8

#define UART_SC_RX     0x00
#define UART_SC_TX     0x00
#define UART_SC_DLL    0x00
#define UART_SC_DLH    0x04
#define UART_SC_IER    0x04
#define UART_SC_IIR    0x08
#define UART_SC_FCR    0x08
#define UART_SC_LCR    0x0C
#define UART_SC_MCR    0x10
#define UART_SC_LSR    0x14
#define UART_SC_MSR    0x18
#define UART_SC_SCR    0x1C
#define UART_SC_SRBR   0x30 
#define UART_SC_STHR   0x30
#define UART_SC_FAR    0x70
#define UART_SC_TFR    0x74
#define UART_SC_RFW    0x78
#define UART_SC_USR    0x7C
#define UART_SC_TFL    0x80
#define UART_SC_RFL    0x84
#define UART_SC_SRR    0x88
#define UART_SC_SRTS   0x8C
#define UART_SC_SBCR   0x90
#define UART_SC_SDMAM  0x94
#define UART_SC_SFE    0x98
#define UART_SC_SRT    0x9C
#define UART_SC_STET   0xA0
#define UART_SC_HTX    0xA4
#define UART_SC_DMASA  0xA8
#define UART_SC_CPR    0xF4
#define UART_SC_UCV    0xF8
#define UART_SC_CTR    0xFC


static DEFINE_MUTEX(wk2xxxs_lock);      /* race on probe */
static DEFINE_MUTEX(wk2xxxs_wr_lock); 
static DEFINE_MUTEX(wk2xxs_work_lock);                /* work on probe */
static DEFINE_MUTEX(wk2xxxs_global_lock); 

/*wk2xxx reset and irq pins */
int wk_reset,wk_irq;

#ifdef CONFIG_OF
struct of_sc_serial {
	unsigned int id;
	unsigned int use_dma;	
	unsigned int uartclk;
	unsigned int uartbaud;
};
#endif

struct uart_sc_port {
	struct uart_port	port;
	struct platform_device	*pdev;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control	*rst;
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		iir;
	unsigned char		fcr;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#if 0
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
#endif

	char			name[16];
	char			fifo[64];
	char 			fifo_size;
	unsigned long		port_activity;
	struct work_struct uart_work;
	struct work_struct uart_work_rx;
	struct workqueue_struct *uart_wq;
	struct rk_uart_dma *dma;
};

struct uart_sc_port  *g_uart_port;;


static inline unsigned int serial_in(struct uart_sc_port *up, int offset)
{
	//offset = offset << 2;
	//printk(KERN_ALERT "%s!  address=0x%x!!\n",__func__,up->port.membase + offset);
	return __raw_readl(up->port.membase + offset);
}

/* Save the LCR value so it can be re-written when a Busy Detect IRQ occurs. */
static inline void dwapb_save_out_value(struct uart_sc_port *up, int offset,unsigned char value)
{
	if (offset == UART_SC_LCR)
		up->lcr = value;
}

/* Read the IER to ensure any interrupt is cleared before returning from ISR. */
static inline void dwapb_check_clear_ier(struct uart_sc_port *up, int offset)
{
	if (offset == UART_SC_TX || offset == UART_SC_IER)
		serial_in(up, UART_SC_IER);
}

static inline void serial_out(struct uart_sc_port *up, int offset, unsigned char value)
{   
	dwapb_save_out_value(up, offset, value);
	__raw_writel(value, up->port.membase + (offset ));
	if (offset != UART_SC_TX){
	}
	dwapb_check_clear_ier(up, offset);
}


/* Uart divisor latch read */
static inline int serial_dl_read(struct uart_sc_port *up)
{
	return serial_in(up, UART_SC_DLL) | serial_in(up, UART_SC_DLH) << 8;
}


/* Uart divisor latch write */
static int serial_dl_write(struct uart_sc_port *up, unsigned int value)
{
	unsigned int tmout = 100;
	while (!(serial_in(up, UART_SC_LCR) & 0x80)) {
		if (--tmout == 0) {
			if (up->port.line != DBG_PORT)
				printk(KERN_ALERT "set serial.%d baudrate fail with DLAB not set\n", up->port.line);
			return -1;
		}
	}
	serial_out(up, UART_SC_DLL, value & 0xff);
	serial_out(up, UART_SC_DLH, value >> 8 & 0xff);
	return 0;
}

#ifdef DEBUG_WK2XXX
static int uart_dump_register(struct uart_sc_port *up)
{
	unsigned int reg_value = 0;

	reg_value =serial_dl_read(up);
	printk(KERN_ALERT "%s  UART_SC_DLL and UART_SC_DLH:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_IER);
	printk(KERN_ALERT "%s  UART_SC_IER:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_IIR);
	printk(KERN_ALERT "%s  UART_SC_IIR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_LSR);
	printk(KERN_ALERT "%s  UART_SC_LSR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_MSR);
	printk(KERN_ALERT "%s  UART_SC_MSR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_MCR);
	printk(KERN_ALERT "%s  UART_SC_MCR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_SCR);
	printk(KERN_ALERT "%s  UART_SC_SCR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, UART_SC_LCR);
	printk(KERN_ALERT "%s  UART_SC_LCR:0x%08x\n", __func__,reg_value);
	reg_value = serial_in(up, 0x68);
	printk(KERN_ALERT "%s  UART_SC_REVISION:0x%08x\n", __func__,reg_value);

#if 0
	reg_value = serial_in(up, UART_USR);
	printk(KERN_ALERT "%s  UART_USR:0x%X\n", __func__,reg_value);
#endif
	return 0;
}
#endif


static int serial_lcr_write(struct uart_sc_port *up, unsigned char value)
{
#if 0
	while(serial_in(up, UART_USR) & 0X01){
		if (--tmout == 0){
			if(up->port.line != DBG_PORT)
				printk(KERN_ALERT"set serial.%d lcr = 0x%02x timeout\n", up->port.line, value);
			return -1;
		}
		udelay(1);
	}
#endif
	serial_out(up, UART_SC_LCR, value);
	return 0;
}

void uart_putc(struct uart_sc_port *up,unsigned char ch)
{
	int timeout = 500;
	
	while (!(serial_in(up, UART_SC_LSR) & 0x40) && timeout)
		timeout--;
	serial_out(up, UART_SC_TX ,ch);

	timeout = 2000;
	while (!(serial_in(up, UART_SC_LSR) & 0x40) && timeout) {
		timeout--;
		udelay(1);
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

int uart_getc(struct uart_sc_port *up,unsigned char *dat)
{
	int timeout = 40000;
	int ret = 0;

	while (!(serial_in(up, UART_SC_LSR) & 0x01) && timeout) {
		--timeout;
		if (timeout == 0) {
			printk(KERN_ALERT "%s  uart get data timeout\n",__func__);
			ret = 1;
		}
		udelay(1);
	}

	*dat= (unsigned char)serial_in(up, UART_SC_RX);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  ret:%d\n",__func__,ret);
#endif
	return ret;
}

void uart_clean_buf(struct uart_sc_port *up)
{
	unsigned char dat;
	dat= (unsigned char)serial_in(up, UART_SC_RX);
	dat= (unsigned char)serial_in(up, UART_SC_RX);
}

static int get_baud_rate(struct uart_sc_port *up,int baud)
{
	int ret = 0;
	ret = up->port.uartclk / (16 * baud);
	return ret;
}

static int uart_init(struct uart_sc_port *up)
{   
	int ret;
	unsigned int baud, quot;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT"%s\n",__func__);
#endif 
	baud=115200;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s uart_baud:%d\n",__func__,baud);
#endif
	serial_lcr_write(up, 0X03|0x80);

	quot = get_baud_rate(up,baud);

	if (serial_dl_write(up, quot)) {
		if (up->port.line != DBG_PORT)
			printk(KERN_ALERT "%s  serial: %d DBG_PORT:%x set dll fail\n",__func__,up->port.line,DBG_PORT);
		serial_out(up, UART_SC_SRR, 0x01);
	}
	serial_lcr_write(up, 0X03); 

	ret = serial_in(up, UART_SC_LCR);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  UART_SC_LCR:%d\n",__func__,ret);
	//uart_dump_register(up);
	printk(KERN_ALERT"%s  exit\n",__func__);
#endif
	return 0;
}

struct wk2xxx_port 
{
	struct uart_port port;//[NR_PORTS];
	spinlock_t conf_lock;	/* shared data */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int suspending;
	void (*wk2xxx_hw_suspend) (int suspend);
	int tx_done;

	int force_end_work;
	int irq;
	int minor;		/* minor number */
 	int tx_empty; 
	int tx_empty_flag;

	//int start_tx;
	int start_tx_flag;
	int stop_tx_flag;
	int stop_rx_flag; 
	int irq_flag;
	int conf_flag;

	int tx_empty_fail;
	int start_tx_fail;
        int stop_tx_fail;
   	int stop_rx_fail;
   	int irq_fail;
        int conf_fail;

	uint8_t new_lcr;
	uint8_t new_scr; 
	/*set baud 0f register*/
	uint8_t new_baud1;
	uint8_t new_baud0;
	uint8_t new_pres;
	unsigned rst;
	unsigned irq_gpio;
	unsigned int rs485_gpio;
};

static struct wk2xxx_port wk2xxxs[NR_PORTS]; /* the chips */

static int wk2xxx_read_reg(struct uart_sc_port *up,uint8_t port,uint8_t reg,uint8_t *dat)
{
	int ret=0;
	uint8_t wk_command;

	mutex_lock(&wk2xxxs_wr_lock);
	uart_clean_buf(up);
	wk_command = 0x40 | (((port-1) << 4) | reg);
	uart_putc(up,wk_command);
	ret = uart_getc(up,dat);
	if (ret == 1) {
		printk(KERN_ALERT "%s  wk_command:0x%x\n",__func__,wk_command);
	}
   	mutex_unlock(&wk2xxxs_wr_lock);

	return ret;
}

static int wk2xxx_write_reg(struct uart_sc_port *up,uint8_t port,uint8_t reg,uint8_t dat)
{
	uint8_t wk_command;

	mutex_lock(&wk2xxxs_wr_lock);
	wk_command = (((port-1) << 4) | reg);
	uart_putc(up,wk_command);
	uart_putc(up,dat);
	mutex_unlock(&wk2xxxs_wr_lock);

	return 0;
}

#ifdef WK_FIFO_FUNCTION
static int wk2xxx_read_fifo(struct uart_sc_port *up,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
	uint8_t wk_command;
	int i,ret=0;

	mutex_lock(&wk2xxxs_wr_lock);  
	uart_clean_buf(up);
	if (fifolen>0) {
		wk_command=0xc0|(((port-1)<<4)|(fifolen-1));
		uart_putc(up,wk_command);
		for (i = 0; i < fifolen; i++) {
			ret = uart_getc(up,dat+i);
			if (ret == 1) {
				printk(KERN_ALERT "%s  command:0x%x i:%d ret:0x%x\n",__func__,wk_command,i,ret);
			}
		}
	}
	mutex_unlock(&wk2xxxs_wr_lock);

	return 0;
}
#endif

static int wk2xxx_write_fifo(struct uart_sc_port *up,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
	uint8_t wk_command;
	int i;

	mutex_lock(&wk2xxxs_wr_lock);
	if (fifolen > 0) {
		wk_command = (0x80 | ((port-1) << 4) | (fifolen-1));
		uart_putc(up,wk_command);
		for (i = 0; i < fifolen; i++)
			uart_putc(up,*(dat+i));
	}
	mutex_unlock(&wk2xxxs_wr_lock);

	return 0;
}


static void wk2xxxirq_app(struct uart_port *port);
static void conf_wk2xxx_subport(struct uart_port *port);
static void wk2xxx_work(struct work_struct *w);
static void wk2xxx_stop_tx(struct uart_port *port);
static u_int wk2xxx_tx_empty(struct uart_port *port);

static int wk2xxx_dowork(struct wk2xxx_port *s)
{    
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT"%s\n",__func__);
#endif
	if (!s->force_end_work && !work_pending(&s->work) && !freezing(current) && !s->suspending) {
		queue_work(s->workqueue, &s->work);
#ifdef DEBUG_WK2XXX
		printk("queue_work ok\n");
#endif
		return 1;	
	} else {
#ifdef DEBUG_WK2XXX
		printk("queue_work error\n");
#endif
		return 0;
	}
}

static void wk2xxx_work(struct work_struct *w)
{  
	struct wk2xxx_port *s = container_of(w, struct wk2xxx_port, work);
	uint8_t rx;
        
	//int work_tx_empty_flag;
	int work_start_tx_flag; 

	int work_stop_rx_flag;
	//int work_stop_tx_flag;
	
	int work_irq_flag;
	//int work_irq_fail;
	int work_conf_flag;
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif
	do {
		mutex_lock(&wk2xxs_work_lock);
		/*
			work_tx_empty_flag = s->tx_empty_flag;
			if(work_tx_empty_flag)
			s->tx_empty_flag = 0;
		*/
		work_start_tx_flag = s->start_tx_flag;
		if (work_start_tx_flag)
			s->start_tx_flag = 0;
		/*
			work_stop_tx_flag = s->stop_tx_flag;
			if(work_stop_tx_flag)
			s->stop_tx_flag = 0;
		*/
		work_stop_rx_flag = s->stop_rx_flag;
		if (work_stop_rx_flag)
			s->stop_rx_flag = 0;
		work_conf_flag = s->conf_flag;
		/*
			if(work_conf_flag)
			s->conf_flag = 0;
		*/
                           
		work_irq_flag = s->irq_flag;
		if (work_irq_flag)
			s->irq_flag = 0;

		mutex_unlock(&wk2xxs_work_lock);
		if (work_start_tx_flag) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,&rx);
			rx |= WK2XXX_TFTRIG_IEN|WK2XXX_RFTRIG_IEN|WK2XXX_RXOUT_IEN; 
			wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,rx);
		}

		if (work_stop_rx_flag) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,&rx);
			rx &=~WK2XXX_RFTRIG_IEN;
			rx &=~WK2XXX_RXOUT_IEN;
			wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,rx);
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,&rx);
			rx &= ~WK2XXX_RXOVT_INT;
			rx &= ~WK2XXX_RFTRIG_INT;
			wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,rx);
		}

		if (work_irq_flag) {
			wk2xxxirq_app(&s->port);
			s->irq_fail = 1;
		}

        }while (!s->force_end_work && !freezing(current) && (work_irq_flag || work_stop_rx_flag)); 

        if (s->start_tx_fail) {
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,&rx);
		rx |= WK2XXX_TFTRIG_IEN|WK2XXX_RFTRIG_IEN|WK2XXX_RXOUT_IEN;
		wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,rx);
		s->start_tx_fail =0;
        }
				
        if (s->stop_rx_fail) {
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,&rx);
		rx &=~WK2XXX_RFTRIG_IEN;
		rx &=~WK2XXX_RXOUT_IEN;
		wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,rx);
		
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,&rx);
		rx &= ~WK2XXX_RFTRIG_INT;
		rx &= ~WK2XXX_RXOVT_INT;
		wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,rx);
		s->stop_rx_fail =0;
        }
	
	if (s->irq_fail) {
		s->irq_fail = 0;
		enable_irq(s->port.irq);      
	}
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
}

static void wk2xxx_rx_chars(struct uart_port *port)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t fsr,lsr,dat[1],rx_dat[256]={0};
	unsigned int ch, flg,sifr, ignored=0,status = 0,rx_count=0;
	int rfcnt=0,rx_num=0,i;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT"%s  in\n",__func__);
#endif
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_LSR,dat);
	lsr = dat[0];
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,dat);
	sifr = dat[0];
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  port:%ld fsr:%x lsr:%x\n",__func__,s->port.iobase,fsr,lsr);
#endif
	if (!(sifr & 0x80)) { 
		flg = TTY_NORMAL;
		if (fsr & WK2XXX_RDAT) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_RFCNT,dat);
			rfcnt = dat[0];
			if (rfcnt == 0) {
				rfcnt=255;	
			}
					
			printk(KERN_ALERT "%s  port:%ld RFCNT:0x%x\n",__func__,s->port.iobase,rfcnt);

			while (rfcnt > 0) {  
#ifdef WK_FIFO_FUNCTION  /*FIFP MODE*/
				if (rfcnt > 16) {
					wk2xxx_read_fifo(g_uart_port,s->port.iobase, 16,rx_dat);
					for (i = 0; i < 16; i++) {
						rx_dat[rx_num]=rx_dat[i];
						rx_num++;
    					}
    					rfcnt=rfcnt-16;
				} else {
					wk2xxx_read_fifo(g_uart_port,s->port.iobase, rfcnt,rx_dat);
					for(i = 0; i < rfcnt; i++) {
						rx_dat[rx_num]=rx_dat[i];
						rx_num++;
					}
					rfcnt=0;
				}
#else   /*FDAT MODE*/
				wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FDAT,&rx_dat[rx_num]); 
				rx_num++;
				rfcnt--;
#endif
			}
	
			s->port.icount.rx+=rx_num;
			for (i = 0; i < rx_num; i++) {
				if (uart_handle_sysrq_char(&s->port,rx_dat[rx_num]))//.state, ch))
					break;
#ifdef DEBUG_WK2XXX
				printk(KERN_ALERT "rx_chars:0x%x\n",rx_dat[i]);
#endif
				uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, rx_dat[i], flg);
				rx_count++;
				if ((rx_count >= 64 ) && (s->port.state->port.tty->port != NULL)) {
					tty_flip_buffer_push(s->port.state->port.tty->port);
					rx_count = 0;
  				}
			}
	
  			if ((rx_count > 0)&&(s->port.state->port.tty != NULL)) {
#ifdef DEBUG_WK2XXX          			
				printk(KERN_ALERT "push buffer tty flip port:%ld count:%d\n",s->port.iobase,rx_count);
#endif
				tty_flip_buffer_push(s->port.state->port.tty->port);
				rx_count = 0;
			}
  		}
	} else {
		while (fsr & WK2XXX_RDAT) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FDAT,dat);
			ch = (int)dat[0];
#ifdef DEBUG_WK2XXX
			printk(KERN_ALERT "%s  port:%ld  RXDAT:0x%x\n",__func__,s->port.iobase,ch);
#endif
			s->port.icount.rx++;
#ifdef DEBUG_WK2XXX			
			printk(KERN_ALERT "icount.rx:%d\n",s->port.icount.rx);
#endif
			flg = TTY_NORMAL;
			if (lsr &(WK2XXX_OE | WK2XXX_FE | WK2XXX_PE | WK2XXX_BI)) {
#ifdef DEBUG_WK2XXX
				printk(KERN_ALERT "%s  port:%ld lsr:%x\n",__func__,s->port.iobase,lsr);
#endif
				if (lsr & WK2XXX_PE) {
					s->port.icount.parity++;
					status |= WK2XXX_STATUS_PE;
					flg = TTY_PARITY;
        			}

				if (lsr & WK2XXX_FE) {
					s->port.icount.frame++;
					status |= WK2XXX_STATUS_FE;
					flg = TTY_FRAME;
				}

				if (lsr & WK2XXX_OE) {
					s->port.icount.overrun++;
					status |= WK2XXX_STATUS_OE;
					flg = TTY_OVERRUN;
				}

				if (lsr&fsr & WK2XXX_BI) {
					s->port.icount.brk++;
					status |= WK2XXX_STATUS_BRK;
					flg = TTY_BREAK;
				}
				
				if (++ignored > 100) 
					goto out;
		
				goto ignore_char;       
			}
error_return:
			if (uart_handle_sysrq_char(&s->port,ch)) 
			goto ignore_char;
		
			uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, ch, flg);
			rx_count++;
		
			if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL)) {
				tty_flip_buffer_push(s->port.state->port.tty->port);
				rx_count = 0;
			} 
        	
#ifdef DEBUG_WK2XXX
			printk(KERN_ALERT "s->port.icount.rx:0x%x char:0x%x flg:0x%x port:%ld rx_count:%d\n",s->port.icount.rx,ch,flg,s->port.iobase,rx_count);
#endif
ignore_char:
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,dat);
			fsr = dat[0];
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_LSR,dat);
			lsr = dat[0];
		}
out:
		if ((rx_count > 0) && (s->port.state->port.tty != NULL)) {
			printk(KERN_ALERT "push buffer tty flip port:%ld count:%d\n",s->port.iobase,rx_count);
			tty_flip_buffer_push(s->port.state->port.tty->port);
		}

	} /* end of if (!(sifr & 0x80))*/
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif	
	return;

#ifdef SUPPORT_SYSRQ
	s->port.state->sysrq = 0;
#endif
	goto error_return;
}


static void wk2xxx_tx_chars(struct uart_port *port)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t uart_empty_flag = 0, fsr,tfcnt,dat[1],rx_fifo[16] = {0};
	int count,tx_count,i;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif	
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0

	if (s->port.x_char) {
		wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_FDAT,s->port.x_char);
		s->port.icount.tx++;
		s->port.x_char = 0;
		goto out;
	}

	if(uart_circ_empty(&s->port.state->xmit) || uart_tx_stopped(&s->port)) {
		goto out;
	}

	/*
	 * Tried using FIFO (not checking TNF) for fifo fill:
	 * still had the '1 bytes repeated' problem.
	*/
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];

	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_TFCNT,dat);	
	tfcnt = dat[0];
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  fsr:0x%x tfcnt:0x%x port:%ld\n",__func__,fsr,tfcnt,s->port.iobase);
#endif
	if (tfcnt == 0) {
		if (fsr & WK2XXX_TFULL) {
			tfcnt = 255;
			tx_count = 0;
		} else {
			tfcnt = 0;
			tx_count = 255;
		}
	} else {
		tx_count = 255-tfcnt;
#ifdef DEBUG_WK2XXX
		printk(KERN_ALERT "%s  tx_count:%d port:%ld\n",__func__,tx_count,s->port.iobase); 
#endif
	}

	count = tx_count;
	while (count > 0) {
		if(uart_empty_flag) { 
			uart_empty_flag=0;
			break;
		}
		do {
			if (uart_circ_empty(&s->port.state->xmit)) {
				uart_empty_flag=1;
				break;
			}
			rx_fifo[i]=s->port.state->xmit.buf[s->port.state->xmit.tail];
			i++;
			tx_count--;
			s->port.state->xmit.tail = (s->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
			s->port.icount.tx++;
			if (i >= 16) { 
				break;
			}
#ifdef DEBUG_WK2XXX
			printk(KERN_ALERT "tx_chars:0x%x\n",rx_fifo[i-1]);
#endif
		}while (tx_count > 0);
		
		gpio_set_value(s->rs485_gpio,1);	
	
		wk2xxx_write_fifo(g_uart_port,s->port.iobase,i,rx_fifo);

		gpio_set_value(s->rs485_gpio,0);
	
		count=count-i;
		i=0;
	}
#ifdef DEBUG_WK2XXX
        printk(KERN_ALERT "%s  count:%d port%ld\n",__func__,count,s->port.iobase);
#endif
out:
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];
	if (((fsr & WK2XXX_TDAT) == 0) && ((fsr & WK2XXX_TBUSY) == 0)) {
		if (uart_circ_chars_pending(&s->port.state->xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);
		if (uart_circ_empty(&s->port.state->xmit)) {
			wk2xxx_stop_tx(&s->port);
		}
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
}

static irqreturn_t wk2xxx_irq(int irq, void *dev_id)
{
	struct wk2xxx_port *s = dev_id;
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif
	disable_irq_nosync(s->port.irq);  
	s->irq_flag = 1;
	if (wk2xxx_dowork(s)) {
		//s->irq_flag = 1;
	} else {
		s->irq_flag = 0;
		s->irq_fail = 1;
	}
	return IRQ_HANDLED;
}

static void wk2xxxirq_app(struct uart_port *port)//
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	unsigned int  pass_counter = 0;
	uint8_t sifr,gifr,sier,dat[1];

	uint8_t  gena,gier;
	uint8_t  sifr1, sifr2;
//	uint8_t  sifr3, sifr4;
	uint8_t  sier1,sier2;
//	uint8_t  sier3,sier4;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIFR ,dat);
	gifr = dat[0];		

	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER ,dat);
	gier = dat[0];

	wk2xxx_read_reg(g_uart_port,1,WK2XXX_SIFR,&sifr1);
	wk2xxx_read_reg(g_uart_port,2,WK2XXX_SIFR,&sifr2);
#if 0
	wk2xxx_read_reg(g_uart_port,3,WK2XXX_SIFR,&sifr3);
	wk2xxx_read_reg(g_uart_port,4,WK2XXX_SIFR,&sifr4);
#endif		
	wk2xxx_read_reg(g_uart_port,1,WK2XXX_SIER,&sier1);
	wk2xxx_read_reg(g_uart_port,2,WK2XXX_SIER,&sier2);
#if 0
	wk2xxx_read_reg(g_uart_port,3,WK2XXX_SIER,&sier3);
	wk2xxx_read_reg(g_uart_port,4,WK2XXX_SIER,&sier4);
#endif	
	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,&gena);
	//printk(KERN_ALERT "irq_app.3...gifr:%x  gier:%x  sier1:%x  sier2:%x sier3:%x sier4:%x   sifr1:%x sifr2:%x sifr3:%x sifr4:%x \n",gifr,gier,sier1,sier2,sier3,sier4,sifr1,sifr2,sifr3,sifr4);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT " %s gifr:%x gier:%x sier1:%x sier2:%x sifr1:%x sifr2:%x\n",__func__,gifr,gier,sier1,sier2,sifr1,sifr2);
#endif
	switch (s->port.iobase) {
		case 1:
			if(!(gifr & WK2XXX_UT1INT)) {
				return;
			}
			break;
		case 2:
			if(!(gifr & WK2XXX_UT2INT)) { 			 
				return;
			} 									  
			break;
#if 0
		case 3 :
			if(!(gifr & WK2XXX_UT3INT)) { 			 
				return;
			}
			break;
		case 4 :
			if(!(gifr & WK2XXX_UT4INT)) {
				return;
			}
			break;
#endif
		default:
			break;
	}
		
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,dat);
	sifr = dat[0];
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,dat);
	sier = dat[0];
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  sifr:%x sier:%x \n",__func__,sifr,sier);
#endif
	do {
		if ((sifr & WK2XXX_RFTRIG_INT) || (sifr & WK2XXX_RXOVT_INT)) {
			wk2xxx_rx_chars(&s->port);
		}
		
		if ((sifr & WK2XXX_TFTRIG_INT) && (sier & WK2XXX_TFTRIG_IEN )) {
			wk2xxx_tx_chars(&s->port);
			return;
		}

		if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
			break;
				
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIFR,dat);
		sifr = dat[0];				  
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,dat);
		sier = dat[0];
#ifdef DEBUG_WK2XXX	
		printk(KERN_ALERT "%s  sifr:0x%x sier:0x%x port:%ld\n",__func__,sifr,sier,s->port.iobase);
#endif
	} while ((sifr & WK2XXX_RXOVT_INT) || (sifr & WK2XXX_RFTRIG_INT) || ((sifr & WK2XXX_TFTRIG_INT) && (sier & WK2XXX_TFTRIG_IEN)));
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
}

/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */
static u_int wk2xxx_tx_empty(struct uart_port *port)// or query the tx fifo is not empty?
{
	uint8_t rx;
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif
	mutex_lock(&wk2xxxs_lock);
	if (!(s->tx_empty_flag || s->tx_empty_fail)) {
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,&rx);
		while ((rx & WK2XXX_TDAT) | (rx & WK2XXX_TBUSY)) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,&rx);
			msleep(1);
   		}
		s->tx_empty = ((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))<=0;
		if (s->tx_empty) {
			s->tx_empty_flag =0;
			s->tx_empty_fail=0;
		} else {
			s->tx_empty_fail=0;
			s->tx_empty_flag =0;
		}
	}
	mutex_unlock(&wk2xxxs_lock);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  tx_empty:0x%x\n",__func__,s->tx_empty);
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
	return s->tx_empty;
}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)//nothing
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

static u_int wk2xxx_get_mctrl(struct uart_port *port)// since no modem control line
{       
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}


/*
 *  interrupts disabled on entry
 */
static void wk2xxx_stop_tx(struct uart_port *port)
{
	uint8_t dat[1],sier;
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif
	mutex_lock(&wk2xxxs_lock);
	if (!(s->stop_tx_flag || s->stop_tx_fail)) {

		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,dat);
		sier=dat[0];
		
		s->stop_tx_fail = (sier & WK2XXX_TFTRIG_IEN) > 0;

		if (s->stop_tx_fail) {
			wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,dat);
			sier = dat[0] | WK2XXX_RFTRIG_IEN | WK2XXX_RXOUT_IEN;
			sier &= ~WK2XXX_TFTRIG_IEN;
			wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,sier);

			s->stop_tx_fail =0;
			s->stop_tx_flag=0;
  
          	} else {
			s->stop_tx_fail =0;
			s->stop_tx_flag=0;
		}              
	}
	mutex_unlock(&wk2xxxs_lock); 
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s exit\n",__func__);
#endif
}

/*
 * interrupts may not be disabled on entry
 */
static void wk2xxx_start_tx(struct uart_port *port)
{	
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);       
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	if (!(s->start_tx_flag || s->start_tx_fail)) {  
		s->start_tx_flag = 1;
		if (wk2xxx_dowork(s)) {
			//s->start_tx_flag = 1;
		} else {
			s->start_tx_fail = 1;
			s->start_tx_flag = 0;
		}
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
}

/*
 *  Interrupts enabled
 */
static void wk2xxx_stop_rx(struct uart_port *port)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	if (!(s->stop_rx_flag ||s->stop_rx_fail )) {
		s->stop_rx_flag = 1;
		if (wk2xxx_dowork(s)) {
			//s->stop_rx_flag = 1;
		} else {
			s->stop_rx_flag = 0;
			s->stop_rx_fail = 1;
		}
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

/*
 * No modem control lines
 */
static void wk2xxx_enable_ms(struct uart_port *port)
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

/*
 * Interrupts always disabled.
 */   
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}


static int wk2xxx_startup(struct uart_port *port)
{       
	char b[12];
	uint8_t gena,grst,gier,sier,scr,dat[1];
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  port:%d\n",__func__,(uint8_t)s->port.iobase);
#endif
	if (s->suspending)
		return 0;
	s->force_end_work = 0;
	sprintf(b, "wk2xxx-%d", (uint8_t)s->port.iobase);
	s->workqueue = create_singlethread_workqueue(b);
		
	if (!s->workqueue) {
		//dev_warn(&s->spi_wk->dev, "cannot create workqueue\n");
		return -EBUSY;
        }
	
	INIT_WORK(&s->work, wk2xxx_work);
		
	if (s->wk2xxx_hw_suspend)
		s->wk2xxx_hw_suspend(0);
	mutex_lock(&wk2xxxs_global_lock);   
	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,dat);
	gena=dat[0];
	
	switch (s->port.iobase) {
		case 1:
			gena|=WK2XXX_UT1EN;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		case 2:
			gena|=WK2XXX_UT2EN;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
#if 0
		case 3:
			gena|=WK2XXX_UT3EN;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		case 4:
			gena|=WK2XXX_UT4EN;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
#endif
		default:
			printk(KERN_ALERT "%s  bad iobase:%d\n",__func__,(uint8_t)s->port.iobase);
	}

	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GRST,dat);
	grst=dat[0];

	switch (s->port.iobase) {
		case 1:
			grst|=WK2XXX_UT1RST;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		case 2:
			grst|=WK2XXX_UT2RST;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
#if 0
		case 3:
			grst|=WK2XXX_UT3RST;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		case 4:
			grst|=WK2XXX_UT4RST;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
#endif
		default:
			printk(KERN_ALERT "%s  bad iobase:%d\n",__func__,(uint8_t)s->port.iobase);
            }

	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,dat);
	sier = dat[0];
	sier &= ~WK2XXX_TFTRIG_IEN;
	sier |= WK2XXX_RFTRIG_IEN;
	sier |= WK2XXX_RXOUT_IEN;
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER,sier);

	/*initiate the fifos*/
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_FCR,0xff);//initiate the fifos
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_FCR,0xfc);

	/*set rx/tx interrupt*/
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE,1);	
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_RFTL,0X20);	//rx 32
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_TFTL,0X20);	//tx 32
        
	/*The default baud rate is 115200*/
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD0 ,0x05);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD1 ,0x0);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_PRES ,0x0);

	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE,0);	

	//enable the sub port interrupt
	wk2xxx_read_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER,dat);
	gier = dat[0];

	switch (s->port.iobase) {
		case 1:
			gier|=WK2XXX_UT1IE;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		case 2:
			gier|=WK2XXX_UT2IE;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
#if 0
		case 3:
			gier|=WK2XXX_UT3IE;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		case 4:
			gier|=WK2XXX_UT4IE;
			wk2xxx_write_reg(g_uart_port,WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
#endif
		default:
			printk(KERN_ALERT "%s  bad iobase:%d\n",__func__,(uint8_t)s->port.iobase);
	}

        /*enable uart_rx and uart_tx*/
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SCR,dat);
	scr = dat[0] | WK2XXX_TXEN|WK2XXX_RXEN;
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SCR,scr);

	mutex_unlock(&wk2xxxs_global_lock);
	if (s->wk2xxx_hw_suspend) {
		s->wk2xxx_hw_suspend(0);
	}
	msleep(50);
	uart_circ_clear(&s->port.state->xmit);
	wk2xxx_enable_ms(&s->port);

        // request  
	if(request_irq(s->port.irq, wk2xxx_irq,IRQF_SHARED|IRQF_TRIGGER_LOW,"wk2xxx", s) < 0) {
		s->port.irq = 0;
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
		return -EBUSY;
	 } 
  
	udelay(100);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
	return 0;
}

// Power down all displays on reboot, poweroff or halt 
static void wk2xxx_shutdown(struct uart_port *port)//
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif	
	if (s->suspending)
		return;
	s->force_end_work = 1;

	if (s->workqueue) {
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}
	
	if (s->port.irq) {
		//disable_irq_nosync(s->port.irq);		
		free_irq(s->port.irq,s);
	}
	
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
}

static void conf_wk2xxx_subport(struct uart_port *port)
{   
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t old_sier,lcr,scr,scr_ss,dat[1],baud0_ss,baud1_ss,pres_ss;
	
	lcr = s->new_lcr;
	scr_ss = s->new_scr;
	baud0_ss=s->new_baud0;
	baud1_ss=s->new_baud1;
	pres_ss=s->new_pres;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SIER ,dat);
	old_sier = dat[0];
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER ,old_sier&(~(WK2XXX_TFTRIG_IEN | WK2XXX_RFTRIG_IEN|WK2XXX_RXOUT_IEN)));
    
	do {
		wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_FSR,dat);
	} while (dat[0] & WK2XXX_TBUSY);
	
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_SCR,dat);
	scr = dat[0];
	
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SCR ,scr&(~(WK2XXX_RXEN|WK2XXX_TXEN)));
    
	// set the parity, stop bits and data size //
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_LCR ,lcr);
	// set the baud rate //   
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SIER ,old_sier);
	// set the baud rate //
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE ,1);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD0 ,baud0_ss);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD1 ,baud1_ss);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_PRES ,pres_ss);
	
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD0,dat);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "WK2XXX_BAUD0=0x%X\n", dat[0]);
#endif
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_BAUD1,dat);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "WK2XXX_BAUD1=0x%X\n", dat[0]);
#endif
	wk2xxx_read_reg(g_uart_port,s->port.iobase,WK2XXX_PRES,dat);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "WK2XXX_PRES=0x%X\n", dat[0]);
#endif	
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SPAGE ,0);	
	udelay(10);
	wk2xxx_write_reg(g_uart_port,s->port.iobase,WK2XXX_SCR ,scr|(WK2XXX_RXEN|WK2XXX_TXEN));
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

// change speed
static void wk2xxx_termios( struct uart_port *port, struct ktermios *termios,struct ktermios *old)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	int baud = 0;
	uint8_t lcr,baud1,baud0,pres;
	unsigned short cflag;
	unsigned short lflag;

	cflag = termios->c_cflag;
	lflag = termios->c_lflag;

	baud1=0;
	baud0=0;
	pres=0;
	baud = tty_termios_baud_rate(termios);

	/*11.0592Mhz*/
	switch (baud) {
		case 600:
			baud1=0x4;
			baud0=0x7f;
			pres=0;
			break;
		case 1200:
			baud1=0x2;
			baud0=0x3F;
			pres=0;
			break;
		case 2400:
			baud1=0x1;
			baud0=0x1f;
			pres=0;
			break;
		case 4800:
			baud1=0x00;
			baud0=0x8f;
			pres=0;
			break;
		case 9600:
			baud1=0x00;
			baud0=0x47;
			pres=0;
			break;
		case 19200:
			baud1=0x00;
			baud0=0x23;
			pres=0;
			break;
		case 38400:
			baud1=0x00;
			baud0=0x11;
			pres=0;
			break;
		case 76800:
			baud1=0x00;
			baud0=0x08;
			pres=0;
			break;  
		case 1800:
			baud1=0x01;
			baud0=0x7f;
			pres=0;
			break;
		case 3600:
			baud1=0x00;
			baud0=0xbf;
			pres=0;
			break;
		case 7200:
			baud1=0x00;
			baud0=0x5f;
			pres=0;
			break;
		case 14400:
			baud1=0x00;
			baud0=0x2f;
			pres=0;
			break;
		case 28800:
			baud1=0x00;
			baud0=0x17;
			pres=0;
			break;
		case 57600:
			baud1=0x00;
			baud0=0x0b;
			pres=0;
			break;
		case 115200:
			baud1=0x00;
			baud0=0x05;
			pres=0;
			break;
		case 230400:
			baud1=0x00;
			baud0=0x02;
			pres=0;
			break;
		default:  
			baud1=0x00;
			baud0=0x00;
			pres=0;
			break;
	}
	tty_termios_encode_baud_rate(termios, baud, baud);

	/* we are sending char from a workqueue so enable */
	lcr =0;
	if (cflag & CSTOPB)
		 lcr|= WK2XXX_STPL;//two  stop_bits
        else
		lcr &= ~WK2XXX_STPL;//one  stop_bits
                
        if (cflag & PARENB) {
		lcr|=WK2XXX_PAEN;//enbale spa
                if (!(cflag & PARODD)) {
			lcr |= WK2XXX_PAM1;
			lcr &= ~WK2XXX_PAM0;
		} else {
			lcr |= WK2XXX_PAM0;//PAM0=1
			lcr &= ~WK2XXX_PAM1;//PAM1=0	 
		}
	} else {
		lcr&=~WK2XXX_PAEN;
        }
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  port:%ld lcr:0x%x cflag:0x%x CSTOPB:0x%x PARENB:0x%x PARODD:0x%x\n",__func__,s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif
	s->new_baud1=baud1;
	s->new_baud0=baud0;
	s->new_pres=pres;
	s->new_lcr = lcr;
	
	conf_wk2xxx_subport(&s->port);
#ifdef DEBUG_WK2XXX
  	printk(KERN_ALERT "%s  exit\n",__func__);
#endif
	return ;
}

static const char *wk2xxx_type(struct uart_port *port)
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif	
	return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;//this is defined in serial_core.h
}

/*
* Release the memory region(s) being used by 'port'.
*/
static void wk2xxx_release_port(struct uart_port *port)
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
}

/*
* Request the memory region(s) being used by 'port'.
*/
static int wk2xxx_request_port(struct uart_port *port)//no such memory region needed for vk32
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	return 0;
}

/*
* Configure/autoconfigure the port
*/
static void wk2xxx_config_port(struct uart_port *port, int flags)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n",__func__);
#endif
	if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
		s->port.type = PORT_WK2XXX;
}

/*
* Verify the new serial_struct (for TIOCSSERIAL).
* The only change we allow are to the flags and type, and
* even then only between PORT_vk32xx and PORT_UNKNOWN
*/
static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;	
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT"%s\n",__func__);
#endif
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_PORT)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
        return ret;
}


static struct uart_ops wk2xxx_pops = {
        tx_empty:       wk2xxx_tx_empty,
        set_mctrl:      wk2xxx_set_mctrl,
        get_mctrl:      wk2xxx_get_mctrl,
        stop_tx:        wk2xxx_stop_tx,
        start_tx:       wk2xxx_start_tx,
        stop_rx:        wk2xxx_stop_rx,
        enable_ms:      wk2xxx_enable_ms,
        break_ctl:      wk2xxx_break_ctl,
        startup:        wk2xxx_startup,
        shutdown:       wk2xxx_shutdown,
	set_termios:    wk2xxx_termios,
        type:           wk2xxx_type,
        release_port:   wk2xxx_release_port,
        request_port:   wk2xxx_request_port,
        config_port:    wk2xxx_config_port,
        verify_port:    wk2xxx_verify_port,

};


static struct uart_driver wk2xxx_uart_driver = {
        owner:                  THIS_MODULE,
        major:        		 SERIAL_WK2XXX_MAJOR,
        driver_name:            "ttySWK",
        dev_name:               "ttysWK",
        minor:                  MINOR_START,
        nr:                     NR_PORTS,
        cons:                   NULL//WK2Xxx_CONSOLE,
};

static int uart_driver_registered;
static struct platform_driver wk2xxx_driver;


/*
static void timer_function(unsigned long data)
{
	timer_flag = 0;
    if (total_bytes == 1024)
        correct_count++;
    else {
        error_count++;
        pr_err("---=== correct_count:%u error_count:%d\n", correct_count, error_count);
    }
    total_bytes = 0;
}
*/
static int reset_gpio_init(struct device *dev)
{
	int reset_gpio = -1;
	
	reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  reset_gpio:%d\n",__func__,reset_gpio);
#endif	
	if (!gpio_is_valid(reset_gpio)) {
		printk(KERN_ALERT "%s  invalid reset_gpio: %d\n",__func__,reset_gpio);
		return -1;
    	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  reset_gpio:%d is valid\n",__func__,reset_gpio);
#endif
	if (gpio_request(reset_gpio ,"wk2xxx-reset-gpio")) {
		printk(KERN_ALERT "%s  reset_gpio:%d request fail\n",__func__,reset_gpio);
		gpio_free(reset_gpio);
		return -1;
	}
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  reset_gpio:%d request sucess\n",__func__, reset_gpio);
#endif
	gpio_direction_output(reset_gpio,1);
	
	return reset_gpio;
}


static int irq_gpio_dt(struct device *dev)
{
	int irq_gpio, irq; 

	irq_gpio = of_get_named_gpio(dev->of_node, "irq-gpio", 0); 
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  irq_gpio:%d\n",__func__,irq_gpio);
#endif	
        if (!gpio_is_valid(irq_gpio)) {
		printk(KERN_ALERT "%s  invalid irq_gpio: %d\n",__func__,irq_gpio);
                return -ENODEV;
        }
	
        if (gpio_request(irq_gpio, "wk2xxx-irq-gpio")) {
                printk(KERN_ALERT "%s  irq-gpio resuest failed\n",__func__);
                return -1;
        }
	
        gpio_direction_input(irq_gpio);

        irq = gpio_to_irq(irq_gpio);
	if (!irq) {
		printk(KERN_ALERT "%s  irq:%d get irq failed\n", __func__,irq);
		gpio_free(irq_gpio);
		return -1;
	}
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  irq_gpio: %d, irq: %d\n",__func__,irq_gpio,irq);
#endif
	return irq;
}


static int rs485_gpio_dt(struct device *dev, unsigned char index)
{
	int rs485_gpio = -1;
	char prop_name[20]={0};
	
	sprintf(prop_name,"rs485-%d-dir-gpio",index);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "prop_name:%s\n",prop_name);
#endif
	rs485_gpio = of_get_named_gpio(dev->of_node, prop_name, 0);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s rs485_gpio:%d\n",__func__,rs485_gpio);
#endif
	if (!gpio_is_valid(rs485_gpio)) {
		printk(KERN_ALERT "%s  invalid %s:%d\n",__func__,prop_name,rs485_gpio);
		return -1;
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s %d is valid\n",__func__,rs485_gpio);
#endif                
	if (gpio_request(rs485_gpio, "rs485-dir-gpio")) {
		printk(KERN_ALERT "%s %d request fail\n",prop_name,rs485_gpio);
		gpio_free(rs485_gpio);
		return -1;
	}
	
	gpio_direction_output(rs485_gpio,0);
	return rs485_gpio;
}


static int of_sc_serial_parse_dt(struct device_node *np, struct of_sc_serial *of_serial) 
{
	unsigned int val = 0;
#ifdef USE_DMA
	const char *s = NULL;
	int ret, i = 0;
#endif
	of_serial->id = of_alias_get_id(np, "serial");
	if(!of_property_read_u32(np, "clock-frequency", &val))
		of_serial->uartclk = val;

	if(!of_property_read_u32(np, "current-speed", &val))
               of_serial->uartbaud = val;

#ifdef USE_DMA
	of_serial->use_dma = 0;
	for(i = 0; i < 2; i++) {
		ret = of_property_read_string_index(np, "dma-names", i, &s);
		if(!ret) {
			if(!strcmp(s, "tx"))
				of_serial->use_dma |= TX_DMA;
			 else if (!strcmp(s, "rx"))
				of_serial->use_dma |= RX_DMA;
		}
	}
#endif
	return 0;
}


static int wk2xxx_probe(struct platform_device *pdev)
{
        unsigned char i;
#ifdef DEBUG_WK2XXX
	unsigned char dat[1];
#endif
	int irq;
   	int status;
	int err;
	struct resource	 *uart_resource;
	struct of_sc_serial of_serial;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n", __func__);
#endif
	g_uart_port = devm_kzalloc(&pdev->dev, sizeof(*g_uart_port), GFP_KERNEL);
	if (!g_uart_port) {  
		printk(KERN_ALERT "%s  devm_kzalloc fail\n", __func__);
		return -ENOMEM;
    	}

	uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	devm_request_mem_region(&pdev->dev, uart_resource->start, resource_size(uart_resource),uart_resource->name);
	g_uart_port->port.membase = ioremap(uart_resource->start, uart_resource->end - uart_resource->start + 1);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  g_uart_port ->port.membase:%p\n",__func__,g_uart_port ->port.membase);
#endif    
	irq = platform_get_irq(pdev, 0);
   	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
	   	return irq;
	}
#ifdef DEBUG_WK2XXX
  	 printk(KERN_ALERT "%s  platform_get_irq:%d\n", __func__,irq);
#endif
#ifdef CONFIG_OF
	of_sc_serial_parse_dt(pdev->dev.of_node, &of_serial);
	pdev->id = of_serial.id;
#endif
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  serial:%d pdev->id\n", __func__,pdev->id);
#endif   
	g_uart_port->pdev = pdev;

	g_uart_port->tx_loadsz = 30;
	g_uart_port->port.dev = &pdev->dev;
	g_uart_port->port.type = PORT_8250;
	g_uart_port->port.irq = irq;
	g_uart_port->port.iotype = UPIO_MEM;
	g_uart_port->port.regshift = 2;
	//fifo size default is 32, but it will be updated later when start_up
	g_uart_port->port.fifosize = 32;
	//g_uart_port->port.ops = &serial_rk_pops;
	g_uart_port->port.line = pdev->id;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  g_uart_port->port.line:%d\n",__func__,g_uart_port->port.line);
#endif
	g_uart_port->port.line = DBG_PORT;
	g_uart_port->port.iobase = uart_resource->start;
	g_uart_port->port.mapbase = uart_resource->start;
	//g_uart_port->port.irqflags = IRQF_DISABLED;
	//g_uart_port->port.baud = rks.uartbaud;
	//g_uart_port->port.uartclk = clk_get_rate(g_uart_port->clk);
	g_uart_port->port.uartclk = of_serial.uartclk;

	/*uart3 init*/
	uart_init(g_uart_port);

	wk_reset = reset_gpio_init(&pdev->dev);
	if (wk_reset < 0) {
		printk(KERN_ALERT "%s  reset_gpio:%d request fail\n",__func__,wk_reset);
		return 1;
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  reset_gpio:%d request sucess\n",__func__,wk_reset);
#endif	
	wk_irq = irq_gpio_dt(&pdev->dev);
	if (wk_irq < 0) {		
		printk(KERN_ALERT "%s  irq_gpio:%d request fail\n",__func__,wk_irq);
		return 1;
	}
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  irq_gpio:%d request sucess\n",__func__,wk_irq);
#endif	
	udelay(100);
	/*reset wk2114*/
	mdelay(10);
	gpio_set_value(wk_reset, 1); 
	mdelay(10);
	gpio_set_value(wk_reset, 0); 
	mdelay(10);
	gpio_set_value(wk_reset, 1); 
	mdelay(10);
	/**put 0x55 to wk2114**/    	
	uart_putc(g_uart_port,0x55);
	udelay(10);
#ifdef DEBUG_WK2XXX
	do {
		wk2xxx_read_reg(g_uart_port,1,WK2XXX_GENA,dat);
		printk(KERN_ALERT "%s  WK2XXX_GENA:0x%x\n",__func__,dat[0]);
	}while(0);
#endif
	mutex_lock(&wk2xxxs_lock); 
	if (!uart_driver_registered) {
		uart_driver_registered = 1;
		status = uart_register_driver(&wk2xxx_uart_driver);
		if (status) {
			printk(KERN_ALERT "%s  can't register wk2xxx uart driver\n",__func__);
			mutex_unlock(&wk2xxxs_lock);
			return status;
        	}
    	}

    	for (i = 0; i < NR_PORTS; i++) {
		struct wk2xxx_port *s = &wk2xxxs[i];//container_of(port,struct wk2xxx_port,port);   
		s->tx_done       = 0;
        	s->port.line     = i;
        	s->port.ops      = &wk2xxx_pops;
        	s->port.uartclk  = WK_CRASTAL_CLK;
        	s->port.fifosize = 256;
        	s->port.iobase   = i+1;
        	s->port.irq      = wk_irq;
        	s->port.iotype   = SERIAL_IO_PORT;
        	s->port.flags    = ASYNC_BOOT_AUTOCONF;
		s->rs485_gpio	= rs485_gpio_dt(&pdev->dev, i+1);
		if (s->rs485_gpio < 0) {
			printk(KERN_ALERT "%s  rs485_gpio:%d request fail!\n",__func__,i);
			return 1;
		}
        	status = uart_add_one_port(&wk2xxx_uart_driver, &s->port);
       		if (status < 0) {
			printk(KERN_ALERT "uart_add_one_port failed for line:%d with error %d\n",i,status);
       		} else {
#ifdef DEBUG_WK2XXX
			printk(KERN_INFO "uart_add_one_port success for line:%d with right %d\n",i,status);
#endif		
		}
    	}
#ifdef DEBUG_WK2XXX
    	printk(KERN_ALERT "%s  uart_add_one_port:0x%d\n",__func__,status);
#endif
    	mutex_unlock(&wk2xxxs_lock);
#ifdef DEBUG_WK2XXX	    
	printk(KERN_ALERT "%s  exit\n", __func__);
#endif
	return 0;

err_clk:
	if (!IS_ERR(g_uart_port->clk))
		clk_disable_unprepare(g_uart_port->clk);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s  error exit\n", __func__);
#endif
	return err;
}


static int wk2xxx_remove(struct platform_device *pdev)
{
	int i;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if (wk_reset > 0) {
		gpio_free(wk_reset);
	}
	
	if (wk_irq > 0) {
		gpio_free(wk_irq);
	}
	
	mutex_lock(&wk2xxxs_lock);
	for (i = 0; i < NR_PORTS; i++) {
		struct wk2xxx_port *s = &wk2xxxs[i];
		uart_remove_one_port(&wk2xxx_uart_driver, &s->port);
	}
	uart_unregister_driver(&wk2xxx_uart_driver);
	mutex_unlock(&wk2xxxs_lock);
#ifdef DEBUG_WK2XXX	
	printk(KERN_ALERT "%s  removing wk2xxxdriver\n",__func__);
#endif	
	return 0;
}

/*
static int wk2xxx_resume(struct platform_device *pdev)
{
	#ifdef _DEBUG_WK_FUNCTION
	printk(KERN_ALERT "%s\n", __func__);
	#endif
	return 0;
}
*/

static const struct of_device_id wk2xxx_of_match[] = {
	{ .compatible = "smartchip,wk2xxx" },
	{}
};
MODULE_DEVICE_TABLE(of, wk2xxx_of_match);

static struct platform_driver wk2xxx_driver = {
	.driver = {
		.name           = "wk2xxx",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(wk2xxx_of_match),
	},
	.probe          = wk2xxx_probe,
	.remove         = wk2xxx_remove,
        //.resume         = wk2xxx_resume,
};

static int __init wk2xxx_init(void)
{
	int retval;
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n", __func__);
#endif	
	retval = platform_driver_register(&wk2xxx_driver);
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "register platform dirver return:%d\n",retval);
#endif
	return retval;
}

static void __exit wk2xxx_exit(void)
{
#ifdef DEBUG_WK2XXX
	printk(KERN_ALERT "%s\n", __func__);
#endif
	platform_driver_unregister(&wk2xxx_driver);

}

module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR("WKMIC Ltd");
MODULE_DESCRIPTION("wk2xxx generic serial port driver");
MODULE_LICENSE("GPL");

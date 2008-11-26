#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include "clock.h"
#include <mach/hardware.h>
#include <mach/uart.h>

#define UART_POLL_MODE 0

struct UARTSettings {
	u32 ureg;
	u32 baud;
	u32 sample_rate;
	int flow_control;
	int fifo;
	u32 mode;
	u32 clock;
};

struct UARTRegisters {
	u32 ULCON;
	u32 UCON;
	u32 UFCON;
	u32 UMCON;

	u32 UTRSTAT;
	u32 UERSTAT;
	u32 UFSTAT;
	u32 UMSTAT;

	u32 UTXH;
	u32 URXH;
	u32 UBAUD;
	u32 UINTP;
};

const struct UARTRegisters HWUarts[] = {
	{UART + UART0 + UART_ULCON, UART + UART0 + UART_UCON, UART + UART0 + UART_UFCON, 0,
		UART + UART0 + UART_UTRSTAT, UART + UART0 + UART_UERSTAT, UART + UART0 + UART_UFSTAT,
		0, UART + UART0 + UART_UTXH, UART + UART0 + UART_URXH, UART + UART0 + UART_UBAUD,
		UART + UART0 + UART_UINTP},
	{UART + UART1 + UART_ULCON, UART + UART1 + UART_UCON, UART + UART1 + UART_UFCON, UART + UART1 + UART_UMCON,
		UART + UART1 + UART_UTRSTAT, UART + UART1 + UART_UERSTAT, UART + UART1 + UART_UFSTAT,
		UART + UART1 + UART_UMSTAT, UART + UART1 + UART_UTXH, UART + UART1 + UART_URXH, UART + UART1 + UART_UBAUD,
		UART + UART1 + UART_UINTP},
	{UART + UART2 + UART_ULCON, UART + UART2 + UART_UCON, UART + UART2 + UART_UFCON, UART + UART2 + UART_UMCON,
		UART + UART2 + UART_UTRSTAT, UART + UART2 + UART_UERSTAT, UART + UART2 + UART_UFSTAT,
		UART + UART2 + UART_UMSTAT, UART + UART2 + UART_UTXH, UART + UART2 + UART_URXH, UART + UART2 + UART_UBAUD,
		UART + UART2 + UART_UINTP},
	{UART + UART3 + UART_ULCON, UART + UART3 + UART_UCON, UART + UART3 + UART_UFCON, UART + UART3 + UART_UMCON,
		UART + UART3 + UART_UTRSTAT, UART + UART3 + UART_UERSTAT, UART + UART3 + UART_UFSTAT,
		UART + UART3 + UART_UMSTAT, UART + UART3 + UART_UTXH, UART + UART3 + UART_URXH, UART + UART3 + UART_UBAUD,
		UART + UART3 + UART_UINTP},
	{UART + UART4 + UART_ULCON, UART + UART4 + UART_UCON, UART + UART4 + UART_UFCON, UART + UART4 + UART_UMCON,
		UART + UART4 + UART_UTRSTAT, UART + UART4 + UART_UERSTAT, UART + UART4 + UART_UFSTAT,
		UART + UART4 + UART_UMSTAT, UART + UART4 + UART_UTXH, UART + UART4 + UART_URXH, UART + UART4 + UART_UBAUD,
		UART + UART4 + UART_UINTP}};

static struct UARTSettings UARTs[5];

static int iphone_uart_set_baud_rate(int ureg, u32 baud) {
	u32 clockFrequency;
	u32 div_val;

	if(ureg > 4)
		return -1; // Invalid ureg

	if(UARTs[ureg].sample_rate == 0 || baud == 0)
		return -1;

	//u32 clockFrequency = (UARTs[ureg].clock == UART_CLOCK_PCLK) ? PeripheralFrequency : FixedFrequency;
	// FIXME: Hardwired to fixed frequency
	clockFrequency = 24000000;
	div_val = clockFrequency / (baud * UARTs[ureg].sample_rate) - 1;

	__raw_writel((__raw_readl(HWUarts[ureg].UBAUD) & (~UART_DIVVAL_MASK)) | div_val, HWUarts[ureg].UBAUD);

	// vanilla iBoot also does a reverse calculation from div_val and solves for baud and reports
	// the "actual" baud rate, or what is after loss during integer division

	UARTs[ureg].baud = baud;

	return 0;
}

static int iphone_uart_set_clk(int ureg, int clock) {
	if(ureg > 4)
		return -1; // Invalid ureg

	if(clock != UART_CLOCK_PCLK && clock != UART_CLOCK_EXT_UCLK0 && clock != UART_CLOCK_EXT_UCLK1) {
		return -1; // Invalid clock		
	}

	__raw_writel((__raw_readl(HWUarts[ureg].UCON) & (~UART_CLOCK_SELECTION_MASK)) | (clock << UART_CLOCK_SELECTION_SHIFT), HWUarts[ureg].UCON);

	UARTs[ureg].clock = clock;
	iphone_uart_set_baud_rate(ureg, UARTs[ureg].baud);

	return 0;
}

static int iphone_uart_set_sample_rate(int ureg, int rate) {
	u32 newSampleRate;

	if(ureg > 4)
		return -1; // Invalid ureg

	switch(rate) {
		case 4:
			newSampleRate = UART_SAMPLERATE_4;
			break;
		case 8:
			newSampleRate = UART_SAMPLERATE_8;
			break;
		case 16:
			newSampleRate = UART_SAMPLERATE_16;
			break;
		default:
			return -1; // Invalid sample rate
	}

	__raw_writel((__raw_readl(HWUarts[ureg].UBAUD) & (~UART_SAMPLERATE_MASK)) | (newSampleRate << UART_SAMPLERATE_SHIFT), HWUarts[ureg].UBAUD);

	UARTs[ureg].sample_rate = rate;
	iphone_uart_set_baud_rate(ureg, UARTs[ureg].baud);

	return 0;
}

static int iphone_uart_set_flow_control(int ureg, int flow_control) {
	if(ureg > 4)
		return -1; // Invalid ureg

	if(flow_control == 1) {
		if(ureg == 0)
			return -1; // uart0 does not support flow control

		__raw_writel(UART_UMCON_AFC_BIT, HWUarts[ureg].UMCON);
	} else {
		if(ureg != 0) {
			__raw_writel(UART_UMCON_NRTS_BIT, HWUarts[ureg].UMCON);
		}
	}

	UARTs[ureg].flow_control = flow_control;

	return 0;
}

static int iphone_uart_set_mode(int ureg, u32 mode) {
	if(ureg > 4)
		return -1; // Invalid ureg

	UARTs[ureg].mode = mode;

	if(mode == UART_POLL_MODE) {
		// Setup some defaults, like no loopback mode
		__raw_writel(__raw_readl(HWUarts[ureg].UCON) & (~UART_UCON_UNKMASK) & (~UART_UCON_UNKMASK) & (~UART_UCON_LOOPBACKMODE), HWUarts[ureg].UCON);

		// Use polling mode
		__raw_writel((__raw_readl(HWUarts[ureg].UCON) & (~UART_UCON_RXMODE_MASK) & (~UART_UCON_TXMODE_MASK))
			| (UART_UCON_MODE_IRQORPOLL << UART_UCON_RXMODE_SHIFT)
			| (UART_UCON_MODE_IRQORPOLL << UART_UCON_TXMODE_SHIFT), HWUarts[ureg].UCON);
	}

	return 0;
}

static int iphone_uart_set_bits(int ureg, int bits) {
	if(ureg > 4)
		return -1; // Invalid ureg

	switch(bits) {
		case 8:	
			__raw_writel(UART_8BITS, HWUarts[ureg].ULCON);
			break;
		case 7:	
			__raw_writel(UART_7BITS, HWUarts[ureg].ULCON);
			break;
		case 6:	
			__raw_writel(UART_6BITS, HWUarts[ureg].ULCON);
			break;
		case 5:	
			__raw_writel(UART_5BITS, HWUarts[ureg].ULCON);
			break;
		default:
			return -1;
	}

	return 0;
}

static int iphone_uart_setup(void) {
	int i;

	iphone_clock_gate_switch(UART_CLOCKGATE, 1);

	for(i = 0; i < NUM_UARTS; i++) {
		// set all uarts to transmit 8 bit frames, one stop bit per frame, no parity, no infrared mode
		__raw_writel(UART_8BITS, HWUarts[i].ULCON);

		// set all uarts to use polling for rx/tx, no breaks, no loopback, no error status interrupts,
		// no timeouts, pulse interrupts for rx/tx, peripheral clock. Basically, the defaults.
		__raw_writel((UART_UCON_MODE_IRQORPOLL << UART_UCON_RXMODE_SHIFT) | (UART_UCON_MODE_IRQORPOLL << UART_UCON_TXMODE_SHIFT), HWUarts[i].UCON);

		// Initialize the settings array a bit so the helper functions can be used properly
		UARTs[i].ureg = i;
		UARTs[i].baud = 115200;

		iphone_uart_set_clk(i, UART_CLOCK_EXT_UCLK0);
		iphone_uart_set_sample_rate(i, 16);
	}

	// Set flow control
	iphone_uart_set_flow_control(0, 0);
	iphone_uart_set_flow_control(1, 1);
	iphone_uart_set_flow_control(2, 1);
	iphone_uart_set_flow_control(3, 1);
	iphone_uart_set_flow_control(4, 0);

	// Reset and enable fifo
	for(i = 0; i < NUM_UARTS; i++) {
		__raw_writel(UART_FIFO_RESET_TX | UART_FIFO_RESET_RX, HWUarts[i].UFCON);
		__raw_writel(UART_FIFO_ENABLE, HWUarts[i].UFCON);
		UARTs[i].fifo = 1;
	}

	for(i = 0; i < NUM_UARTS; i++) {
		iphone_uart_set_mode(i, UART_POLL_MODE);
	}

	iphone_uart_set_mode(0, UART_POLL_MODE);

	return 0;
}


int iphone_uart_write(int ureg, const char *buffer, u32 length) {
	const struct UARTRegisters* uart;
	struct UARTSettings* settings;
	int written;

	if(ureg > 4)
		return -1; // Invalid ureg

	uart = &HWUarts[ureg];
	settings = &UARTs[ureg];

	if(settings->mode != UART_POLL_MODE)
		return -1; // unhandled uart mode

	written = 0;
	while(written < length) {
		int i;
		for(i = 0; i < 2; i++) {
			if(settings->fifo) {
				// spin until the tx fifo buffer is no longer full
				while((__raw_readl(uart->UFSTAT) & UART_UFSTAT_TXFIFO_FULL) != 0);
			} else {
				// spin while not Transmitter Empty
				while((__raw_readl(uart->UTRSTAT) & UART_UTRSTAT_TRANSMITTEREMPTY) == 0);
			}

			if(settings->flow_control) {		// only need to do this when there is flow control
				// spin while not Transmitter Empty
				while((__raw_readl(uart->UTRSTAT) & UART_UTRSTAT_TRANSMITTEREMPTY) == 0);

				// spin while not Clear To Send
				while((__raw_readl(uart->UMSTAT) & UART_UMSTAT_CTS) == 0);
			}

			if(i == 1) {
				// flush buffer
				__raw_writel(0, uart->UTXH);
				break;
			} else {
				if(*buffer == '\n') {
					__raw_writel('\r', uart->UTXH); 
					while((__raw_readl(uart->UTRSTAT) & UART_UTRSTAT_TRANSMITTEREMPTY) == 0);
				}
				__raw_writel(*buffer, uart->UTXH); 
			}
		}

		buffer++;
		written++;
	}

	return written;
}

int iphone_uart_read(int ureg, char *buffer, u32 length, u64 timeout) {
	const struct UARTRegisters* uart;
	struct UARTSettings* settings;
	u64 startTime;
	int written;
	u32 discard;
	int canRead;
	u32 ufstat;

	if(ureg > 4)
		return -1; // Invalid ureg

	uart = &HWUarts[ureg];
	settings = &UARTs[ureg];

	if(settings->mode != UART_POLL_MODE)
		return -1; // unhandled uart mode

	startTime = iphone_microtime();
	written = 0;

	while(written < length) {
		canRead = 0;
		if(settings->fifo) {
			ufstat = __raw_readl(uart->UFSTAT);
			canRead = (ufstat & UART_UFSTAT_RXFIFO_FULL) | (ufstat & UART_UFSTAT_RXCOUNT_MASK);
		} else {
			canRead = __raw_readl(uart->UTRSTAT) & UART_UTRSTAT_RECEIVEDATAREADY;
		}

		if(canRead) {
			if(__raw_readl(uart->UERSTAT)) {
				discard = __raw_readl(uart->URXH);
			} else {
				*buffer = __raw_readl(uart->URXH);
				written++;
				buffer++;
			}
		} else {
			if((iphone_microtime() - startTime) >= timeout) {
				break;
			}
		}
	}

	return written;
}

static int __init iphone_console_setup(struct console *co, char *options) {
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if(options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	iphone_uart_set_bits(0, bits);
	iphone_uart_set_baud_rate(0, baud);

	if(parity != 'n')
		return -1;

	if(flow != 'n')
		iphone_uart_set_flow_control(0, 1);
	else
		iphone_uart_set_flow_control(0, 0);

	return 0;
}

static void iphone_console_write(struct console *co, const char *s, unsigned int count) {
	iphone_uart_write(0, s, count);
}

/**
 * iphone_uart_type - What type of console are we?
 * @port: Port to operate with (we ignore since we only have one port)
 *
 */
static const char *iphone_uart_type(struct uart_port *port)
{
	return ("iPhone Serial");
}

/**
 * iphone_uart_tx_empty - Is the transmitter empty?  We pretend we're always empty
 * @port: Port to operate on (we ignore since we only have one port)
 *
 */
static unsigned int iphone_uart_tx_empty(struct uart_port *port)
{
	return 1;
}

/**
 * iphone_uart_stop_tx - stop the transmitter - no-op for us
 * @port: Port to operat eon - we ignore - no-op function
 *
 */
static void iphone_uart_stop_tx(struct uart_port *port)
{
}

/**
 * iphone_uart_release_port - Free i/o and resources for port - no-op for us
 * @port: Port to operate on - we ignore - no-op function
 *
 */
static void iphone_uart_release_port(struct uart_port *port)
{
}

/**
 * iphone_uart_enable_ms - Force modem status interrupts on - no-op for us
 * @port: Port to operate on - we ignore - no-op function
 *
 */
static void iphone_uart_enable_ms(struct uart_port *port)
{
}

/**
 * iphone_uart_shutdown - shut down the port - free irq and disable - no-op for us
 * @port: Port to shut down - we ignore
 *
 */
static void iphone_uart_shutdown(struct uart_port *port)
{
}

/**
 * iphone_uart_set_mctrl - set control lines (dtr, rts, etc) - no-op for our console
 * @port: Port to operate on - we ignore
 * @mctrl: Lines to set/unset - we ignore
 *
 */
static void iphone_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

/**
 * iphone_uart_get_mctrl - get control line info, we just return a static value
 * @port: port to operate on - we only have one port so we ignore this
 *
 */
static unsigned int iphone_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_RNG | TIOCM_DSR | TIOCM_CTS;
}

/**
 * iphone_uart_stop_rx - Stop the receiver - we ignor ethis
 * @port: Port to operate on - we ignore
 *
 */
static void iphone_uart_stop_rx(struct uart_port *port)
{
}

/**
 * iphone_uart_start_tx - Start transmitter
 * @port: Port to operate on
 *
 */
static void iphone_uart_start_tx(struct uart_port *port)
{
}

/**
 * iphone_uart_break_ctl - handle breaks - ignored by us
 * @port: Port to operate on
 * @break_state: Break state
 *
 */
static void iphone_uart_break_ctl(struct uart_port *port, int break_state)
{
}

/**
 * iphone_uart_startup - Start up the serial port - always return 0 (We're always on)
 * @port: Port to operate on
 *
 */
static int iphone_uart_startup(struct uart_port *port)
{
	return 0;
}

/**
 * iphone_uart_set_termios - set termios stuff - we ignore these
 * @port: port to operate on
 * @termios: New settings
 * @termios: Old
 *
 */
static void
iphone_uart_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
}

/**
 * iphone_uart_request_port - allocate resources for port - ignored by us
 * @port: port to operate on
 *
 */
static int iphone_uart_request_port(struct uart_port *port)
{
	return 0;
}

/**
 * iphone_uart_config_port - allocate resources, set up - we ignore,  we're always on
 * @port: Port to operate on
 * @flags: flags used for port setup
 *
 */
static void iphone_uart_config_port(struct uart_port *port, int flags)
{
}

static struct uart_ops iphone_uart_ops = {
	.tx_empty     = iphone_uart_tx_empty,
	.set_mctrl    = iphone_uart_set_mctrl,
	.get_mctrl    = iphone_uart_get_mctrl,
	.stop_tx      = iphone_uart_stop_tx,
	.start_tx     = iphone_uart_start_tx,
	.stop_rx      = iphone_uart_stop_rx,
	.enable_ms    = iphone_uart_enable_ms,
	.break_ctl    = iphone_uart_break_ctl,
	.startup      = iphone_uart_startup,
	.shutdown     = iphone_uart_shutdown,
	.type         = iphone_uart_type,
	.release_port = iphone_uart_release_port,
	.request_port = iphone_uart_request_port,
	.config_port  = iphone_uart_config_port,
	.verify_port  = NULL,
	.set_termios  = iphone_uart_set_termios,
};

struct uart_driver iphone_reg = {
	.owner        = THIS_MODULE,
	.driver_name  = "iphone_serial",
	.dev_name     = "ttyS",
	.major        = TTY_MAJOR,
	.minor        = 64,
	.nr           = 1,
};

static struct console iphone_console = {
	.name     = "ttyS",
	.write    = iphone_console_write,
	.device   = uart_console_device,
	.setup    = iphone_console_setup,
	.flags    = CON_PRINTBUFFER,
	.index    = -1,
};

static struct uart_port iphone_uart_port;

static int iphone_console_initconsole(void)
{
	iphone_uart_setup();
	iphone_console.data = &iphone_reg;
	register_console(&iphone_console);
	return 0;
}

static int __init iphone_console_moduleinit(void) {
	uart_register_driver(&iphone_reg);

	spin_lock_init(&iphone_uart_port.lock);

	/* Setup the port struct with the minimum needed */
	iphone_uart_port.membase = (char *)1;	/* just needs to be non-zero */
	iphone_uart_port.type = PORT_16550A;
	iphone_uart_port.fifosize = 10;
	iphone_uart_port.ops = &iphone_uart_ops;
	iphone_uart_port.line = 0;

	uart_add_one_port(&iphone_reg, &iphone_uart_port);
	return 0;
}

console_initcall(iphone_console_initconsole);

module_init(iphone_console_moduleinit);

/*
 * drivers/serial/tegra_hsuart.c
 *
 * High-speed serial driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

// #define DEBUG           1

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/termios.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
#include <linux/tty_flip.h>
#include <linux/tegra_uart.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include <mach/clk.h>

#define TX_FORCE_PIO			0
#define RX_FORCE_PIO			0

#define TEGRA_UART_TYPE			"TEGRA_UART"

#define RX_DMA_BUFFER_SIZE		4096
#define UART_LSR_FIFOE			0x80
#define UART_IER_EORD			0x20
#define UART_MCR_RTS_EN			0x40
#define UART_MCR_CTS_EN			0x20
#define UART_LSR_ANY			(UART_LSR_OE | UART_LSR_BI | \
					 UART_LSR_PE | UART_LSR_FE)
#define TX_EMPTY_STATUS			(UART_LSR_TEMT | UART_LSR_THRE)

#define TX_PIO				BIT(1)
#define TX_DMA				BIT(2)
#define TX_STOP				BIT(3)
#define RX_PIO				BIT(4)
#define RX_DMA				BIT(5)

#define TX_ACTIVE			(TX_PIO | TX_DMA)
#define RX_ACTIVE			(RX_PIO | RX_DMA)

#define TEGRA_UART_MIN_DMA		16
#define TEGRA_UART_FIFO_SIZE		32

/*
 * Tx fifo trigger level setting in tegra uart is in
 * reverse way then conventional uart.
 */
#define TEGRA_UART_TX_TRIG_16B		0x00
#define TEGRA_UART_TX_TRIG_8B		0x10
#define TEGRA_UART_TX_TRIG_4B		0x20
#define TEGRA_UART_TX_TRIG_1B		0x30

#define TEGRA_SERIAL_NAME		"ttyHS"
#define TEGRA_SERIAL_CONSOLE		0
#define CONFIG_SERIAL_TEGRA_UARTS	5

struct tegra_uart_port {
	struct uart_port	uport;
	char			port_name[16];

	/* Module info */
	struct clk		*clk;
	unsigned int		baud;

	/* Register shadow */
	unsigned long		fcr_shadow;
	unsigned long		mcr_shadow;
	unsigned long		lcr_shadow;
	unsigned long		ier_shadow;
	bool			rts_active;

	/* TX */
	struct dma_async_tx_descriptor *tx_dma_desc;
	struct dma_chan		*tx_dma_chan;
	dma_addr_t		tx_dma_addr;
	dma_cookie_t		tx_cookie;
	u32			tx_dma_bytes;
	u32			tx_pio_bytes;

	/* RX */
	struct dma_async_tx_descriptor *rx_dma_desc;
	struct dma_chan		*rx_dma_chan;
	dma_addr_t		rx_dma_addr;
	dma_cookie_t		rx_cookie;
	void			*rx_buffer;

	int			dma_req_sel;

	int			transfer_state;
	unsigned long		irq_saved_flags;

	/* optional callback to exit low power mode */
	void (*exit_lpm_cb)(struct uart_port *);
	/* optional callback to indicate rx is done */
	void (*rx_done_cb)(struct uart_port *);

};

static struct uart_driver tegra_uart_drv = {
	.owner		= THIS_MODULE,
	.driver_name	= "tegra_uart",
	.dev_name	= TEGRA_SERIAL_NAME,
	.cons		= TEGRA_SERIAL_CONSOLE,
	.nr		= CONFIG_SERIAL_TEGRA_UARTS,
};

static void tegra_tx_dma_complete_callback(void *args);
static void tegra_rx_dma_complete_callback(void *args);

static void tegra_uart_write(struct tegra_uart_port *t, u32 val, u32 reg)
{
	struct uart_port *u = &t->uport;

	writel(val, u->membase + (reg << u->regshift));
}

static u32 tegra_uart_read(struct tegra_uart_port *t, u32 reg)
{
	struct uart_port *u = &t->uport;

	return readl(u->membase + (reg << u->regshift));
}

/* Wait for a symbol-time. */
static void wait_sym_time(struct tegra_uart_port *t, unsigned int syms)
{
	/* Definitely have a start bit. */
	unsigned int bits = 1;
	switch (t->lcr_shadow & 3) {
	case UART_LCR_WLEN5:
		bits += 5;
		break;
	case UART_LCR_WLEN6:
		bits += 6;
		break;
	case UART_LCR_WLEN7:
		bits += 7;
		break;
	default:
		bits += 8;
		break;
	}

	/* Technically 5 bits gets 1.5 bits of stop... */
	if (t->lcr_shadow & UART_LCR_STOP)
		bits += 2;
	else
		bits++;

	if (t->lcr_shadow & UART_LCR_PARITY)
		bits++;

	if (likely(t->baud))
		udelay(DIV_ROUND_UP(syms * bits * 1000000, t->baud));
}

static void tegra_fifo_reset(struct tegra_uart_port *t, u8 fcr_bits)
{
	unsigned long fcr = t->fcr_shadow;
	fcr |= fcr_bits & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	tegra_uart_write(t, fcr, UART_FCR);
	tegra_uart_read(t, UART_SCR);
	wait_sym_time(t, 1);
}

static char tegra_decode_rx_error(struct tegra_uart_port *t, u8 lsr)
{
	struct uart_port *u = &t->uport;
	char flag = TTY_NORMAL;

	if (unlikely(lsr & UART_LSR_ANY)) {
		if (lsr & UART_LSR_OE) {
			/* Overrrun error  */
			flag |= TTY_OVERRUN;
			u->icount.overrun++;
			dev_err(u->dev, "Got overrun errors\n");
		} else if (lsr & UART_LSR_PE) {
			/* Parity error */
			flag |= TTY_PARITY;
			u->icount.parity++;
			dev_err(u->dev, "Got Parity errors\n");
		} else if (lsr & UART_LSR_FE) {
			flag |= TTY_FRAME;
			u->icount.frame++;
			dev_err(u->dev, "Got frame errors\n");
		} else if (lsr & UART_LSR_BI) {
			dev_err(u->dev, "Got Break\n");
			u->icount.brk++;
			/* If FIFO read error without any data, reset Rx FIFO */
			if (!(lsr & UART_LSR_DR) && (lsr & UART_LSR_FIFOE))
				tegra_fifo_reset(t, UART_FCR_CLEAR_RCVR);
		}
	}

	return flag;
}

static void fill_tx_fifo(struct tegra_uart_port *t, u32 bytes)
{
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &u->state->xmit;

	while (bytes--) {
		BUG_ON(uart_circ_empty(xmit));
		tegra_uart_write(t, xmit->buf[xmit->tail], UART_TX);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		u->icount.tx++;
	}
}

static void tegra_start_pio_tx(struct tegra_uart_port *t, u32 bytes)
{
	if (bytes > TEGRA_UART_MIN_DMA)
		bytes = TEGRA_UART_MIN_DMA;

	t->transfer_state |= TX_PIO;
	t->tx_pio_bytes = bytes;
	t->ier_shadow |= UART_IER_THRI;
	tegra_uart_write(t, t->ier_shadow, UART_IER);
}

static void tegra_start_dma_tx(struct tegra_uart_port *t, u32 bytes)
{
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &u->state->xmit;
	dma_addr_t tx_phys_addr = t->tx_dma_addr + xmit->tail;

	t->tx_dma_bytes = bytes & ~(0xF);
	t->tx_dma_desc = dmaengine_prep_slave_single(t->tx_dma_chan,
					tx_phys_addr, t->tx_dma_bytes,
					DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!t->tx_dma_desc) {
		dev_err(u->dev, "tx dma slave prepare failed\n");
		return;
	}

	dma_sync_single_for_device(u->dev, t->tx_dma_addr,
				   UART_XMIT_SIZE, DMA_TO_DEVICE);

	t->transfer_state |= TX_DMA;
	t->tx_dma_desc->callback = tegra_tx_dma_complete_callback;
	t->tx_dma_desc->callback_param = t;
	t->tx_cookie = dmaengine_submit(t->tx_dma_desc);
	dma_async_issue_pending(t->tx_dma_chan);
}

#define BYTES_TO_ALIGN(x)	((unsigned long)(x) & 0x3)
static void tegra_start_next_tx(struct tegra_uart_port *t,
				struct circ_buf *xmit)
{
	u32 tail = (u32)&xmit->buf[xmit->tail];
	u32 count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (!count || (t->transfer_state & (TX_ACTIVE | TX_STOP)))
		return;

	if (!t->tx_dma_chan || count < TEGRA_UART_MIN_DMA)
		tegra_start_pio_tx(t, count);
	else if (BYTES_TO_ALIGN(tail) > 0)
		tegra_start_pio_tx(t, BYTES_TO_ALIGN(tail));
	else
		tegra_start_dma_tx(t, count);
}

static void tegra_tx_dma_complete_callback(void *args)
{
	struct tegra_uart_port *t = args;
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &u->state->xmit;
	struct dma_tx_state state;
	u32 bytes_transferred;

	dmaengine_tx_status(t->tx_dma_chan, t->tx_cookie, &state);
	bytes_transferred = t->tx_dma_bytes - state.residue;
	
	async_tx_ack(t->tx_dma_desc);

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	xmit->tail = (xmit->tail + bytes_transferred) & (UART_XMIT_SIZE - 1);
	t->transfer_state &= ~TX_DMA;
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
		uart_write_wakeup(&t->uport);
		spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	}
	tegra_start_next_tx(t, xmit);
	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);

	dev_dbg(u->dev, "%s transferred: %d of %d\n",
		__func__, bytes_transferred, t->tx_dma_bytes);
}

static void tegra_start_tx(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	struct circ_buf *xmit = &t->uport.state->xmit;

	tegra_start_next_tx(t, xmit);
}

static int tegra_start_dma_rx(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;

	t->rx_dma_desc = dmaengine_prep_slave_single(t->rx_dma_chan,
					t->rx_dma_addr, RX_DMA_BUFFER_SIZE,
					DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!t->rx_dma_desc) {
		dev_err(u->dev, "rx dma slave prepare failed\n");
		return -ENOMEM;
	}

	dma_sync_single_for_device(u->dev, t->rx_dma_addr,
				   RX_DMA_BUFFER_SIZE, DMA_TO_DEVICE);

	t->transfer_state |= RX_DMA;
	t->rx_dma_desc->callback = tegra_rx_dma_complete_callback;
	t->rx_dma_desc->callback_param = t;
	t->rx_cookie = dmaengine_submit(t->rx_dma_desc);
	dma_async_issue_pending(t->rx_dma_chan);

	return 0;
}

static void tegra_uart_copy_rx_to_tty(struct tegra_uart_port *t,
				      struct tty_struct *tty, int count)
{
	struct uart_port *u = &t->uport;
	int copied;

	u->icount.rx += count;
	if (!tty) {
		dev_err(u->dev, "No tty port\n");
		return;
	}

	dma_sync_single_for_cpu(u->dev, t->rx_dma_addr,
				RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);

	copied = tty_insert_flip_string(tty,
			((unsigned char *)(t->rx_buffer)), count);
	if (copied != count) {
		WARN_ON(1);
		dev_err(u->dev, "RxData copy to tty layer failed\n");
	}

	dma_sync_single_for_device(u->dev, t->rx_dma_addr,
				   RX_DMA_BUFFER_SIZE, DMA_TO_DEVICE);
}

static void tegra_handle_rx_pio(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	struct tty_struct *tty = tty_port_tty_get(&u->state->port);

	do {
		char flag = TTY_NORMAL;
		unsigned long lsr = 0;
		unsigned char ch;

		lsr = tegra_uart_read(t, UART_LSR);
		if (!(lsr & UART_LSR_DR))
			break;

		flag =  tegra_decode_rx_error(t, lsr);
		ch = (unsigned char) tegra_uart_read(t, UART_RX);
		u->icount.rx++;

		if (!uart_handle_sysrq_char(&t->uport, ch) && tty)
			tty_insert_flip_char(tty, ch, flag);
	} while (1);

	if (tty)
		tty_flip_buffer_push(tty);
	tty_kref_put(tty);

	if (t->rx_done_cb)
		t->rx_done_cb(u);
}

static void set_rts(struct tegra_uart_port *t, bool active)
{
	unsigned long mcr = t->mcr_shadow;

	if (active)
		mcr |= UART_MCR_RTS_EN;
	else
		mcr &= ~UART_MCR_RTS_EN;

	if (mcr != t->mcr_shadow) {
		tegra_uart_write(t, mcr, UART_MCR);
		t->mcr_shadow = mcr;
	}
}

static void tegra_handle_rx_dma(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	struct tty_struct *tty = tty_port_tty_get(&u->state->port);
	struct dma_tx_state state;
	bool stopped = !(t->transfer_state & RX_DMA);
	u32 bytes_transferred;

	if (t->rts_active && !stopped)
		set_rts(t, false);

	async_tx_ack(t->rx_dma_desc);
	dmaengine_tx_status(t->rx_dma_chan, t->rx_cookie, &state);

	bytes_transferred = RX_DMA_BUFFER_SIZE - state.residue;
	if (bytes_transferred)
		tegra_uart_copy_rx_to_tty(t, tty, bytes_transferred);

	tegra_handle_rx_pio(t);

	dev_dbg(u->dev, "%s transferred: %d of %d\n",
		__func__, bytes_transferred, RX_DMA_BUFFER_SIZE);

	if (stopped)
		return;

	tegra_start_dma_rx(t);

	if (t->rts_active)
		set_rts(t, true);
}

static void tegra_rx_dma_complete_callback(void *args)
{
	struct tegra_uart_port *t = args;
	struct uart_port *u = &t->uport;

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	tegra_handle_rx_dma(t);
	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
}

static void do_handle_modem_signal(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned long msr = tegra_uart_read(t, UART_MSR);

	if (msr & UART_MSR_CTS)
		dev_dbg(u->dev, "CTS triggered\n");
	if (msr & UART_MSR_DSR)
		dev_dbg(u->dev, "DSR enabled\n");
	if (msr & UART_MSR_DCD)
		dev_dbg(u->dev, "CD enabled\n");
	if (msr & UART_MSR_RI)
		dev_dbg(u->dev, "RI enabled\n");
}

static void tegra_handle_tx_pio(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &u->state->xmit;

	fill_tx_fifo(t, t->tx_pio_bytes);

	t->transfer_state &= ~TX_PIO;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
		uart_write_wakeup(&t->uport);
		spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	}

	tegra_start_next_tx(t, xmit);
}

static irqreturn_t tegra_uart_isr(int irq, void *data)
{
	struct tegra_uart_port *t = data;
	struct uart_port *u = &t->uport;
	unsigned long iir;
	unsigned long ier;
	bool is_rx_int = false;

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	while (1) {
		iir = tegra_uart_read(t, UART_IIR);
		if (iir & UART_IIR_NO_INT) {
			if (t->rx_dma_chan && is_rx_int) {
				dmaengine_terminate_all(t->rx_dma_chan);
				tegra_handle_rx_dma(t);

				if (t->transfer_state & RX_ACTIVE) {
					ier = t->ier_shadow;
					ier |= (UART_IER_RLSI | UART_IER_RTOIE |
						UART_IER_EORD);
					t->ier_shadow = ier;
					tegra_uart_write(t, ier, UART_IER);
				}
			}
			spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
			return IRQ_HANDLED;
		}

		switch ((iir >> 1) & 0x7) {
		case 0: /* Modem signal change interrupt */
			do_handle_modem_signal(u);
			break;

		case 1: /* Transmit interrupt only triggered when using PIO */
			t->ier_shadow &= ~UART_IER_THRI;
			tegra_uart_write(t, t->ier_shadow, UART_IER);
			tegra_handle_tx_pio(t);
			break;

		case 4: /* End of data */
		case 6: /* Rx timeout */
		case 2: /* Receive */
			if (t->rx_dma_chan && !is_rx_int) {
				is_rx_int = true;
				/* Disable interrups */
				ier = t->ier_shadow;
				ier |= UART_IER_RDI;
				tegra_uart_write(t, ier, UART_IER);
				ier &= ~(UART_IER_RDI | UART_IER_RLSI |
					UART_IER_RTOIE | UART_IER_EORD);
				t->ier_shadow = ier;
				tegra_uart_write(t, ier, UART_IER);
			} else
				tegra_handle_rx_pio(t);
			break;

		case 3: /* Receive error */
			/* FIXME how to handle this? Why do we get here */
			tegra_decode_rx_error(t, tegra_uart_read(t, UART_LSR));
			break;

		case 5: /* break nothing to handle */
		case 7: /* break nothing to handle */
			break;
		}
	}
}

static void tegra_stop_rx(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned long ier;

	if (t->rts_active)
		set_rts(t, false);

	if (!(t->transfer_state & RX_ACTIVE))
		return;

	wait_sym_time(t, 1); /* wait a character interval */

	ier = t->ier_shadow;
	ier &= ~(UART_IER_RDI | UART_IER_RLSI | UART_IER_RTOIE | UART_IER_EORD);
	t->ier_shadow = ier;
	tegra_uart_write(t, ier, UART_IER);

	t->transfer_state &= ~RX_ACTIVE;

	if (t->rx_dma_chan) {
		dmaengine_terminate_all(t->rx_dma_chan);
		tegra_handle_rx_dma(t);
	}
	else
		tegra_handle_rx_pio(t);
}

static int clk_div16_get_divider(unsigned long parent_rate,
				 unsigned long rate)
{
	s64 divider_u16 = parent_rate;

	if (!rate)
		return -EINVAL;

	divider_u16 += rate - 1;
	do_div(divider_u16, rate);

	if (divider_u16 > 0xFFFF)
		return -EINVAL;

	return divider_u16;
}

static unsigned long find_best_clock_source(struct tegra_uart_port *t,
					    unsigned long rate)
{
	struct uart_port *u = &t->uport;
	struct tegra_uart_platform_data *pdata = u->dev->platform_data;
	unsigned long parent_rate;
	unsigned long new_rate;
	unsigned long err_rate;
	unsigned long error_2perc;
	unsigned long fin_err = rate;
	unsigned long fin_rate = rate;
	int final_index = -1;
	int divider;
	int count;
	int i;

	if (!pdata->parent_clk_count)
		return fin_rate;

	error_2perc = (rate / 50);

	for (count = 0; count < pdata->parent_clk_count; ++count) {
		parent_rate = pdata->parent_clk_list[count].fixed_clk_rate;

		if (parent_rate < rate)
			continue;

		/* Get the divisor by uart controller dll/dlm */
		divider = clk_div16_get_divider(parent_rate, rate);

		/* Get the best divider around calculated value */
		if (divider > 2) {
			for (i = divider - 2; i < (divider + 2); ++i) {
				new_rate = parent_rate/i;
				err_rate = abs(new_rate - rate);
				if (err_rate < fin_err) {
					final_index = count;
					fin_err = err_rate;
					fin_rate = parent_rate;
					if (fin_err < error_2perc)
						break;
				}
			}
			if (fin_err < error_2perc)
				break;
		}
	}

	if (final_index >= 0)
		clk_set_parent(t->clk,
			       pdata->parent_clk_list[final_index].parent_clk);

	return fin_rate;
}

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud)
{
	struct uart_port *u = &t->uport;
	unsigned long rate;
	unsigned long divisor;
	unsigned long best_rate;
	unsigned int baud_actual;
	unsigned int baud_delta;
	unsigned long lcr;

	if (t->baud == baud)
		return;

	best_rate = find_best_clock_source(t, baud * 16);
	clk_set_rate(t->clk, best_rate);

	rate = clk_get_rate(t->clk);

	divisor = rate;
	do_div(divisor, 16);
	divisor += baud / 2;
	do_div(divisor, baud);

	/* The allowable baudrate error from desired baudrate is 5% */
	baud_actual = divisor ? rate / (16 * divisor) : 0;
	baud_delta = abs(baud_actual - baud);
	if (WARN_ON(baud_delta * 20 > baud)) {
		dev_err(u->dev, "Requested baud %u, actual %u\n",
				baud, baud_actual);
	}

	lcr = t->lcr_shadow;
	lcr |= UART_LCR_DLAB;
	tegra_uart_write(t, lcr, UART_LCR);

	tegra_uart_write(t, divisor & 0xFF, UART_TX);
	tegra_uart_write(t, ((divisor >> 8) & 0xFF), UART_IER);

	lcr &= ~UART_LCR_DLAB;
	tegra_uart_write(t, lcr, UART_LCR);
	/* Dummy read to ensure the write is posted */
	tegra_uart_read(t, UART_SCR);

	t->baud = baud;
	/* wait two character intervals at new rate */
	wait_sym_time(t, 2);
}

static void tegra_uart_free_dma(struct tegra_uart_port *t, int type)
{
	struct uart_port *u = &t->uport;

	if (type & RX_DMA && t->rx_dma_chan) {
		dma_release_channel(t->rx_dma_chan);
		t->rx_dma_chan = NULL;

		dma_free_coherent(u->dev, RX_DMA_BUFFER_SIZE,
				  t->rx_buffer, t->rx_dma_addr);
	}

	if (type & TX_DMA && t->tx_dma_chan) {
		dma_release_channel(t->tx_dma_chan);
		t->tx_dma_chan = NULL;

		dma_unmap_single(u->dev, t->tx_dma_addr,
				 UART_XMIT_SIZE, DMA_TO_DEVICE);
	}
}

static struct dma_chan* tegra_uart_init_dma(struct tegra_uart_port *t, int type)
{
	struct uart_port *u = &t->uport;
	struct dma_chan *dma_channel;
	struct dma_slave_config dma_config;
	dma_cap_mask_t mask;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_channel = dma_request_channel(mask, NULL, NULL);
	if (!dma_channel) {
		dev_err(u->dev, "Failed to allocate DMA channel\n");
		return NULL;
	}

	if (type == RX_DMA) {
		dma_config.slave_id = t->dma_req_sel;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.src_maxburst = 4;
		dma_config.src_addr = u->mapbase;

	} else {
		dma_config.slave_id = t->dma_req_sel;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_maxburst = 16;
		dma_config.dst_addr = u->mapbase;
	}

	ret = dmaengine_slave_config(dma_channel, &dma_config);
	if (ret < 0) {
		dev_err(u->dev, "DMA slave config failed\n");
		dma_release_channel(dma_channel);
		return NULL;
	}

	if (type == RX_DMA) {
		t->rx_buffer = dma_alloc_coherent(u->dev, RX_DMA_BUFFER_SIZE,
						  &t->rx_dma_addr, GFP_KERNEL);
		if (!t->rx_buffer) {
			dev_err(u->dev, "DMA buffers allocate failed\n");
			dma_release_channel(dma_channel);
			return NULL;
		}
	} else
		t->tx_dma_addr = dma_map_single(u->dev, u->state->xmit.buf,
						UART_XMIT_SIZE, DMA_TO_DEVICE);

	return dma_channel;
}

static int tegra_startup(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	int ret;

	t->fcr_shadow = 0;
	t->mcr_shadow = 0;
	t->lcr_shadow = 0;
	t->ier_shadow = 0;
	t->baud = 0;

	if (!TX_FORCE_PIO && !t->tx_dma_chan)
		t->tx_dma_chan = tegra_uart_init_dma(t, TX_DMA);

	if (!RX_FORCE_PIO && !t->rx_dma_chan)
		t->rx_dma_chan = tegra_uart_init_dma(t, RX_DMA);

	sprintf(t->port_name, "tegra_uart_%d", u->line);

	ret = request_irq(u->irq, tegra_uart_isr, IRQF_DISABLED,
			  t->port_name, t);
	if (ret) {
		dev_err(u->dev, "Failed to request irq\n");
		return ret;
	}

	clk_prepare_enable(t->clk);

	/* Reset the UART controller to clear all previous status.*/
	tegra_periph_reset_assert(t->clk);
	udelay(100);
	tegra_periph_reset_deassert(t->clk);
	udelay(100);

	/*
	 * Set the trigger level
	 *
	 * For PIO mode:
	 *
	 * For receive, this will interrupt the CPU after that many number of
	 * bytes are received, for the remaining bytes the receive timeout
	 * interrupt is received. Rx high watermark is set to 4.
	 *
	 * For transmit, if the trasnmit interrupt is enabled, this will
	 * interrupt the CPU when the number of entries in the FIFO reaches the
	 * low watermark. Tx low watermark is set to 16 bytes.
	 *
	 * For DMA mode:
	 *
	 * Set the Tx trigger to 16. This should match the DMA burst size that
	 * programmed in the DMA registers.
	 */
	t->fcr_shadow = UART_FCR_ENABLE_FIFO;
	t->fcr_shadow |= UART_FCR_R_TRIG_01;
	t->fcr_shadow |= TEGRA_UART_TX_TRIG_16B;
	tegra_uart_write(t, t->fcr_shadow, UART_FCR);

	if (t->rx_dma_chan) {
		/*
		 * Initialize the UART with default configuration
		 * (115200, N, 8, 1) so that the receive DMA buffer may be
		 * enqueued
		 */
		t->lcr_shadow = UART_LCR_WLEN8;
		tegra_set_baudrate(t, 115200);
		t->fcr_shadow |= UART_FCR_DMA_SELECT;
		tegra_uart_write(t, t->fcr_shadow, UART_FCR);
		if (tegra_start_dma_rx(t)) {
			dev_err(u->dev, "Rx DMA enqueue failed\n");
			/* fallback to pio */
			tegra_uart_free_dma(t, RX_DMA);
			t->fcr_shadow &= ~UART_FCR_DMA_SELECT;
		}
	}

	if (!t->rx_dma_chan) {
		tegra_uart_write(t, t->fcr_shadow, UART_FCR);
		t->transfer_state |= RX_PIO;
	}

	/*
	 *  Enable IE_RXS for the receive status interrupts like line errros.
	 *  Enable IE_RX_TIMEOUT to get the bytes which cannot be DMA'd.
	 *
	 *  If using DMA mode, enable EORD instead of receive interrupt which
	 *  will interrupt after the UART is done with the receive instead of
	 *  the interrupt when the FIFO "threshold" is reached.
	 *
	 *  EORD is different interrupt than RX_TIMEOUT - RX_TIMEOUT occurs when
	 *  the DATA is sitting in the FIFO and couldn't be transferred to the
	 *  DMA as the DMA size alignment(4 bytes) is not met. EORD will be
	 *  triggered when there is a pause of the incomming data stream for 4
	 *  characters long.
	 *
	 *  For pauses in the data which is not aligned to 4 bytes, we get
	 *  both the EORD as well as RX_TIMEOUT - SW sees RX_TIMEOUT first
	 *  then the EORD.
	 *
	 *  Don't get confused, believe in the magic of nvidia hw... :(
	 */
	t->ier_shadow = UART_IER_RLSI | UART_IER_RTOIE;
	t->ier_shadow |= t->rx_dma_chan ? UART_IER_EORD : UART_IER_RDI;
	tegra_uart_write(t, t->ier_shadow, UART_IER);

	return 0;
}

static void tegra_shutdown(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned long char_time = DIV_ROUND_UP(10000000, t->baud);
	unsigned long fifo_empty_time = u->fifosize * char_time;
	unsigned long wait_time;
	unsigned long lsr;
	unsigned long msr;
	unsigned long mcr;

	/* Disable interrupts */
	tegra_uart_write(t, 0, UART_IER);

	lsr = tegra_uart_read(t, UART_LSR);
	if ((lsr & UART_LSR_TEMT) != UART_LSR_TEMT) {
		msr = tegra_uart_read(t, UART_MSR);
		mcr = tegra_uart_read(t, UART_MCR);
		if ((mcr & UART_MCR_CTS_EN) && (msr & UART_MSR_CTS))
			dev_err(u->dev, "%s: Tx fifo not empty and "
				"slave disabled CTS, Waiting for slave to"
				" be ready\n", __func__);

		/* Wait for Tx fifo to be empty */
		while ((lsr & UART_LSR_TEMT) != UART_LSR_TEMT) {
			wait_time = min(fifo_empty_time, 100lu);
			udelay(wait_time);
			fifo_empty_time -= wait_time;
			if (!fifo_empty_time) {
				msr = tegra_uart_read(t, UART_MSR);
				mcr = tegra_uart_read(t, UART_MCR);
				if ((mcr & UART_MCR_CTS_EN) &&
					(msr & UART_MSR_CTS))
					dev_err(u->dev, "%s: Slave is "
					"still not ready!\n", __func__);
				break;
			}
			lsr = tegra_uart_read(t, UART_LSR);
		}
	}

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);

	/* Reset the Rx and Tx FIFOs */
	tegra_fifo_reset(t, UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR);

	t->baud = 0;
	t->transfer_state = 0;

	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);

	clk_disable_unprepare(t->clk);

	tegra_uart_free_dma(t, RX_DMA | TX_DMA);
	free_irq(u->irq, t);
}

static unsigned int tegra_get_mctrl(struct uart_port *u)
{
	return TIOCM_CTS;
}

static void set_dtr(struct tegra_uart_port *t, bool active)
{
	unsigned long mcr = t->mcr_shadow;
	
	if (active)
		mcr |= UART_MCR_DTR;
	else
		mcr &= ~UART_MCR_DTR;

	if (mcr != t->mcr_shadow) {
		tegra_uart_write(t, mcr, UART_MCR);
		t->mcr_shadow = mcr;
	}
}

static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl)
{
	struct tegra_uart_port *t = u->private_data;
	int dtr_enable;

	t->rts_active = !!(mctrl & TIOCM_RTS);
	set_rts(t, t->rts_active);

	dtr_enable = !!(mctrl & TIOCM_DTR);
	set_dtr(t, dtr_enable);
}

static void tegra_break_ctl(struct uart_port *u, int break_ctl)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned long lcr;

	lcr = t->lcr_shadow;
	if (break_ctl)
		lcr |= UART_LCR_SBC;
	else
		lcr &= ~UART_LCR_SBC;
	tegra_uart_write(t, lcr, UART_LCR);
	t->lcr_shadow = lcr;
}

static int tegra_request_port(struct uart_port *u)
{
	return 0;
}

static void tegra_release_port(struct uart_port *u)
{
	/* Nothing to do here */
}

static unsigned int tegra_tx_empty(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned int ret = 0;
	unsigned long lsr;

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);
	if (!(t->transfer_state & TX_ACTIVE)) {
		lsr = tegra_uart_read(t, UART_LSR);
		if ((lsr & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
			ret = TIOCSER_TEMT;
	}
	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);

	return ret;
}

static void tegra_stop_tx(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;
	struct circ_buf *xmit;
	struct dma_tx_state state;
	int count;

	if (t->tx_dma_chan && t->transfer_state & TX_DMA) {
		dmaengine_terminate_all(t->tx_dma_chan);
		dmaengine_tx_status(t->tx_dma_chan, t->tx_cookie, &state);
		count = t->tx_dma_bytes - state.residue;
		async_tx_ack(t->tx_dma_desc);
		xmit = &u->state->xmit;
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	}

	t->transfer_state = TX_STOP;
}

static void tegra_set_termios(struct uart_port *u, struct ktermios *termios,
			      struct ktermios *oldtermios)
{
	struct tegra_uart_port *t = u->private_data;
	unsigned int baud;
	unsigned int lcr;

	spin_lock_irqsave(&u->lock, t->irq_saved_flags);

	/* Changing configuration, it is safe to stop any rx now */
	if (t->rts_active)
		set_rts(t, false);

	/* Clear all interrupts as configuration is going to be change */
	tegra_uart_write(t, t->ier_shadow | UART_IER_RDI, UART_IER);
	tegra_uart_read(t, UART_IER);
	tegra_uart_write(t, 0, UART_IER);
	tegra_uart_read(t, UART_IER);

	/* Parity */
	lcr = t->lcr_shadow;
	lcr &= ~UART_LCR_PARITY;

	/* CMSPAR isn't supported by this driver */
	termios->c_cflag &= ~CMSPAR;
	
	if ((termios->c_cflag & PARENB) == PARENB) {
		if (termios->c_cflag & PARODD) {
			lcr |= UART_LCR_PARITY;
			lcr &= ~UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		} else {
			lcr |= UART_LCR_PARITY;
			lcr |= UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		}
	}

	lcr &= ~UART_LCR_WLEN8;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr |= UART_LCR_WLEN5;
		break;
	case CS6:
		lcr |= UART_LCR_WLEN6;
		break;
	case CS7:
		lcr |= UART_LCR_WLEN7;
		break;
	default:
		lcr |= UART_LCR_WLEN8;
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	else
		lcr &= ~UART_LCR_STOP;

	tegra_uart_write(t, lcr, UART_LCR);
	t->lcr_shadow = lcr;

	/* Baud rate. */
	baud = uart_get_baud_rate(u, termios, oldtermios, 200, 4000000);
	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
	tegra_set_baudrate(t, baud);
	spin_lock_irqsave(&u->lock, t->irq_saved_flags);

	/* Flow control */
	if (termios->c_cflag & CRTSCTS)	{
		t->mcr_shadow |= UART_MCR_CTS_EN;
		t->mcr_shadow &= ~UART_MCR_RTS_EN;
		tegra_uart_write(t, t->mcr_shadow, UART_MCR);
		/* if top layer has asked to set rts active then do so here */
		if (t->rts_active)
			set_rts(t, true);
	} else {
		t->mcr_shadow &= ~UART_MCR_CTS_EN;
		t->mcr_shadow &= ~UART_MCR_RTS_EN;
		tegra_uart_write(t, t->mcr_shadow, UART_MCR);
	}

	/* update the port timeout based on new settings */
	uart_update_timeout(u, termios->c_cflag, baud);

	/* Make sure all write has completed */
	tegra_uart_read(t, UART_IER);

	/* Reenable interrupt */
	tegra_uart_write(t, t->ier_shadow, UART_IER);
	tegra_uart_read(t, UART_IER);

	spin_unlock_irqrestore(&u->lock, t->irq_saved_flags);
}

/*
 * Flush any TX data submitted for DMA and PIO. Called when the
 * TX circular buffer is reset.
 */
static void tegra_flush_buffer(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;

	t->tx_pio_bytes = 0;
	if (t->tx_dma_chan)
		dmaengine_terminate_all(t->tx_dma_chan);
}

static const char *tegra_type(struct uart_port *u)
{
	return TEGRA_UART_TYPE;
}

static void tegra_config_port(struct uart_port *u, int flags)
{
	/* stub */
}

static void tegra_enable_ms(struct uart_port *u)
{
	/* stub */
}

static void tegra_wake_peer(struct uart_port *u)
{
	struct tegra_uart_port *t = u->private_data;

	if (t->exit_lpm_cb)
		t->exit_lpm_cb(u);
}

static struct uart_ops tegra_uart_ops = {
	.tx_empty	= tegra_tx_empty,
	.set_mctrl	= tegra_set_mctrl,
	.get_mctrl	= tegra_get_mctrl,
	.stop_tx	= tegra_stop_tx,
	.start_tx	= tegra_start_tx,
	.stop_rx	= tegra_stop_rx,
	.flush_buffer	= tegra_flush_buffer,
	.break_ctl	= tegra_break_ctl,
	.startup	= tegra_startup,
	.shutdown	= tegra_shutdown,
	.set_termios	= tegra_set_termios,
	.type		= tegra_type,
	.request_port	= tegra_request_port,
	.release_port	= tegra_release_port,
	.config_port	= tegra_config_port,
	.enable_ms	= tegra_enable_ms,
	.wake_peer	= tegra_wake_peer,
};

static struct tegra_uart_platform_data *tegra_uart_parse_dt(
		struct platform_device *pdev, struct tegra_uart_port *tup)
{
	struct tegra_uart_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	u32 of_dma[2];
	int port;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!of_property_read_u32_array(np, "nvidia,dma-request-selector", 
								of_dma, 2))
		pdata->dma_req_sel = of_dma[1];
	else {
		dev_err(&pdev->dev, "missing dma requestor in device tree\n");
		return NULL;
	}

	port = of_alias_get_id(np, "serial");
	if (port < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", port);
		return NULL;
	}
	pdata->line = port;

	return pdata;
}

static int tegra_uart_probe(struct platform_device *pdev)
{
	struct tegra_uart_port *t;
	struct uart_port *u;
	struct tegra_uart_platform_data *pdata = pdev->dev.platform_data;
	struct resource *resource;
	int ret;

	if (!pdata && pdev->dev.of_node)
		pdata = tegra_uart_parse_dt(pdev, t);

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -ENODEV;
	}

	t = devm_kzalloc(&pdev->dev, sizeof(*t), GFP_KERNEL);
	if (!t) {
		dev_err(&pdev->dev, "Failed to allocate memory for t\n");
		return -ENOMEM;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev, "No IO memory resource\n");
		return -ENODEV;
	}

	t->clk = devm_clk_get(&pdev->dev, "uart-clk");
	t->rx_done_cb = pdata->rx_done_cb;
	t->exit_lpm_cb = pdata->exit_lpm_cb;
	t->dma_req_sel = pdata->dma_req_sel;

	u = &t->uport;
	u->dev = &pdev->dev;
	u->ops = &tegra_uart_ops;
	u->line = pdata->line;
	u->type = PORT_TEGRA;
	u->iotype = UPIO_MEM32;
	u->fifosize = TEGRA_UART_FIFO_SIZE;
	u->regshift = 2;
	u->mapbase = resource->start;
	u->membase = devm_request_and_ioremap(&pdev->dev, resource);
	u->irq = platform_get_irq(pdev, 0);
	u->private_data = t;

	platform_set_drvdata(pdev, u);

	if (!u->membase) {
		dev_err(&pdev->dev, "memregion/iomap address req failed\n");
		return -EADDRNOTAVAIL;
	}

	if (IS_ERR(t->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return PTR_ERR(t->clk);
	}

	ret = uart_add_one_port(&tegra_uart_drv, u);
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to add uart port, err %d\n", ret);

	return ret;
}

static int tegra_uart_remove(struct platform_device *pdev)
{
	struct uart_port *u = platform_get_drvdata(pdev);

	uart_remove_one_port(&tegra_uart_drv, u);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_uart_suspend(struct device *dev)
{
	struct uart_port *u = dev_get_drvdata(dev);

	return uart_suspend_port(&tegra_uart_drv, u);
}

static int tegra_uart_resume(struct device *dev)
{
	struct uart_port *u = dev_get_drvdata(dev);

	return uart_resume_port(&tegra_uart_drv, u);
}
#endif

static SIMPLE_DEV_PM_OPS(tegra_uart_pm_ops,
			 tegra_uart_suspend, tegra_uart_resume);

static struct of_device_id tegra_uart_of_match[] = {
	{ .compatible	= "nvidia,tegra20-hsuart", },
	{ },
}
MODULE_DEVICE_TABLE(of, tegra_uart_of_match);

static struct platform_driver tegra_hs_serial_driver = {
	.probe		= tegra_uart_probe,
	.remove		= tegra_uart_remove,
	.driver		= {
		.name	= "tegra_uart",
		.owner	= THIS_MODULE,
		.pm	= &tegra_uart_pm_ops,
		.of_match_table = of_match_ptr(tegra_uart_of_match),
	},
};

static int __init tegra_uart_modinit(void)
{
	int ret;

	ret = uart_register_driver(&tegra_uart_drv);
	if (ret < 0) {
		pr_err("Failed to register Tegra UART driver\n");
		return ret;
	}

	return platform_driver_register(&tegra_hs_serial_driver);
}

static void __exit tegra_uart_modexit(void)
{
	uart_unregister_driver(&tegra_uart_drv);
}

module_init(tegra_uart_modinit);
module_exit(tegra_uart_modexit);
MODULE_DESCRIPTION("High speed UART driver for tegra chipset");

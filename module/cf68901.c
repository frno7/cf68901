// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Fredrik Noring */

#include "cf68901/macro.h"

#include "cf68901/module/assert.h"
#include "cf68901/module/cf68901.h"

#define NO_EVENT        ((struct cf68901_event) { })
#define CLK_EVENT(clk_) ((struct cf68901_event) { .clk = clk_ })

struct cf68901_timer {
	const char *name;
	uint8_t channel;
	uint8_t *data;
	uint8_t *ctrl;
	uint8_t ctrl_shift;

	struct cf68901_timer_state *state;
};

static uint64_t cycle_transform(
	uint64_t to_frequency, uint64_t from_frequcy, uint64_t cycle)
{
	const uint64_t q = cycle / from_frequcy;
	const uint64_t r = cycle % from_frequcy;

	return q * to_frequency + (r * to_frequency) / from_frequcy;
}

static uint64_t cycle_transform_align(
	uint64_t to_frequency, uint64_t from_frequcy, uint64_t cycle)
{
	const uint64_t q = cycle / from_frequcy;
	const uint64_t r = cycle % from_frequcy;

	return q * to_frequency +
		(r * to_frequency + from_frequcy - 1) / from_frequcy;
}

static struct cf68901_timer_cycle timer_from_mfp_cycle(
	const struct cf68901_frequency frequency,
	const struct cf68901_clk clk)
{
	return (struct cf68901_timer_cycle) {
		.c = cycle_transform(frequency.xtal1, frequency.clk, clk.c)
	};
}

static struct cf68901_clk mfp_from_timer_cycle_align(
	const struct cf68901_frequency frequency,
	const struct cf68901_timer_cycle timer_cycle)
{
	return (struct cf68901_clk) {
		.c = cycle_transform_align(
			frequency.clk, frequency.xtal1, timer_cycle.c)
	};
}

#define DEFINE_MFP_IR(symbol_, reg_)					\
static uint16_t mfp_##symbol_(struct cf68901_module *module)		\
{									\
	return (module->state.reg.u8[CF68901_REG_##reg_##A] << 8) |	\
		module->state.reg.u8[CF68901_REG_##reg_##B];		\
}

DEFINE_MFP_IR(ier, IER)		/* 16-bit interrupt enable register */
DEFINE_MFP_IR(ipr, IPR)		/* 16-bit interrupt pending register */
DEFINE_MFP_IR(isr, ISR)		/* 16-bit interrupt service register */
DEFINE_MFP_IR(imr, IMR)		/* 16-bit interrupt mask register */

static uint32_t mfp_ctrl_prescale(struct cf68901_module *module,
	const enum cf68901_ctrl ctrl)
{
	switch (ctrl) {
#define CF68901_CTRL_DIV_PRESCALE(div)					\
	case cf68901_ctrl_div_##div: return div;
CF68901_CTRL_DIV(CF68901_CTRL_DIV_PRESCALE)
	default:
		MODULE_BUG(module);
		return cf68901_ctrl_div_200;
	}
}

static void mfp_wr_interrupt_pending(
	struct cf68901_module *module, int i, bool pending)
{
	uint8_t *ipr = &module->state.reg.u8
		[i < 8 ? CF68901_REG_IPRB : CF68901_REG_IPRA];
	uint8_t m = 1 << (i & 7);

	if (pending)
		*ipr |= m;
	else
		*ipr &= ~m;
}

static void mfp_wr_interrupt_service(
	struct cf68901_module *module, int i, bool service)
{
	uint8_t *isr = &module->state.reg.u8
		[i < 8 ? CF68901_REG_ISRB : CF68901_REG_ISRA];
	uint8_t m = 1 << (i & 7);

	if (service)
		*isr |= m;
	else
		*isr &= ~m;
}

static bool timer_rd_interrupt_enable(
	struct cf68901_module *module, const struct cf68901_timer *timer)
{
	return (mfp_ier(module) & (1 << timer->channel)) != 0;
}

static void timer_wr_interrupt_pending(struct cf68901_module *module,
	const struct cf68901_timer *timer, bool pending)
{
	mfp_wr_interrupt_pending(module, timer->channel, pending);
}

static uint32_t timer_period(const struct cf68901_timer *timer)
{
	return *timer->data ? *timer->data : 256;
}

static uint32_t timer_ctrl(const struct cf68901_timer *timer)
{
	return (*timer->ctrl >> timer->ctrl_shift) & 7;
}

static uint32_t timer_prescale(struct cf68901_module *module,
	const struct cf68901_timer *timer)
{
	return mfp_ctrl_prescale(module, timer_ctrl(timer));
}

static uint8_t timer_counter(struct cf68901_module *module,
	const struct cf68901_timer *timer,
	const struct cf68901_timer_cycle timer_cycle)
{
	if (timer_ctrl(timer) == cf68901_ctrl_stop)
		return *timer->data;

	const struct cf68901_timer_cycle *timeout = &timer->state->timeout;

	if (!timeout->c)
		return *timer->data;

	const uint32_t prescale = timer_prescale(module, timer);

	if (timer_cycle.c < timeout->c) {
		const uint64_t remaining = timeout->c - timer_cycle.c;

		return (remaining + prescale - 1) / prescale;
	}

	const uint64_t elapsed = (timer_cycle.c - timeout->c) / prescale;
	const uint32_t period = timer_period(timer);

	return period - (elapsed % period);
}

static void timer_delay_event(struct cf68901_module *module,
	struct cf68901_event *event, const struct cf68901_timer *timer,
	const struct cf68901_timer_cycle timer_cycle)
{
	struct cf68901_timer_cycle *timeout = &timer->state->timeout;
	const bool counting = timeout->c > 0;

	if (timer_ctrl(timer) == cf68901_ctrl_stop) {
		if (counting && !timer->state->stopped.c)
			timer->state->stopped = timer_cycle;

		return;
	}

	if (counting && timer->state->stopped.c > 0) {
		timeout->c += timer_cycle.c - timer->state->stopped.c;

		timer->state->stopped.c = 0;
	}

	if (counting && timer_cycle.c < timeout->c)
		goto request_event;

	const uint32_t period = timer_period(timer);
	const uint32_t prescale = timer_prescale(module, timer);
	const uint64_t elapsed = counting ? timer_cycle.c - timeout->c : 0;

	timeout->c = timer_cycle.c +
		period * prescale - (elapsed % (period * prescale));

	/*
	 * When an interrupt is received on an enabled channel, the
	 * corresponding interrupt pending bit is set in IPRA or IPRB.
	 *
	 * In a vectored interrupt scheme, this bit will be cleared
	 * when the processor acknowledges the interrupting channel
	 * and the MFP responds with a vector number.
	 *
	 * In a polled interrupt scheme, the IPRs must be read to
	 * determine the interrupting channel, and then the interrupt
	 * pending bit is cleared by the interrupt handling routine
	 * without performing an interrupt acknowledge sequence.
	 *
	 * A disabled channel is inactive; interrupts received on the
	 * channel are ignored by the MFP.
	 */
	if (counting && timer_rd_interrupt_enable(module, timer))
		timer_wr_interrupt_pending(module, timer, true);

request_event:; /* Label followed by a declaration is a C23 extension. */
	const struct cf68901_clk e =
		mfp_from_timer_cycle_align(module->frequency, *timeout);

	if (!event->clk.c || e.c < event->clk.c)
		event->clk = e;
}

static void timer_event_count(struct cf68901_module *module,
	const struct cf68901_timer *timer, struct cf68901_tabi *tabi,
	const struct cf68901_timer_cycle timer_cycle)
{
	/*
	 * A disabled channel is completely inactive; interrupts
	 * received on the channel are ignored by the MFP.
	 */
	if (!timer_rd_interrupt_enable(module, timer))
		return;

	if (tabi->events < timer_period(timer))
		return;

	tabi->events %= timer_period(timer);

	timer_wr_interrupt_pending(module, timer, true);
}

static int assert_mfp_irq(struct cf68901_module *module)
{
	const uint16_t intr = mfp_ipr(module) & mfp_imr(module);
	const uint16_t isr = mfp_isr(module);

	for (uint16_t irq = 15, m = 0x8000; m > isr; irq--, m >>= 1)
		if (intr & m)
			return irq;

	return -1;
}

#define DEFINE_TIMER(state_, symbol_, name_, channel_, dr_, cr_, cr_shift_) \
	const struct cf68901_timer timer_##symbol_ = (struct cf68901_timer) { \
		.name = #name_,						\
		.channel = channel_,					\
		.data = &state_.reg.u8[CF68901_REG_##dr_],		\
		.ctrl = &state_.reg.u8[CF68901_REG_##cr_],		\
		.ctrl_shift = cr_shift_,				\
		.state = &state_.timer_##symbol_,			\
	}

#define DEFINE_TIMERS(state_)						\
	DEFINE_TIMER(state_, a, A, 13, TADR, TACR,  0);			\
	DEFINE_TIMER(state_, b, B,  8, TBDR, TBCR,  0);			\
	DEFINE_TIMER(state_, c, C,  5, TCDR, TCDCR, 4);			\
	DEFINE_TIMER(state_, d, D,  4, TDDR, TCDCR, 0)

#define IS_DELAY_MODE(tabcr)       (!(tabcr).event)
#define IS_EVENT_COUNT_MODE(tabcr) ( (tabcr).event && !(tabcr).ctrl)
#define IS_PULSE_WIDTH_MODE(tabcr) ( (tabcr).event &&  (tabcr).ctrl)

#define TIMER_OP_MODE(tabcr_, timer_, tabi_)				\
	if (IS_DELAY_MODE(tabcr_))					\
		timer_delay_event(module, &event, (timer_), timer_cycle); \
	else if (IS_EVENT_COUNT_MODE(tabcr_))				\
		timer_event_count(module, (timer_), (tabi_), timer_cycle); \
	else if (IS_PULSE_WIDTH_MODE(tabcr_))				\
		MODULE_WARN_ONCE(module, "Pulse width mode not implemented\n"); \
	else MODULE_BUG(module)

static struct cf68901_event mfp_event(struct cf68901_module *module,
	const struct cf68901_clk clk)
{
	const struct cf68901_timer_cycle timer_cycle =
		timer_from_mfp_cycle(module->frequency, clk);
	struct cf68901_event event = { };
	DEFINE_TIMERS(module->state);

	TIMER_OP_MODE(module->state.reg.tacr, &timer_a, &module->state.tai);
	TIMER_OP_MODE(module->state.reg.tbcr, &timer_b, &module->state.tbi);
	timer_delay_event(module, &event, &timer_c, timer_cycle);
	timer_delay_event(module, &event, &timer_d, timer_cycle);

	/*
	 * Interrupts are masked for a channel by clearing the
	 * appropriate bit in IMRA or IMRB. Even though an enabled
	 * channel is masked, the channel will recognise subsequent
	 * interrupts and set its interrupt pending bit. However, the
	 * channel is prevented from requesting interrupt service
	 * (!IRQ to the processor) as long as the mask bit for that
	 * channel is cleared.
	 *
	 * If a channel is requesting interrupt service at the time
	 * that its corresponding bit in IMRA or IMRB is cleared, the
	 * request will cease, and !IRQ will be negated unless another
	 * channel is requesting interrupt service. Later, when the
	 * mask bit is set, any pending interrupt on the channel will
	 * be processed according to the channel's assigned priority.
	 * IMRA and IMRB may be read at any time.
	 */
	event.irq = assert_mfp_irq(module) != -1;

	return event;
}

static uint32_t mfp_irq_vector(struct cf68901_module *module)
{
	const int irq = assert_mfp_irq(module);

	if (irq == -1) {
		MODULE_WARN_ONCE(module, "CF68901_INT_ACK_SPURIOUS\n");
		return CF68901_INT_ACK_SPURIOUS;
	}

	/*
	 * In-service registers ISRA and ISRB allow interrupts to
	 * be nested. A bit is set whenever an interrupt vector
	 * is passed for an interrupt channel. The bit is cleared
	 * whenever the processor writes a zero to the bit.
	 *
	 * In an M68000 vectored interrupt system, the MFP is
	 * assigned to one of seven possible interrupt levels.
	 * When an interrupt is received from the MFP, an
	 * interrupt acknowledge for that level is initiated.
	 * Once an interrupt is recognised at a particular level,
	 * interrupts at the same level or below are masked by
	 * the processor.
	 *
	 * As long as the processor's interrupt mask is unchanged,
	 * the M68000 interrupt structure prohibits nesting the
	 * interrupts at the same interrupt level. However,
	 * additional interrupt requests from the MFP can be
	 * recognised before a previous channel's interrupt
	 * service routine is finished by lowering the processor's
	 * interrupt mask to the next lower interrupt level within
	 * the interrupt handler.
	 */
	mfp_wr_interrupt_pending(module, irq, false);
	if (module->state.reg.vr.sei)
		mfp_wr_interrupt_service(module, irq, true);

	return (module->state.reg.vr.base << 4) + irq;
}

static uint8_t mfp_rd_u8(struct cf68901_module *module,
	struct cf68901_clk clk, uint8_t reg)
{
	const struct cf68901_timer_cycle timer_cycle =
		timer_from_mfp_cycle(module->frequency, clk);
	DEFINE_TIMERS(module->state);

	if (ARRAY_SIZE(module->state.reg.u8) <= reg)
		return 0;

	switch (reg) {
	case CF68901_REG_TADR: return timer_counter(module, &timer_a, timer_cycle);
	case CF68901_REG_TBDR: return timer_counter(module, &timer_b, timer_cycle);
	case CF68901_REG_TCDR: return timer_counter(module, &timer_c, timer_cycle);
	case CF68901_REG_TDDR: return timer_counter(module, &timer_d, timer_cycle);
	default:	       return module->state.reg.u8[reg];
	}
}

static void mfp_hardwire(struct cf68901_module *module)
{
	module->state.reg.vr.unused = 0;
	module->state.reg.tacr.unused = 0;
	module->state.reg.tbcr.unused = 0;
	module->state.reg.tcdcr.tc_unused = 0;
	module->state.reg.tcdcr.td_unused = 0;
	module->state.reg.ucr.unused = 0;
}

#define TIMER_RESET(symbol_, tdr_, tcr_, mode_mask_)			\
	CF68901_REG_##tdr_:						\
		if (!(module->state.reg.u8[CF68901_REG_##tcr_] & mode_mask_)) \
			module->state.timer_##symbol_ =			\
				(struct cf68901_timer_state) { }

static struct cf68901_event mfp_wr_u8(struct cf68901_module *module,
	struct cf68901_clk clk, uint8_t reg, uint8_t db)
{
	const bool sei = module->state.reg.vr.sei;

	if (ARRAY_SIZE(module->state.reg.u8) <= reg)
		goto out;

	/* FIXME: Proper read/write of GPIP */

	switch (reg) {
		/*
		 * IPRA and IPRB are readable; thus by polling IPRA and
		 * IPRB, it can be determined whether a channel has a
		 * pending interrupt. IPRA and IPRB are also writable
		 * and a pending interrupt can be cleared without
		 * going through the acknowledge sequence by writing a
		 * zero to the appropriate bit. This allows any one bit
		 * to be cleared without altering any other bits, simply
		 * by writing all ones except for the bit position to be
		 * cleared to IPRA and IPRB. Thus a fully polled
		 * interrupt scheme is possible.
		 *
		 * Note: writing a one to IPRA, IPRB has no effect on
		 * the interrupt pending register.
		 */
	case CF68901_REG_IPRA:
	case CF68901_REG_IPRB:
		db &= module->state.reg.u8[reg];
		break;

	case TIMER_RESET(a, TADR, TACR,  0x0f); break;
	case TIMER_RESET(b, TBDR, TBCR,  0x0f); break;
	case TIMER_RESET(c, TCDR, TCDCR, 0x70); break;
	case TIMER_RESET(d, TDDR, TCDCR, 0x07); break;
	}

	module->state.reg.u8[reg] = db;
	mfp_hardwire(module);

	switch (reg) {
	case CF68901_REG_IERA:
	case CF68901_REG_IERB:
		/*
		 * Writing a zero to a bit of IERA or IERB causes the
		 * corresponding bit of the IPR to be cleared, which
		 * terminates all interrupt service requests for the
		 * channel and also negates !IRQ unless interrupts are
		 * pending from other sources.
		 *
		 * Disabling a channel, however, does not affect the
		 * corresponding bit in ISRA or ISRB. Therefore, since
		 * the MFP for the Atari ST is in the software end-of-
		 * interrupt mode and an interrupt is in service when
		 * a channel is disabled, the in-service bit of that
		 * channel will remain set until cleared by software.
		 */
		module->state.reg.u8[CF68901_REG_IPRA] &=
		module->state.reg.u8[CF68901_REG_IERA];
		module->state.reg.u8[CF68901_REG_IPRB] &=
		module->state.reg.u8[CF68901_REG_IERB];
		break;

	case CF68901_REG_ISRA:
	case CF68901_REG_ISRB:
		/*
		 * S=1: Software end-of-interrupt mode and ISR enabled.
		 * S=0: Automatic end-of-interrupt mode and ISR forced low.
		 */
		if (!module->state.reg.vr.sei)
			module->state.reg.u8[CF68901_REG_ISRA] =
			module->state.reg.u8[CF68901_REG_ISRB] = 0;
		break;

	case CF68901_REG_VR:
		/*
		 * The interrupt in-service registers ISRA and ISRB
		 * are cleared if the S-bit of the vector register is
		 * cleared.
		 */
		if (sei && !module->state.reg.vr.sei)
			module->state.reg.u8[CF68901_REG_ISRA] =
			module->state.reg.u8[CF68901_REG_ISRB] = 0;
		break;
	}

	if (!module->state.reg.vr.sei)
		MODULE_BUG_ON(module, module->state.reg.u8[CF68901_REG_ISRA]
				   || module->state.reg.u8[CF68901_REG_ISRB]);

out:
	return mfp_event(module, clk);
}

#define WR_GPIP(bit_, rbab_, ier_, ipr_)				\
	case bit_:							\
		if ((reg->u8[CF68901_REG_##ier_] & (1 << rbab_)) == 0)	\
			return NO_EVENT;				\
		reg->u8[CF68901_REG_##ipr_] |= 1 << rbab_;		\
		break

static struct cf68901_event mfp_wr_gpip(struct cf68901_module *module,
	struct cf68901_clk clk, uint8_t bit, bool level)
{
	union cf68901_reg *reg = &module->state.reg;

	if ((module->port.state.gpip & (1 << bit)) == (level << bit))
		return NO_EVENT;
	module->port.state.gpip ^= 1 << bit;

	if ((reg->u8[CF68901_REG_DDR] & (1 << bit)) != 0)
		return NO_EVENT;
	if ((reg->u8[CF68901_REG_AER] & (1 << bit)) != (level << bit))
		return NO_EVENT;

	switch (bit) {
		WR_GPIP(7, 7, IERA, IPRA);
		WR_GPIP(6, 6, IERA, IPRA);
		WR_GPIP(5, 7, IERB, IPRB);
		WR_GPIP(4, 6, IERB, IPRB);
		WR_GPIP(3, 3, IERB, IPRB);
		WR_GPIP(2, 2, IERB, IPRB);
		WR_GPIP(1, 1, IERB, IPRB);
		WR_GPIP(0, 0, IERB, IPRB);
		default: MODULE_BUG(module);
	};

	return CLK_EVENT(clk);
}

#define TABI(port_level, tabi_, gpip_, tabcr_)				\
	if (level == port_level)					\
		return NO_EVENT;					\
	port_level = level;						\
	if (!module->state.reg.tabcr_.event)				\
		return NO_EVENT;					\
	if (level != module->state.reg.aer.gpip_)			\
		return NO_EVENT;					\
	tabi_.events++;							\
	return mfp_event(module, clk)

static struct cf68901_event mfp_tai(struct cf68901_module *module,
	struct cf68901_clk clk, bool level)
{
	TABI(module->port.state.tai, module->state.tai, gpip4, tacr);
}

static struct cf68901_event mfp_tbi(struct cf68901_module *module,
	struct cf68901_clk clk, bool level)
{
	TABI(module->port.state.tbi, module->state.tbi, gpip3, tbcr);
}

static void cf68901_reset_l(struct cf68901_module *module,
	struct cf68901_clk clk, bool reset_l)
{
	if (!reset_l) {
		/* FIXME: Reset is more selective */
		module->state = (struct cf68901_state) { };
	} else if (reset_l && !module->port.state.reset_l)
		module->clk = clk;

	module->port.state.reset_l = reset_l;
}

struct cf68901_module cf68901_init(uint32_t clk_frequency,
	uint32_t xtal1_frequency)
{
	BUILD_BUG_ON(sizeof(union cf68901_reg) != 24);

	return (struct cf68901_module) {
		.port = {
			.reset_l = cf68901_reset_l,

			.rd_da   = mfp_rd_u8,
			.wr_da   = mfp_wr_u8,
			.vector  = mfp_irq_vector,
			.event   = mfp_event,

			.wr_gpip = mfp_wr_gpip,

			.tai     = mfp_tai,
			.tbi     = mfp_tbi,

			.state = {
				/* RESET_L has a pull-up resistance. */
				.reset_l = 1,
			},
		},

		.frequency = {
			.clk   = clk_frequency,
			.xtal1 = xtal1_frequency,
		},
	};
};

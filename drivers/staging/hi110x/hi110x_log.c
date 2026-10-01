// SPDX-License-Identifier: GPL-2.0
/*
 * Log gate for the vendor Hi110x code.
 *
 * The vendor driver logs every step of every power transition, firmware download,
 * calibration and scan, through half a dozen macro families (ps_print_*, oal_io_print,
 * oal_print_hi11xx_log, pci_print_log, OAM, INI) that all end in printk(). On a laptop
 * that pushes everything else out of the kernel log. hi110x_compat.h routes the
 * driver's printk() and print_hex_dump() through here; unless hi110x.verbose is set,
 * only messages at KERN_ERR or more severe get through. hi110x.verbose=2 keeps the
 * kernel log at errors only and writes the rest to the ftrace buffer through
 * __ftrace_vprintk() (not the ftrace_vprintk() macro: its static format pointer lands in
 * __trace_printk_fmt, and then every module load allocates the trace_printk buffers
 * and prints the "trace_printk() being used" banner)
 * (/sys/kernel/tracing/trace): cheap enough not to change the timing of the power
 * transitions, which printing every step to the console does.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/stdarg.h>
#include <linux/trace.h>

int hi110x_verbose;

static int hi110x_verbose_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (!ret && READ_ONCE(hi110x_verbose) == 2)
		trace_printk_init_buffers();
	return ret;
}

static const struct kernel_param_ops hi110x_verbose_ops = {
	.set = hi110x_verbose_set,
	.get = param_get_int,
};
module_param_cb(verbose, &hi110x_verbose_ops, &hi110x_verbose, 0644);
MODULE_PARM_DESC(verbose, "Driver messages: 0 = errors only (default), 1 = everything, 2 = errors to the log, the rest to the trace buffer");

/* the level printk() will use: the last KERN_<level> prefix; 0 if there is none */
static int hi110x_msg_level(const char *s)
{
	int lvl = 0, c;

	while ((c = printk_get_level(s)) != 0) {
		lvl = c;
		s += 2;
	}
	return lvl;
}

static bool hi110x_level_is_error(int lvl)
{
	return lvl >= '0' + LOGLEVEL_EMERG && lvl <= '0' + LOGLEVEL_ERR;
}

bool hi110x_log_pass(const char *level)
{
	return READ_ONCE(hi110x_verbose) == 1 || hi110x_level_is_error(hi110x_msg_level(level));
}

int hi110x_printk(const char *fmt, ...)
{
	static bool last_shown;
	int verbose = READ_ONCE(hi110x_verbose);
	va_list args;
	int ret;

	va_start(args, fmt);
	if (verbose != 1) {
		char head[16];
		va_list aq;
		int lvl;

		/* several macros pass the level in a leading "%s" argument, so look at the output */
		va_copy(aq, args);
		vsnprintf(head, sizeof(head), fmt, aq);
		va_end(aq);
		lvl = hi110x_msg_level(head);
		if (lvl != 'c')
			last_shown = hi110x_level_is_error(lvl);
		if (!last_shown) {
			if (verbose == 2)
				__ftrace_vprintk(_THIS_IP_, fmt, args);
			va_end(args);
			return 0;
		}
	}
	ret = vprintk(fmt, args);
	va_end(args);
	return ret;
}

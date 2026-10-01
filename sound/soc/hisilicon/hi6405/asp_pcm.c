// SPDX-License-Identifier: GPL-2.0
/*
 * Kirin 990 ASP DMA PCM for the SLIMbus link to the Hi6405 codec, without
 * the HiFi DSP ("codec_without_hifi" configuration of the vendor kernel).
 *
 * Every SLIMbus data port carries one mono channel of 32-bit samples
 * (24 bit, left aligned) and is fed/drained by its own ASP DMA channel. Each
 * DMA channel runs a two-entry linked list over a ping-pong buffer of two
 * 20 ms chunks in the ASP-reachable HiFi carve-out. On every DMA interrupt
 * the chunk that just finished is converted from/to the interleaved ALSA
 * buffer, which lives in normal memory. Based on the vendor pcm_codec.c,
 * platform_io.c, format.c, armpc_custom.c and asp_dma.c (Huawei, GPL-2.0).
 *
 * Playback copies a period out of the ALSA buffer one chunk before the DMA
 * plays it, so the reported hw_ptr is the copy position: the application may
 * overwrite everything before it. The chunks in flight are reported as delay.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "hi6405_drv.h"
#include "slimbus.h"

/* ASP DMA registers (AP interrupt group 0) */
#define DMA_INT_STAT		0x000
#define DMA_INT_TC1		0x004
#define DMA_INT_TC2		0x008
#define DMA_INT_ERR1		0x00c
#define DMA_INT_ERR2		0x010
#define DMA_INT_ERR3		0x014
#define DMA_INT_TC1_MASK	0x018
#define DMA_INT_TC2_MASK	0x01c
#define DMA_INT_ERR1_MASK	0x020
#define DMA_INT_ERR2_MASK	0x024
#define DMA_INT_ERR3_MASK	0x028
#define DMA_INT_TC1_RAW		0x600
#define DMA_INT_TC2_RAW		0x608
#define DMA_INT_ERR1_RAW	0x610
#define DMA_INT_ERR2_RAW	0x618
#define DMA_INT_ERR3_RAW	0x620
#define DMA_CH_STAT		0x690
#define DMA_CX_LLI(c)		(0x800 + 0x40 * (c))
#define DMA_CX_BINDX(c)		(0x804 + 0x40 * (c))
#define DMA_CX_CINDX(c)		(0x808 + 0x40 * (c))
#define DMA_CX_CNT1(c)		(0x80c + 0x40 * (c))
#define DMA_CX_CNT0(c)		(0x810 + 0x40 * (c))
#define DMA_CX_SRC(c)		(0x814 + 0x40 * (c))
#define DMA_CX_DES(c)		(0x818 + 0x40 * (c))
#define DMA_CX_CONFIG(c)	(0x81c + 0x40 * (c))
#define DMA_CX_AXI_CONF(c)	(0x820 + 0x40 * (c))
#define DMA_CX_CURR_CNT0(c)	(0x704 + 0x10 * (c))
#define DMA_CX_CURR_SRC(c)	(0x708 + 0x10 * (c))
#define DMA_CX_CURR_DES(c)	(0x70c + 0x10 * (c))
#define DMA_CH_PRI		0x688
#define DMA_CTRL		0x698
#define DMA_CH_ENABLE		BIT(0)
#define DMA_LLI_LINK		0x2
#define DMA_NR_CHANNELS		16

/* SLIMbus data port FIFOs, one per port, relative to the SLIMbus base */
#define SLIMBUS_DPORT_FIFO(p)	(0x1000 + 0x40 * (p))

/*
 * Layout of the vendor PCM area in the HiFi data carve-out: DMA buffers for
 * (device, stream) = (0,0), (0,1), ... then 1 KiB of LLIs per (device, stream).
 */
#define PCM_AREA_OFFSET		(0x132000 + 0x32000)
#define PCM_DMA_BUF_SIZE	(48 * 20 * 8 * 2 * 5)
#define PCM_DMA_BUF_ALL		(PCM_DMA_BUF_SIZE * 3 * 2)
#define PCM_LLI_SIZE		1024

#define ASP_RATE		48000
#define CHUNK_MS		20
#define PERIOD_FRAMES		(ASP_RATE * CHUNK_MS / 1000)	/* 960 */
#define CHUNK_BYTES		(PERIOD_FRAMES * 4)		/* per port */
#define PERIODS_MIN		3
#define PERIODS_MAX		32
#define PLAY_CH_MAX		2
#define CAP_CH_MAX		4

struct dma_lli {
	u32 lli;
	u32 reserved[3];
	u32 a_count;
	u32 src_addr;
	u32 des_addr;
	u32 config;
};

struct asp_port {
	u8 dport;	/* SLIMbus data port */
	u8 chan;	/* ASP DMA channel */
	u32 config;	/* DMA channel config (request line = dport) */
};

/* same channel/port assignment as the vendor kernel (platform_io.c) */
static const struct asp_port play_ports[PLAY_CH_MAX] = {
	{ .dport = 0, .chan = 5, .config = 0x83322007 },	/* D1 */
	{ .dport = 1, .chan = 6, .config = 0x83322017 },	/* D2 */
};

static const struct asp_port cap_ports[CAP_CH_MAX] = {
	{ .dport = 2, .chan = 7, .config = 0x43322027 },	/* U1 */
	{ .dport = 3, .chan = 8, .config = 0x43322037 },	/* U2 */
	{ .dport = 12, .chan = 9, .config = 0x433220c7 },	/* U3 */
	{ .dport = 13, .chan = 11, .config = 0x433220d7 },	/* U4 */
};

struct asp_stream {
	struct asp_pcm *asp;
	struct snd_pcm_substream *substream;
	const struct asp_port *ports;
	unsigned int nports;
	void __iomem *buf;		/* ping-pong DMA buffer */
	u32 buf_phys;
	struct dma_lli __iomem *lli;	/* 2 per port */
	u32 lli_phys;
	unsigned int sample_bytes;	/* 2 (S16_LE) or 4 (S32_LE) */
	unsigned int pos;		/* capture: periods completed since start */
	snd_pcm_uframes_t fill;		/* playback: buffer offset of the next period to copy */
	unsigned int irq_pending;	/* channels that finished this round */
	bool running;
};

struct asp_pcm {
	struct device *dev;
	void __iomem *dma;
	phys_addr_t slimbus_phys;
	void __iomem *area;		/* PCM area of the HiFi carve-out */
	phys_addr_t area_phys;
	struct regulator *asp_supply;
	struct clk *asp_clk;
	struct mutex lock;
	spinlock_t reg_lock;
	unsigned int users;		/* open substreams (ASP powered and clocked) */
	int irq;
	struct asp_stream streams[2];
	void __iomem *cfg;		/* ASP_CFG (read-only here, for diagnostics) */
};

static void dma_update_bits(struct asp_pcm *asp, u32 reg, u32 mask, u32 val)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&asp->reg_lock, flags);
	v = readl(asp->dma + reg);
	writel((v & ~mask) | (val & mask), asp->dma + reg);
	spin_unlock_irqrestore(&asp->reg_lock, flags);
}

static void dma_irq_mask(struct asp_pcm *asp, unsigned int chan, bool enable)
{
	u32 bit = enable ? BIT(chan) : 0;

	dma_update_bits(asp, DMA_INT_TC1_MASK, BIT(chan), bit);
	dma_update_bits(asp, DMA_INT_TC2_MASK, BIT(chan), bit);
	dma_update_bits(asp, DMA_INT_ERR1_MASK, BIT(chan), bit);
	dma_update_bits(asp, DMA_INT_ERR2_MASK, BIT(chan), bit);
	dma_update_bits(asp, DMA_INT_ERR3_MASK, BIT(chan), bit);
}

static void dma_clear_irq(struct asp_pcm *asp, u32 mask)
{
	writel(mask, asp->dma + DMA_INT_TC1_RAW);
	writel(mask, asp->dma + DMA_INT_TC2_RAW);
	writel(mask, asp->dma + DMA_INT_ERR1_RAW);
	writel(mask, asp->dma + DMA_INT_ERR2_RAW);
	writel(mask, asp->dma + DMA_INT_ERR3_RAW);
}

static void dma_stop_channel(struct asp_pcm *asp, unsigned int chan)
{
	int i;

	dma_update_bits(asp, DMA_CX_CONFIG(chan), DMA_CH_ENABLE, 0);
	for (i = 0; i < 40; i++) {
		if (!(readl(asp->dma + DMA_CH_STAT) & BIT(chan)))
			break;
		udelay(250);
	}
	if (i == 40)
		dev_warn(asp->dev, "dma channel %u did not stop\n", chan);
	dma_irq_mask(asp, chan, false);
	dma_clear_irq(asp, BIT(chan));
}

static inline unsigned int stream_chunk_off(struct asp_stream *s, unsigned int half,
	unsigned int port)
{
	/* [A: port0 .. portN-1][B: port0 .. portN-1] */
	return (half * s->nports + port) * CHUNK_BYTES;
}

/* program the two-entry LLI ring of every port and load entry A */
static void asp_stream_setup_dma(struct asp_stream *s)
{
	struct asp_pcm *asp = s->asp;
	bool playback = s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int p, h;

	for (p = 0; p < s->nports; p++) {
		const struct asp_port *port = &s->ports[p];
		u32 fifo = asp->slimbus_phys + SLIMBUS_DPORT_FIFO(port->dport);
		u32 lli_phys = s->lli_phys + p * 2 * sizeof(struct dma_lli);
		struct dma_lli __iomem *lli = &s->lli[p * 2];

		for (h = 0; h < 2; h++) {
			u32 mem = s->buf_phys + stream_chunk_off(s, h, p);
			u32 next = lli_phys + ((h + 1) % 2) * sizeof(struct dma_lli);

			writel(next | DMA_LLI_LINK, &lli[h].lli);
			writel(CHUNK_BYTES, &lli[h].a_count);
			writel(playback ? mem : fifo, &lli[h].src_addr);
			writel(playback ? fifo : mem, &lli[h].des_addr);
			writel(port->config, &lli[h].config);
		}

		dma_update_bits(asp, DMA_CX_CONFIG(port->chan), DMA_CH_ENABLE, 0);
		writel(CHUNK_BYTES, asp->dma + DMA_CX_CNT0(port->chan));
		writel(0, asp->dma + DMA_CX_CNT1(port->chan));
		writel(0, asp->dma + DMA_CX_BINDX(port->chan));
		writel(0, asp->dma + DMA_CX_CINDX(port->chan));
		writel(readl(&lli[0].src_addr), asp->dma + DMA_CX_SRC(port->chan));
		writel(readl(&lli[0].des_addr), asp->dma + DMA_CX_DES(port->chan));
		writel(readl(&lli[0].lli), asp->dma + DMA_CX_LLI(port->chan));
		dma_clear_irq(asp, BIT(port->chan));
		dma_irq_mask(asp, port->chan, true);
	}
}

/* frames of the next period to copy that the application has written */
static unsigned int asp_play_written(struct asp_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	snd_pcm_uframes_t hw = rt->status->hw_ptr % rt->buffer_size;
	snd_pcm_sframes_t n;

	/* written past hw_ptr, less how far the copy position already is past it */
	n = snd_pcm_playback_hw_avail(rt) -
	    (snd_pcm_sframes_t)((s->fill + rt->buffer_size - hw) % rt->buffer_size);
	return clamp_t(snd_pcm_sframes_t, n, 0, PERIOD_FRAMES);
}

/*
 * Next ALSA period -> one chunk per port (mono, 32-bit left aligned). Frames
 * the application has not written yet play as silence rather than whatever
 * the buffer held one cycle earlier.
 */
static void asp_fill_chunk(struct asp_stream *s, unsigned int half)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	unsigned int ch = rt->channels, n = asp_play_written(s), p, i;
	u8 *src = rt->dma_area + frames_to_bytes(rt, s->fill);

	for (p = 0; p < s->nports; p++) {
		u32 __iomem *dst = s->buf + stream_chunk_off(s, half, p);

		if (s->sample_bytes == 2) {
			const s16 *in = (const s16 *)src + p;

			for (i = 0; i < n; i++, in += ch)
				writel((u32)((s32)*in << 16), dst + i);
		} else {
			const s32 *in = (const s32 *)src + p;

			for (i = 0; i < n; i++, in += ch)
				writel((u32)*in, dst + i);
		}
		for (; i < PERIOD_FRAMES; i++)
			writel(0, dst + i);
	}

	s->fill = (s->fill + PERIOD_FRAMES) % rt->buffer_size;
}

static void asp_silence_chunk(struct asp_stream *s, unsigned int half)
{
	unsigned int p;

	for (p = 0; p < s->nports; p++)
		memset_io(s->buf + stream_chunk_off(s, half, p), 0, CHUNK_BYTES);
}

/* one chunk per port -> ALSA period */
static void asp_drain_chunk(struct asp_stream *s, unsigned int half, unsigned int period)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	unsigned int ch = rt->channels, p, i;
	u8 *dst = rt->dma_area + frames_to_bytes(rt, period * PERIOD_FRAMES);

	for (p = 0; p < s->nports; p++) {
		const u32 __iomem *src = s->buf + stream_chunk_off(s, half, p);

		if (s->sample_bytes == 2) {
			s16 *out = (s16 *)dst + p;

			for (i = 0; i < PERIOD_FRAMES; i++, out += ch)
				*out = (s32)readl(src + i) >> 16;
		} else {
			s32 *out = (s32 *)dst + p;

			for (i = 0; i < PERIOD_FRAMES; i++, out += ch)
				*out = readl(src + i);
		}
	}
}

/* chunk the DMA is not working on right now */
static unsigned int asp_idle_half(struct asp_stream *s)
{
	struct asp_pcm *asp = s->asp;
	unsigned int chan = s->ports[0].chan;
	bool playback = s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 cur = readl(asp->dma + (playback ? DMA_CX_SRC(chan) : DMA_CX_DES(chan)));
	u32 a = s->buf_phys + stream_chunk_off(s, 0, 0);

	/*
	 * The channel address register points into the chunk being transferred
	 * (or at the end of the one just finished before the next LLI loads).
	 */
	return cur >= a && cur < a + CHUNK_BYTES ? 1 : 0;
}

/*
 * One 20 ms chunk finished on all ports of the stream. Called with asp->lock
 * held; the caller reports the elapsed period after dropping the lock, as
 * snd_pcm_period_elapsed() may stop the stream (xrun) and re-enter trigger.
 */
static void asp_stream_period(struct asp_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	unsigned int half = asp_idle_half(s);

	if (s->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* the idle chunk plays after the running one */
		asp_fill_chunk(s, half);
	} else {
		asp_drain_chunk(s, half, s->pos % rt->periods);
		s->pos++;
	}
}

/* what a failing channel was doing, and the LLI ring we wrote for it (asp->lock held) */
static void asp_dump_err_channels(struct asp_pcm *asp, u32 err)
{
	static DEFINE_RATELIMIT_STATE(rs, 10 * HZ, 4);
	int i, p;

	if (!__ratelimit(&rs))
		return;
	dev_err(asp->dev, "dma: ch_stat %#x ch_pri %#x ctrl %#x\n", readl(asp->dma + DMA_CH_STAT),
		readl(asp->dma + DMA_CH_PRI), readl(asp->dma + DMA_CTRL));
	for (i = 0; i < ARRAY_SIZE(asp->streams); i++) {
		struct asp_stream *s = &asp->streams[i];

		for (p = 0; p < s->nports && s->ports; p++) {
			unsigned int c = s->ports[p].chan;
			struct dma_lli __iomem *l = &s->lli[p * 2];

			if (!(err & BIT(c)))
				continue;
			dev_err(asp->dev,
				"dma ch%u: config %#x axi %#x lli %#x src %#x des %#x cnt0 %#x cur src %#x des %#x cnt0 %#x; ring A %#x/%#x/%#x B %#x/%#x/%#x\n",
				c, readl(asp->dma + DMA_CX_CONFIG(c)), readl(asp->dma + DMA_CX_AXI_CONF(c)),
				readl(asp->dma + DMA_CX_LLI(c)), readl(asp->dma + DMA_CX_SRC(c)),
				readl(asp->dma + DMA_CX_DES(c)), readl(asp->dma + DMA_CX_CNT0(c)),
				readl(asp->dma + DMA_CX_CURR_SRC(c)), readl(asp->dma + DMA_CX_CURR_DES(c)),
				readl(asp->dma + DMA_CX_CURR_CNT0(c)),
				readl(&l[0].lli), readl(&l[0].src_addr), readl(&l[0].des_addr),
				readl(&l[1].lli), readl(&l[1].src_addr), readl(&l[1].des_addr));
		}
	}
}

static irqreturn_t asp_dma_irq(int irq, void *data)
{
	struct asp_pcm *asp = data;
	struct snd_pcm_substream *elapsed[ARRAY_SIZE(asp->streams)] = { };
	u32 stat, tc, err;
	int i, p;

	/* the DMA block is only clocked while a substream is open */
	mutex_lock(&asp->lock);
	if (!asp->users) {
		mutex_unlock(&asp->lock);
		return IRQ_NONE;
	}
	stat = readl(asp->dma + DMA_INT_STAT);
	if (!stat) {
		mutex_unlock(&asp->lock);
		return IRQ_NONE;
	}
	tc = readl(asp->dma + DMA_INT_TC1) | readl(asp->dma + DMA_INT_TC2);
	err = readl(asp->dma + DMA_INT_ERR1) | readl(asp->dma + DMA_INT_ERR2) |
	      readl(asp->dma + DMA_INT_ERR3);
	if (err) {
		u32 e1 = readl(asp->dma + DMA_INT_ERR1), e2 = readl(asp->dma + DMA_INT_ERR2);
		u32 e3 = readl(asp->dma + DMA_INT_ERR3);

		dev_err_ratelimited(asp->dev, "dma error, channels %#x (err1 %#x err2 %#x err3 %#x)\n",
			err, e1, e2, e3);
		asp_dump_err_channels(asp, err);
	}
	dma_clear_irq(asp, stat);

	for (i = 0; i < ARRAY_SIZE(asp->streams); i++) {
		struct asp_stream *s = &asp->streams[i];
		u32 all = 0;

		if (!s->running)
			continue;
		for (p = 0; p < s->nports; p++) {
			all |= BIT(s->ports[p].chan);
			if (tc & BIT(s->ports[p].chan))
				s->irq_pending |= BIT(s->ports[p].chan);
		}
		/* the ports of a stream run in lock step: act once all are done */
		if (s->irq_pending == all) {
			s->irq_pending = 0;
			asp_stream_period(s);
			elapsed[i] = s->substream;
		}
	}
	mutex_unlock(&asp->lock);

	for (i = 0; i < ARRAY_SIZE(elapsed); i++)
		if (elapsed[i])
			snd_pcm_period_elapsed(elapsed[i]);

	return IRQ_HANDLED;
}

static const struct snd_pcm_hardware asp_pcm_hw = {
	/* BATCH: the position only moves in whole periods */
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = ASP_RATE,
	.rate_max = ASP_RATE,
	.channels_min = 1,
	.channels_max = CAP_CH_MAX,
	.period_bytes_min = PERIOD_FRAMES * 2,
	.period_bytes_max = PERIOD_FRAMES * 4 * CAP_CH_MAX,
	.periods_min = PERIODS_MIN,
	.periods_max = PERIODS_MAX,
	.buffer_bytes_max = PERIOD_FRAMES * 4 * CAP_CH_MAX * PERIODS_MAX,
};

static struct asp_pcm *to_asp(struct snd_soc_component *component)
{
	return snd_soc_component_get_drvdata(component);
}

static int asp_pcm_open(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct asp_pcm *asp = to_asp(component);
	struct snd_pcm_runtime *rt = substream->runtime;
	struct asp_stream *s = &asp->streams[substream->stream];
	int ret;

	snd_soc_set_runtime_hwparams(substream, &asp_pcm_hw);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_pcm_hw_constraint_minmax(rt, SNDRV_PCM_HW_PARAM_CHANNELS,
			PLAY_CH_MAX, PLAY_CH_MAX);
	/* the DMA works in fixed 20 ms chunks */
	snd_pcm_hw_constraint_minmax(rt, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
		PERIOD_FRAMES, PERIOD_FRAMES);
	snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);

	/* ASP power domain and bus clock must be up before touching the DMA */
	ret = regulator_enable(asp->asp_supply);
	if (ret)
		return ret;
	ret = clk_prepare_enable(asp->asp_clk);
	if (ret) {
		regulator_disable(asp->asp_supply);
		return ret;
	}

	mutex_lock(&asp->lock);
	s->substream = substream;
	s->running = false;
	asp->users++;
	mutex_unlock(&asp->lock);
	return 0;
}

static int asp_pcm_close(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];

	mutex_lock(&asp->lock);
	s->substream = NULL;
	asp->users--;
	mutex_unlock(&asp->lock);
	clk_disable_unprepare(asp->asp_clk);
	regulator_disable(asp->asp_supply);
	return 0;
}

static int asp_pcm_hw_params(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	mutex_lock(&asp->lock);
	s->nports = params_channels(params);
	s->ports = playback ? play_ports : cap_ports;
	s->sample_bytes = params_physical_width(params) / 8;
	mutex_unlock(&asp->lock);
	return 0;
}

static int asp_pcm_prepare(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];
	unsigned int p;

	mutex_lock(&asp->lock);
	for (p = 0; p < s->nports; p++)
		dma_stop_channel(asp, s->ports[p].chan);
	s->running = false;
	s->pos = 0;
	s->fill = 0;
	s->irq_pending = 0;
	memset_io(s->buf, 0, 2 * s->nports * CHUNK_BYTES);
	asp_stream_setup_dma(s);
	mutex_unlock(&asp->lock);
	return 0;
}

static int asp_pcm_trigger(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, int cmd)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];
	unsigned int p;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mutex_lock(&asp->lock);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/*
			 * Chunk A plays silence, chunk B the first period: a start
			 * takes one period, which is what the application has
			 * at least prefilled.
			 */
			asp_silence_chunk(s, 0);
			asp_fill_chunk(s, 1);
		}
		if (cmd != SNDRV_PCM_TRIGGER_START)
			asp_stream_setup_dma(s);
		for (p = 0; p < s->nports; p++)
			slimbus_clear_port_fifo(s->ports[p].dport);
		s->irq_pending = 0;
		s->running = true;
		for (p = 0; p < s->nports; p++)
			writel(s->ports[p].config, asp->dma + DMA_CX_CONFIG(s->ports[p].chan));
		mutex_unlock(&asp->lock);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mutex_lock(&asp->lock);
		s->running = false;
		for (p = 0; p < s->nports; p++)
			dma_stop_channel(asp, s->ports[p].chan);
		mutex_unlock(&asp->lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* the IRQ thread may still be reporting a period of a stopped stream */
static int asp_pcm_sync_stop(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	synchronize_irq(to_asp(component)->irq);
	return 0;
}

static snd_pcm_uframes_t asp_pcm_pointer(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return s->fill;
	return (s->pos % substream->runtime->periods) * PERIOD_FRAMES;
}

/* playback: frames copied to the DMA chunks but not played yet */
static snd_pcm_sframes_t asp_pcm_delay(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct asp_pcm *asp = to_asp(component);
	struct asp_stream *s = &asp->streams[substream->stream];
	u32 cur, a, b, done = 0;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK || !s->running)
		return 0;

	cur = readl(asp->dma + DMA_CX_SRC(s->ports[0].chan));
	a = s->buf_phys + stream_chunk_off(s, 0, 0);
	b = s->buf_phys + stream_chunk_off(s, 1, 0);
	if (cur >= a && cur <= a + CHUNK_BYTES)
		done = (cur - a) / 4;
	else if (cur >= b && cur <= b + CHUNK_BYTES)
		done = (cur - b) / 4;

	/* the rest of the running chunk and all of the queued one */
	return 2 * PERIOD_FRAMES - min_t(u32, done, PERIOD_FRAMES);
}

static int asp_pcm_new(struct snd_soc_component *component,
	struct snd_soc_pcm_runtime *rtd)
{
	/* the DMA never touches the ALSA buffer: plain vmalloc memory */
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
		0, asp_pcm_hw.buffer_bytes_max);
	return 0;
}

static struct snd_soc_dai_driver asp_pcm_dai = {
	.name = "slimbus-dai",
	.playback = {
		.stream_name = "ASP Playback",
		.channels_min = PLAY_CH_MAX,
		.channels_max = PLAY_CH_MAX,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "ASP Capture",
		.channels_min = 1,
		.channels_max = CAP_CH_MAX,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
};

static const struct snd_soc_component_driver asp_pcm_component = {
	.name = "asp-pcm",
	.open = asp_pcm_open,
	.close = asp_pcm_close,
	.hw_params = asp_pcm_hw_params,
	.prepare = asp_pcm_prepare,
	.trigger = asp_pcm_trigger,
	.sync_stop = asp_pcm_sync_stop,
	.pointer = asp_pcm_pointer,
	.delay = asp_pcm_delay,
	.pcm_construct = asp_pcm_new,
};

/*
 * debugfs asp-pcm/dmac: the whole ASP DMAC, read-only. The block is only clocked while
 * a substream is open, so power it up around the read like asp_pcm_open() does.
 */
static int asp_dmac_show(struct seq_file *m, void *unused)
{
	struct asp_pcm *asp = m->private;
	unsigned int c, g;
	int ret;

	ret = regulator_enable(asp->asp_supply);
	if (ret)
		return ret;
	ret = clk_prepare_enable(asp->asp_clk);
	if (ret) {
		regulator_disable(asp->asp_supply);
		return ret;
	}
	mutex_lock(&asp->lock);
	seq_printf(m, "users %u ch_stat %#010x ch_pri %#010x ctrl %#010x\n", asp->users,
		readl(asp->dma + DMA_CH_STAT), readl(asp->dma + DMA_CH_PRI), readl(asp->dma + DMA_CTRL));
	seq_printf(m, "raw tc1 %#x tc2 %#x err1 %#x err2 %#x err3 %#x\n",
		readl(asp->dma + DMA_INT_TC1_RAW), readl(asp->dma + DMA_INT_TC2_RAW),
		readl(asp->dma + DMA_INT_ERR1_RAW), readl(asp->dma + DMA_INT_ERR2_RAW),
		readl(asp->dma + DMA_INT_ERR3_RAW));
	for (g = 0; g < 4; g++)
		seq_printf(m, "group %u: stat %#x masks tc1 %#x tc2 %#x err1 %#x err2 %#x err3 %#x\n", g,
			readl(asp->dma + 0x40 * g), readl(asp->dma + DMA_INT_TC1_MASK + 0x40 * g),
			readl(asp->dma + DMA_INT_TC2_MASK + 0x40 * g),
			readl(asp->dma + DMA_INT_ERR1_MASK + 0x40 * g),
			readl(asp->dma + DMA_INT_ERR2_MASK + 0x40 * g),
			readl(asp->dma + DMA_INT_ERR3_MASK + 0x40 * g));
	for (c = 0; c < DMA_NR_CHANNELS; c++)
		seq_printf(m, "ch%-2u config %#010x axi %#010x lli %#010x src %#010x des %#010x cnt0 %#x cur src %#010x des %#010x cnt0 %#x\n",
			c, readl(asp->dma + DMA_CX_CONFIG(c)), readl(asp->dma + DMA_CX_AXI_CONF(c)),
			readl(asp->dma + DMA_CX_LLI(c)), readl(asp->dma + DMA_CX_SRC(c)),
			readl(asp->dma + DMA_CX_DES(c)), readl(asp->dma + DMA_CX_CNT0(c)),
			readl(asp->dma + DMA_CX_CURR_SRC(c)), readl(asp->dma + DMA_CX_CURR_DES(c)),
			readl(asp->dma + DMA_CX_CURR_CNT0(c)));
	mutex_unlock(&asp->lock);
	clk_disable_unprepare(asp->asp_clk);
	regulator_disable(asp->asp_supply);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(asp_dmac);

/* ASP_CFG ranges worth seeing: resets/gates/clocks/DSP/DMAC select, security, SLIMbus format */
static const struct { u32 from, to; } asp_cfg_ranges[] = {
	{ 0x000, 0x07c }, { 0x100, 0x11c }, { 0x1b8, 0x1e8 },
};

static int asp_cfg_show(struct seq_file *m, void *unused)
{
	struct asp_pcm *asp = m->private;
	unsigned int i, r;
	int ret;

	if (!asp->cfg)
		return -ENODEV;
	ret = regulator_enable(asp->asp_supply);
	if (ret)
		return ret;
	ret = clk_prepare_enable(asp->asp_clk);
	if (ret) {
		regulator_disable(asp->asp_supply);
		return ret;
	}
	for (i = 0; i < ARRAY_SIZE(asp_cfg_ranges); i++)
		for (r = asp_cfg_ranges[i].from; r <= asp_cfg_ranges[i].to; r += 4)
			seq_printf(m, "%#05x %#010x\n", r, readl(asp->cfg + r));
	clk_disable_unprepare(asp->asp_clk);
	regulator_disable(asp->asp_supply);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(asp_cfg);

static int asp_pcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct reserved_mem *rmem;
	struct resource res;
	struct asp_pcm *asp;
	int irq, ret, i;

	if (!slimbus_is_ready())
		return dev_err_probe(dev, -EPROBE_DEFER, "waiting for SLIMbus\n");

	asp = devm_kzalloc(dev, sizeof(*asp), GFP_KERNEL);
	if (!asp)
		return -ENOMEM;
	asp->dev = dev;
	mutex_init(&asp->lock);
	spin_lock_init(&asp->reg_lock);

	asp->dma = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(asp->dma))
		return PTR_ERR(asp->dma);

	np = of_find_compatible_node(NULL, NULL, "candance,slimbus");
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no SLIMbus node\n");
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		of_node_put(np);
		return ret;
	}
	asp->slimbus_phys = res.start;
	/* the SLIMbus node's second range is ASP_CFG; mapped again (shared), only read */
	if (!of_address_to_resource(np, 1, &res))
		asp->cfg = devm_ioremap(dev, res.start, 0x200);
	of_node_put(np);

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no memory-region\n");
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem || rmem->size < PCM_AREA_OFFSET + PCM_DMA_BUF_ALL + 4 * PCM_LLI_SIZE)
		return dev_err_probe(dev, -EINVAL, "bad memory-region\n");
	asp->area_phys = rmem->base + PCM_AREA_OFFSET;
	asp->area = devm_ioremap_wc(dev, asp->area_phys, PCM_DMA_BUF_ALL + 4 * PCM_LLI_SIZE);
	if (!asp->area)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(asp->streams); i++) {
		struct asp_stream *s = &asp->streams[i];

		s->asp = asp;
		s->buf = asp->area + i * PCM_DMA_BUF_SIZE;
		s->buf_phys = asp->area_phys + i * PCM_DMA_BUF_SIZE;
		s->lli = asp->area + PCM_DMA_BUF_ALL + i * PCM_LLI_SIZE;
		s->lli_phys = asp->area_phys + PCM_DMA_BUF_ALL + i * PCM_LLI_SIZE;
	}

	asp->asp_supply = devm_regulator_get(dev, "asp-dmac");
	if (IS_ERR(asp->asp_supply))
		return dev_err_probe(dev, PTR_ERR(asp->asp_supply), "no ASP supply\n");

	asp->asp_clk = devm_clk_get(dev, "clk_asp_subsys");
	if (IS_ERR(asp->asp_clk))
		return dev_err_probe(dev, PTR_ERR(asp->asp_clk), "no ASP clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	asp->irq = irq;
	ret = devm_request_threaded_irq(dev, irq, NULL, asp_dma_irq, IRQF_ONESHOT,
		"asp_dma_irq", asp);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, asp);
	ret = devm_snd_soc_register_component(dev, &asp_pcm_component, &asp_pcm_dai, 1);
	if (ret)
		return ret;
	{
		struct dentry *d = debugfs_create_dir("asp-pcm", NULL);

		debugfs_create_file("dmac", 0400, d, asp, &asp_dmac_fops);
		debugfs_create_file("asp_cfg", 0400, d, asp, &asp_cfg_fops);
	}
	if (asp->cfg && !regulator_enable(asp->asp_supply)) {
		if (!clk_prepare_enable(asp->asp_clk)) {
			dev_info(dev, "ASP_CFG: rst %#x gate %#x dsp runstall %#x status %#x dmac_sel %#x tz %#x slim fmt %#x chnl %#x\n",
				readl(asp->cfg + 0x8), readl(asp->cfg + 0x18), readl(asp->cfg + 0x44),
				readl(asp->cfg + 0x50), readl(asp->cfg + 0x54), readl(asp->cfg + 0x100),
				readl(asp->cfg + 0x1bc), readl(asp->cfg + 0x1c0));
			clk_disable_unprepare(asp->asp_clk);
		}
		regulator_disable(asp->asp_supply);
	}
	dev_info(dev, "ASP PCM: dma buffers at %pa, SLIMbus fifos at %pa\n",
		&asp->area_phys, &asp->slimbus_phys);
	return 0;
}

static const struct of_device_id asp_pcm_match[] = {
	{ .compatible = "hisilicon,hi64xx-asp-dma", },
	{ }
};
MODULE_DEVICE_TABLE(of, asp_pcm_match);

struct platform_driver asp_pcm_driver = {
	.driver = {
		.name = "hisi-asp-pcm",
		.of_match_table = asp_pcm_match,
	},
	.probe = asp_pcm_probe,
};

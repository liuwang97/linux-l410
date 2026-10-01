// SPDX-License-Identifier: GPL-2.0
/*
 * debugfs register dump for the Hi6405, in the vendor /proc/audio/rr format
 * ("w <addr> <value>" lines per page) so it can be diffed against dumps
 * taken on the vendor kernel. Writing "<addr> <value>" writes a register.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <sound/soc.h>

#include "codec_debugfs.h"

static const struct {
	const char *name;
	unsigned int start, end, stride;
} hi6405_dump_pages[] = {
	/* the 0x2000xxxx base is added by the codec read callback */
	{ "PAGE IO",  0x1000, 0x104c, 4 },
	{ "PAGE CFG", 0x7000, 0x70ff, 1 },
	{ "PAGE ANA", 0x7100, 0x71ff, 1 },
	{ "PAGE DIG", 0x7200, 0x763d, 1 },
};

static struct dentry *hi6405_debugfs_dir;

static int hi6405_regs_show(struct seq_file *s, void *unused)
{
	struct snd_soc_component *codec = s->private;
	unsigned int i, reg;

	seq_puts(s, "BEGIN rr\n");
	for (i = 0; i < ARRAY_SIZE(hi6405_dump_pages); i++) {
		seq_printf(s, "%s\n", hi6405_dump_pages[i].name);
		for (reg = hi6405_dump_pages[i].start; reg <= hi6405_dump_pages[i].end;
		     reg += hi6405_dump_pages[i].stride)
			seq_printf(s, "w 0x%08X 0x%08X\n", 0x20000000 | reg,
				snd_soc_component_read(codec, reg));
	}
	seq_puts(s, "\nEND\n");
	return 0;
}

static int hi6405_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hi6405_regs_show, inode->i_private);
}

static ssize_t hi6405_regs_write(struct file *file, const char __user *ubuf,
	size_t count, loff_t *ppos)
{
	struct snd_soc_component *codec =
		((struct seq_file *)file->private_data)->private;
	char buf[48];
	unsigned int reg, val;
	size_t len = min(count, sizeof(buf) - 1);

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;
	/* accept the dump format (0x2000xxxx) as well as page offsets */
	reg &= ~0x20000000;
	snd_soc_component_write(codec, reg, val);
	return count;
}

static const struct file_operations hi6405_regs_fops = {
	.open = hi6405_regs_open,
	.read = seq_read,
	.write = hi6405_regs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void hi6405_debugfs_init(struct snd_soc_component *codec)
{
	hi6405_debugfs_dir = debugfs_create_dir("hi6405", NULL);
	debugfs_create_file("registers", 0600, hi6405_debugfs_dir, codec,
		&hi6405_regs_fops);
}

void hi6405_debugfs_remove(void)
{
	debugfs_remove_recursive(hi6405_debugfs_dir);
	hi6405_debugfs_dir = NULL;
}

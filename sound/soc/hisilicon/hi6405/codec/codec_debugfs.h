/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __HI6405_CODEC_DEBUGFS_H__
#define __HI6405_CODEC_DEBUGFS_H__

struct snd_soc_component;

void hi6405_debugfs_init(struct snd_soc_component *codec);
void hi6405_debugfs_remove(void);

#endif

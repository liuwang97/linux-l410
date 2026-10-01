// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal implementation of the Huawei "SecureC" (*_s) functions used by the vendor
 * Hi110x code. The vendor kernel linked lib/libc_sec; only the subset used here is
 * provided, with the same return conventions (EOK / E*_AND_RESET / -1).
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/stdarg.h>
#include <linux/ctype.h>
#include "securec.h"

#define SEC_MEM_MAX_LEN		(0x7fffffffUL)
#define SEC_STRING_MAX_LEN	(0x7fffffffUL)

static bool sec_overlap(const void *d, size_t dl, const void *s, size_t sl)
{
	const char *dp = d, *sp = s;

	return (dp < sp + sl) && (sp < dp + dl);
}

errno_t memcpy_s(void *dest, size_t destMax, const void *src, size_t count)
{
	if (!dest || destMax == 0 || destMax > SEC_MEM_MAX_LEN)
		return EINVAL;
	if (!src) {
		memset(dest, 0, destMax);
		return EINVAL_AND_RESET;
	}
	if (count > destMax) {
		memset(dest, 0, destMax);
		return ERANGE_AND_RESET;
	}
	if (dest == src)
		return EOK;
	if (sec_overlap(dest, count, src, count)) {
		memset(dest, 0, destMax);
		return EOVERLAP_AND_RESET;
	}
	memcpy(dest, src, count);
	return EOK;
}

errno_t memmove_s(void *dest, size_t destMax, const void *src, size_t count)
{
	if (!dest || destMax == 0 || destMax > SEC_MEM_MAX_LEN)
		return EINVAL;
	if (!src) {
		memset(dest, 0, destMax);
		return EINVAL_AND_RESET;
	}
	if (count > destMax) {
		memset(dest, 0, destMax);
		return ERANGE_AND_RESET;
	}
	memmove(dest, src, count);
	return EOK;
}

errno_t memset_s(void *dest, size_t destMax, int c, size_t count)
{
	if (!dest || destMax == 0 || destMax > SEC_MEM_MAX_LEN)
		return EINVAL;
	if (count > destMax) {
		memset(dest, c, destMax);
		return ERANGE_AND_RESET;
	}
	memset(dest, c, count);
	return EOK;
}

errno_t strcpy_s(char *strDest, size_t destMax, const char *strSrc)
{
	size_t n;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN)
		return EINVAL;
	if (!strSrc) {
		strDest[0] = '\0';
		return EINVAL_AND_RESET;
	}
	n = strnlen(strSrc, destMax);
	if (n >= destMax) {
		strDest[0] = '\0';
		return ERANGE_AND_RESET;
	}
	memmove(strDest, strSrc, n + 1);
	return EOK;
}

errno_t strncpy_s(char *strDest, size_t destMax, const char *strSrc, size_t count)
{
	size_t n;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN)
		return EINVAL;
	if (!strSrc) {
		strDest[0] = '\0';
		return EINVAL_AND_RESET;
	}
	n = strnlen(strSrc, count);
	if (n >= destMax) {
		strDest[0] = '\0';
		return ERANGE_AND_RESET;
	}
	memmove(strDest, strSrc, n);
	strDest[n] = '\0';
	return EOK;
}

errno_t strcat_s(char *strDest, size_t destMax, const char *strSrc)
{
	size_t dl, sl;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN)
		return EINVAL;
	if (!strSrc) {
		strDest[0] = '\0';
		return EINVAL_AND_RESET;
	}
	dl = strnlen(strDest, destMax);
	sl = strlen(strSrc);
	if (dl + sl >= destMax) {
		strDest[0] = '\0';
		return ERANGE_AND_RESET;
	}
	memmove(strDest + dl, strSrc, sl + 1);
	return EOK;
}

errno_t strncat_s(char *strDest, size_t destMax, const char *strSrc, size_t count)
{
	size_t dl, sl;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN)
		return EINVAL;
	if (!strSrc) {
		strDest[0] = '\0';
		return EINVAL_AND_RESET;
	}
	dl = strnlen(strDest, destMax);
	sl = strnlen(strSrc, count);
	if (dl + sl >= destMax) {
		strDest[0] = '\0';
		return ERANGE_AND_RESET;
	}
	memmove(strDest + dl, strSrc, sl);
	strDest[dl + sl] = '\0';
	return EOK;
}

int vsnprintf_s(char *strDest, size_t destMax, size_t count, const char *format, va_list argList)
{
	size_t lim;
	int n;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN || !format) {
		if (strDest && destMax > 0 && destMax <= SEC_STRING_MAX_LEN)
			strDest[0] = '\0';
		return -1;
	}
	lim = min(count, destMax - 1);
	n = vsnprintf(strDest, lim + 1, format, argList);
	if (n < 0)
		return -1;
	if ((size_t)n > lim)
		return -1;	/* truncated, like SECUREC_SNPRINTF_TRUNCATED */
	return n;
}

int snprintf_s(char *strDest, size_t destMax, size_t count, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = vsnprintf_s(strDest, destMax, count, format, ap);
	va_end(ap);
	return ret;
}

int vsprintf_s(char *strDest, size_t destMax, const char *format, va_list argList)
{
	int n;

	if (!strDest || destMax == 0 || destMax > SEC_STRING_MAX_LEN || !format)
		return -1;
	n = vsnprintf(strDest, destMax, format, argList);
	if (n < 0 || (size_t)n >= destMax) {
		strDest[0] = '\0';
		return -1;
	}
	return n;
}

int sprintf_s(char *strDest, size_t destMax, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = vsprintf_s(strDest, destMax, format, ap);
	va_end(ap);
	return ret;
}

/*
 * sscanf_s: %s/%c/%[ take an extra buffer size argument after the pointer.
 * Rewrite the format with explicit widths and hand the pointers to sscanf().
 */
#define SEC_SCANF_MAX_ARGS	10

int vsscanf_s(const char *buffer, const char *format, va_list argList)
{
	void *p[SEC_SCANF_MAX_ARGS] = { NULL };
	char fmt[256];
	const char *f = format;
	size_t o = 0;
	int n = 0;

	if (!buffer || !format)
		return -1;

	while (*f && o < sizeof(fmt) - 16) {
		bool suppress = false, width = false;
		char conv;

		if (*f != '%') {
			fmt[o++] = *f++;
			continue;
		}
		fmt[o++] = *f++;
		if (*f == '%') {
			fmt[o++] = *f++;
			continue;
		}
		if (*f == '*') {
			suppress = true;
			fmt[o++] = *f++;
		}
		while (isdigit(*f)) {
			width = true;
			fmt[o++] = *f++;
		}
		/* length modifiers are copied through */
		while (*f == 'h' || *f == 'l' || *f == 'L' || *f == 'z' || *f == 'j' || *f == 't')
			fmt[o++] = *f++;
		conv = *f;
		if (!conv)
			break;
		if (suppress) {
			fmt[o++] = *f++;
			continue;
		}
		if (n >= SEC_SCANF_MAX_ARGS)
			return -1;
		p[n] = va_arg(argList, void *);
		if (conv == 's' || conv == 'c' || conv == '[') {
			size_t sz = va_arg(argList, size_t) & 0xffffffffUL;

			if (sz == 0)
				return -1;
			if (!width) {
				/* insert the width before the length modifiers/conversion */
				char w[16];
				size_t wl, i;
				size_t pos = o;

				/* walk back over length modifiers already copied */
				while (pos > 0 && strchr("hlLzjt", fmt[pos - 1]))
					pos--;
				wl = scnprintf(w, sizeof(w), "%zu", conv == 'c' ? (size_t)1 : sz - 1);
				for (i = o; i > pos; i--)
					fmt[i - 1 + wl] = fmt[i - 1];
				memcpy(&fmt[pos], w, wl);
				o += wl;
			}
		}
		n++;
		fmt[o++] = *f++;
		if (conv == '[') {
			while (*f && *f != ']' && o < sizeof(fmt) - 2)
				fmt[o++] = *f++;
			if (*f == ']')
				fmt[o++] = *f++;
		}
	}
	fmt[o] = '\0';

	return sscanf(buffer, fmt, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
}

int sscanf_s(const char *buffer, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = vsscanf_s(buffer, format, ap);
	va_end(ap);
	return ret;
}

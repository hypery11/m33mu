/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * A scripted I2C bus master on the I2C1 pins, standing in for
 * the RK3576 CPU that talks to the Flipper One MCU firmware's I2C slave
 * (address 0x69, applications/services/i2c_intercom).  It drives the
 * DW_apb_i2c slave model of I2C1 (rp2350_extra.c) byte by byte, in emulated
 * time, and logs what happened.  It is NOT a model of the RK3576 I2C
 * controller or of any Linux driver.
 *
 * Environment:
 *   M33MU_I2C1_MASTER=<file>       the script (unset, "" or "0": off, and
 *                                  I2C1 stays the empty-bus model)
 *   M33MU_I2C1_MASTER_LOG=<file>   results ("-" or unset: stdout)
 *   M33MU_I2C1_MASTER_HZ=400000    SCL frequency
 *   M33MU_I2C1_STRETCH_MS=100      longest clock stretch the master waits for
 *   M33MU_I2C1_GAP_US=50           bus free time from a STOP to the next START
 *   M33MU_I2C1_TRACE=1             also log what the firmware does with the
 *                                  I2C1 registers and the slave's events
 *
 * Script: one command per line, '#' starts a comment.  Addresses (7 bit) and
 * bytes are hex with or without 0x, counts and times decimal.
 *   wait_log <text>            wait until <text> shows up in the firmware's
 *                              RTT log (needs M33MU_RTT; the RTT buffer is
 *                              drained every 2^18 cycles, so it is seen up
 *                              to ~2 ms of emulated time late)
 *   delay_ms <n>, delay_us <n> wait, emulated time
 *   gap_us <n>                 bus free time before each later START (as
 *                              M33MU_I2C1_GAP_US)
 *   irq_hold_us <n>            from now on, for n us of emulated time, the
 *                              I2C1 IRQ line reads low to the NVIC, as if the
 *                              firmware sat in a critical section; the
 *                              script goes on at once.  The FIFOs and the
 *                              interrupt status keep working; when the time
 *                              is up the line is sampled again
 *   write <addr> [<byte> ...]  START, addr+W, the bytes, STOP
 *   read <addr> <n>            START, addr+R, n bytes (the master ACKs all
 *                              but the last and NACKs the last), STOP
 *   write_read <addr> <byte> ... : <n>
 *                              START, addr+W, the bytes, repeated START,
 *                              addr+R, n bytes, STOP
 *   log <text>                 copy <text> to the log
 * The script starts at reset; each command starts when the one before it
 * ended, a transfer no earlier than the gap after the last STOP (the I2C
 * specification's minimum bus free time is 4.7 us at 100 kHz, 1.3 us at
 * 400 kHz; a host adds its own turnaround between two transfers).  A NACK from the slave ends the transfer with a STOP.  A clock
 * stretch longer than M33MU_I2C1_STRETCH_MS ends it too ("TIMEOUT"; a real
 * master cannot send a STOP while SCL is held low, this one does so that the
 * run goes on).  Up to 64 bytes per direction and 256 commands.
 *
 * Bus timing, in bit times of 1/M33MU_I2C1_MASTER_HZ: START 1, address or
 * data byte 8 plus 1 for the ACK bit, repeated START 1, STOP 1, plus however
 * long the slave holds SCL low.  Nothing below a bit exists (no SCL/SDA
 * edges, set-up/hold, rise times, spikes, arbitration).
 *
 * Log lines (t_us = emulated time since reset, system clock cycles / MHz):
 *   t_us=<t> line=<n> <command>                      a command starts
 *   t_us=<t> line=<n> XFER <events> dur_us=<d> stretch_us=<s> holds=<k>
 *       events: S, Sr, P; "69W:ACK" the address byte and the slave's answer;
 *       "00:ACK" a byte written and the slave's answer; "rd=01:ACK" a byte
 *       read and the master's answer; "hold" SCL held by the slave;
 *       "TIMEOUT"
 *   RESULT line=<n> <command> -> OK | NACK ... | TIMEOUT ...  [read: <bytes>]
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rp2350/rp2350_i2c1_master.h"
#include "rp2350/rp2350_extra.h"
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_timers.h"

#define M_MAX_CMDS  256u
#define M_MAX_BYTES 64u
#define M_MAX_TEXT  200u

enum { K_WAIT_LOG = 1, K_DELAY_US, K_WRITE, K_READ, K_WRITE_READ, K_LOG, K_GAP_US, K_IRQ_HOLD };
enum { R_NEXT = 0, R_DELAY, R_WAIT, R_XFER, R_DONE };
/* phases of a transfer; the action of a phase happens at ph_end */
enum { P_S = 1, P_ADDR, P_WDATA, P_SR, P_RREQ, P_RDATA, P_STOP };

struct m_cmd {
    int kind;
    int line;
    char text[M_MAX_TEXT]; /* the command, normalised */
    char arg[M_MAX_TEXT];  /* wait_log / log text */
    mm_u64 us;             /* delay */
    mm_u32 addr;
    mm_u8 wr[M_MAX_BYTES];
    mm_u32 n_wr;
    mm_u32 n_rd;
};

static struct {
    int state; /* -1 = environment not read yet, 0 = off, 1 = on */
    mm_bool busy;
    FILE *log;
    mm_bool trace;
    const char *script;
    mm_u64 hz;
    mm_u64 stretch_ms;
    mm_u64 gap_us;
    mm_u64 last_stop; /* time of the last STOP on the bus */
    struct m_cmd cmds[M_MAX_CMDS];
    mm_u32 n_cmds;
    mm_u32 pc;
    int run;
    mm_u64 t;     /* when the previous command ended */
    mm_u64 until; /* end of a delay */
    /* wait_log */
    char tail[M_MAX_TEXT];
    mm_u32 tail_n;
    mm_bool seen;
    mm_u64 seen_t;
    /* transfer in progress */
    struct m_cmd *x;
    int ph;
    mm_u64 ph_end;
    mm_u64 bit;
    mm_u64 x_t0;
    mm_u64 ev_t;     /* time of the bus event being handed to the slave */
    mm_bool rd_dir;  /* the address phase being sent is addr+R */
    mm_u32 i;        /* byte index in the current direction */
    mm_bool holding;
    mm_u64 hold_t0;
    mm_u64 hold_total;
    mm_u32 holds;
    mm_u8 rd[M_MAX_BYTES];
    mm_u32 n_rdone;
    char ev[2048];
    size_t ev_n;
    char status[160];
    /* counters */
    mm_u64 n_xfer, n_ok, n_nack, n_timeout, n_released;
    /* irq_hold_us */
    mm_bool irq_hold;
    mm_u64 irq_hold_until;
    mm_u64 n_irq_holds;
} g_m = {.state = -1};

static mm_u64 m_cycles_per_us(void)
{
    mm_u64 per_us = mm_rp2350_cpu_hz() / 1000000ull;
    return per_us ? per_us : 1u;
}

static mm_u64 m_us(mm_u64 cycles)
{
    return cycles / m_cycles_per_us();
}

static void m_logf(mm_u64 t, const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

static void m_logf(mm_u64 t, const char *fmt, ...)
{
    va_list ap;
    if (g_m.log == 0) return;
    fprintf(g_m.log, "t_us=%-10llu ", (unsigned long long)m_us(t));
    va_start(ap, fmt);
    vfprintf(g_m.log, fmt, ap);
    va_end(ap);
    fputc('\n', g_m.log);
    fflush(g_m.log);
}

static void m_ev(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

static void m_ev(const char *fmt, ...)
{
    va_list ap;
    int n;
    if (g_m.ev_n + 2u >= sizeof(g_m.ev)) return;
    if (g_m.ev_n != 0u) g_m.ev[g_m.ev_n++] = ' ';
    va_start(ap, fmt);
    n = vsnprintf(g_m.ev + g_m.ev_n, sizeof(g_m.ev) - g_m.ev_n, fmt, ap);
    va_end(ap);
    if (n > 0) g_m.ev_n += (size_t)n;
    if (g_m.ev_n >= sizeof(g_m.ev)) g_m.ev_n = sizeof(g_m.ev) - 1u;
}

/* ------------------------------------------------------------------ */
/* script parser                                                        */
/* ------------------------------------------------------------------ */

static char *m_trim(char *s)
{
    char *e;
    while (*s == ' ' || *s == '\t') ++s;
    e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) --e;
    *e = '\0';
    return s;
}

static mm_bool m_hex(const char *tok, mm_u32 max, mm_u32 *out)
{
    char *end = 0;
    unsigned long v;
    if (tok == 0 || *tok == '\0') return MM_FALSE;
    v = strtoul(tok, &end, 16); /* accepts an optional 0x */
    if (end == tok || *end != '\0' || v > max) return MM_FALSE;
    *out = (mm_u32)v;
    return MM_TRUE;
}

static mm_bool m_dec(const char *tok, mm_u64 max, mm_u64 *out)
{
    char *end = 0;
    unsigned long long v;
    if (tok == 0 || *tok < '0' || *tok > '9') return MM_FALSE;
    v = strtoull(tok, &end, 10);
    if (end == tok || *end != '\0' || v > max) return MM_FALSE;
    *out = (mm_u64)v;
    return MM_TRUE;
}

static void m_cmd_text(struct m_cmd *c)
{
    size_t n = 0;
    mm_u32 k;
    const char *name = (c->kind == K_WRITE) ? "write" : (c->kind == K_READ) ? "read" : "write_read";
    n += (size_t)snprintf(c->text + n, sizeof(c->text) - n, "%s %02x", name, c->addr);
    for (k = 0; k < c->n_wr && n < sizeof(c->text); ++k) {
        n += (size_t)snprintf(c->text + n, sizeof(c->text) - n, " %02x", c->wr[k]);
    }
    if (n >= sizeof(c->text)) return;
    if (c->kind == K_READ) snprintf(c->text + n, sizeof(c->text) - n, " %u", c->n_rd);
    else if (c->kind == K_WRITE_READ) snprintf(c->text + n, sizeof(c->text) - n, " : %u", c->n_rd);
}

/* Returns NULL on success, else what is wrong with the line. */
static const char *m_parse_line(char *line, int lineno, struct m_cmd *c)
{
    char buf[512];
    char *tok[2 * M_MAX_BYTES + 8];
    mm_u32 nt = 0;
    char *p;
    char *rest;
    size_t k = 0;
    mm_u32 v, i;
    mm_u64 d;
    memset(c, 0, sizeof(*c));
    c->line = lineno;
    /* command word and the rest of the line (for the text commands) */
    rest = line;
    while (*rest != '\0' && *rest != ' ' && *rest != '\t') ++rest;
    if (*rest != '\0') { *rest++ = '\0'; rest = m_trim(rest); }
    if (strcmp(line, "wait_log") == 0 || strcmp(line, "log") == 0) {
        if (*rest == '\0' && line[0] == 'w') return "wait_log needs a text";
        snprintf(c->arg, sizeof(c->arg), "%s", rest);
        c->kind = (line[0] == 'w') ? K_WAIT_LOG : K_LOG;
        snprintf(c->text, sizeof(c->text), "%s %s", line, rest);
        return 0;
    }
    /* the others: split into tokens, ':' is a token of its own */
    for (p = rest; *p != '\0' && k + 3u < sizeof(buf); ++p) {
        if (*p == ':') { buf[k++] = ' '; buf[k++] = ':'; buf[k++] = ' '; }
        else buf[k++] = *p;
    }
    buf[k] = '\0';
    for (p = strtok(buf, " \t"); p != 0 && nt < sizeof(tok) / sizeof(tok[0]); p = strtok(0, " \t")) tok[nt++] = p;
    if (strcmp(line, "irq_hold_us") == 0) {
        if (nt != 1 || !m_dec(tok[0], 100000000ull, &d)) return "expected one decimal number";
        c->kind = K_IRQ_HOLD;
        c->us = d;
        snprintf(c->text, sizeof(c->text), "%s %llu", line, (unsigned long long)d);
        return 0;
    }
    if (strcmp(line, "delay_ms") == 0 || strcmp(line, "delay_us") == 0 || strcmp(line, "gap_us") == 0) {
        if (nt != 1 || !m_dec(tok[0], 100000000ull, &d)) return "expected one decimal number";
        c->kind = (line[0] == 'g') ? K_GAP_US : K_DELAY_US;
        c->us = (line[6] == 'm') ? d * 1000ull : d;
        snprintf(c->text, sizeof(c->text), "%s %llu", line, (unsigned long long)d);
        return 0;
    }
    if (strcmp(line, "write") == 0) c->kind = K_WRITE;
    else if (strcmp(line, "read") == 0) c->kind = K_READ;
    else if (strcmp(line, "write_read") == 0) c->kind = K_WRITE_READ;
    else return "unknown command";
    if (nt < 1 || !m_hex(tok[0], 0x7fu, &c->addr)) return "expected a 7-bit hex address";
    if (c->kind == K_READ) {
        if (nt != 2 || !m_dec(tok[1], M_MAX_BYTES, &d) || d == 0u) return "expected: read <addr> <count 1..64>";
        c->n_rd = (mm_u32)d;
    } else {
        for (i = 1; i < nt && strcmp(tok[i], ":") != 0; ++i) {
            if (c->n_wr >= M_MAX_BYTES) return "more than 64 bytes";
            if (!m_hex(tok[i], 0xffu, &v)) return "expected hex bytes";
            c->wr[c->n_wr++] = (mm_u8)v;
        }
        if (c->kind == K_WRITE && i != nt) return "':' only belongs in write_read";
        if (c->kind == K_WRITE_READ) {
            if (i + 2u != nt || !m_dec(tok[i + 1u], M_MAX_BYTES, &d) || d == 0u)
                return "expected: write_read <addr> <bytes> : <count 1..64>";
            c->n_rd = (mm_u32)d;
        }
    }
    m_cmd_text(c);
    return 0;
}

static void m_load_script(void)
{
    FILE *f = fopen(g_m.script, "r");
    char line[512];
    int lineno = 0;
    if (f == 0) {
        printf("[I2C1M] cannot open script %s, nothing will run\n", g_m.script);
        m_logf(0, "SCRIPT ERROR cannot open %s", g_m.script);
        g_m.run = R_DONE;
        return;
    }
    while (fgets(line, sizeof(line), f) != 0) {
        char *s;
        char *hash;
        const char *err;
        ++lineno;
        hash = strchr(line, '#');
        if (hash != 0) *hash = '\0';
        s = m_trim(line);
        if (*s == '\0') continue;
        if (g_m.n_cmds >= M_MAX_CMDS) {
            err = "more than 256 commands";
        } else {
            err = m_parse_line(s, lineno, &g_m.cmds[g_m.n_cmds]);
        }
        if (err != 0) {
            printf("[I2C1M] %s line %d: %s; nothing will run\n", g_m.script, lineno, err);
            m_logf(0, "SCRIPT ERROR line=%d %s", lineno, err);
            g_m.n_cmds = 0u;
            g_m.run = R_DONE;
            fclose(f);
            return;
        }
        g_m.n_cmds++;
    }
    fclose(f);
}

mm_bool mm_rp2350_i2c1_master_enabled(void)
{
    const char *script, *path, *hz, *st, *tr, *gap;
    if (g_m.state >= 0) return g_m.state > 0 ? MM_TRUE : MM_FALSE;
    g_m.state = 0;
    script = getenv("M33MU_I2C1_MASTER");
    if (script == 0 || script[0] == '\0' || strcmp(script, "0") == 0) return MM_FALSE;
    g_m.script = script;
    path = getenv("M33MU_I2C1_MASTER_LOG");
    hz = getenv("M33MU_I2C1_MASTER_HZ");
    st = getenv("M33MU_I2C1_STRETCH_MS");
    tr = getenv("M33MU_I2C1_TRACE");
    gap = getenv("M33MU_I2C1_GAP_US");
    g_m.log = (path == 0 || path[0] == '\0' || strcmp(path, "-") == 0) ? stdout : fopen(path, "w");
    if (g_m.log == 0) g_m.log = stdout;
    g_m.hz = (hz != 0 && hz[0] != '\0') ? strtoull(hz, 0, 0) : 400000ull;
    if (g_m.hz == 0u) g_m.hz = 400000ull;
    g_m.stretch_ms = (st != 0 && st[0] != '\0') ? strtoull(st, 0, 0) : 100ull;
    g_m.trace = (tr != 0 && tr[0] != '\0' && strcmp(tr, "0") != 0) ? MM_TRUE : MM_FALSE;
    g_m.gap_us = (gap != 0 && gap[0] != '\0') ? strtoull(gap, 0, 0) : 50ull;
    g_m.state = 1;
    g_m.run = R_NEXT;
    fprintf(g_m.log, "# m33mu I2C1 scripted master: script=%s scl=%llu Hz stretch_timeout=%llu ms gap=%llu us%s\n"
                     "# emulated time throughout: t_us = system clock cycles / %llu; not real-world timings\n",
            script, (unsigned long long)g_m.hz, (unsigned long long)g_m.stretch_ms,
            (unsigned long long)g_m.gap_us, g_m.trace ? " trace=on" : "", (unsigned long long)m_cycles_per_us());
    fflush(g_m.log);
    m_load_script();
    printf("[I2C1M] scripted master on I2C1: %s, %u commands, scl=%llu Hz, log=%s\n", script, g_m.n_cmds,
           (unsigned long long)g_m.hz, (g_m.log == stdout) ? "stdout" : path);
    return MM_TRUE;
}

mm_bool mm_rp2350_i2c1_master_tracing(void)
{
    return (g_m.state > 0 && g_m.trace) ? MM_TRUE : MM_FALSE;
}

void mm_rp2350_i2c1_master_trace(const char *fmt, ...)
{
    va_list ap;
    if (!mm_rp2350_i2c1_master_tracing() || g_m.log == 0) return;
    /* inside the master: the time of the bus event, else the CPU's time */
    fprintf(g_m.log, "t_us=%-10llu   slv: ", (unsigned long long)m_us(g_m.busy ? g_m.ev_t : mm_rp2350_now_cycles()));
    va_start(ap, fmt);
    vfprintf(g_m.log, fmt, ap);
    va_end(ap);
    fputc('\n', g_m.log);
    fflush(g_m.log);
}

/* ------------------------------------------------------------------ */
/* wait_log                                                             */
/* ------------------------------------------------------------------ */

void mm_rp2350_i2c1_master_rtt(const mm_u8 *data, mm_u32 len)
{
    const struct m_cmd *c;
    size_t plen, n, k;
    char *buf;
    if (g_m.state <= 0 || g_m.run != R_WAIT || g_m.seen || len == 0u) return;
    c = &g_m.cmds[g_m.pc - 1u];
    plen = strlen(c->arg);
    n = g_m.tail_n + (size_t)len;
    buf = (char *)malloc(n + 1u);
    if (buf == 0) return;
    memcpy(buf, g_m.tail, g_m.tail_n);
    memcpy(buf + g_m.tail_n, data, len);
    buf[n] = '\0';
    for (k = 0; k + plen <= n; ++k) {
        if (memcmp(buf + k, c->arg, plen) == 0) {
            g_m.seen = MM_TRUE;
            g_m.seen_t = mm_rp2350_now_cycles();
            break;
        }
    }
    /* keep the last plen-1 bytes: a match may straddle two drains */
    k = (plen > 1u) ? plen - 1u : 0u;
    if (k > n) k = n;
    memcpy(g_m.tail, buf + n - k, k);
    g_m.tail_n = (mm_u32)k;
    free(buf);
}

/* ------------------------------------------------------------------ */
/* the bus                                                              */
/* ------------------------------------------------------------------ */

static void m_start_xfer(struct m_cmd *c)
{
    mm_u64 hz_cpu = mm_rp2350_cpu_hz();
    mm_u64 free_at = g_m.last_stop + g_m.gap_us * m_cycles_per_us();
    g_m.x = c;
    g_m.bit = (hz_cpu + g_m.hz / 2u) / g_m.hz;
    if (g_m.bit == 0u) g_m.bit = 1u;
    g_m.x_t0 = (g_m.n_xfer != 0u && free_at > g_m.t) ? free_at : g_m.t;
    g_m.ph = P_S;
    g_m.ph_end = g_m.x_t0;
    g_m.rd_dir = (c->kind == K_READ) ? MM_TRUE : MM_FALSE;
    g_m.i = 0u;
    g_m.holding = MM_FALSE;
    g_m.hold_total = 0u;
    g_m.holds = 0u;
    g_m.n_rdone = 0u;
    g_m.ev_n = 0u;
    g_m.ev[0] = '\0';
    g_m.status[0] = '\0';
    g_m.n_xfer++;
    g_m.run = R_XFER;
}

static void m_finish_xfer(mm_u64 t)
{
    const struct m_cmd *c = g_m.x;
    char rd[3u * M_MAX_BYTES + 16u];
    size_t n = 0;
    mm_u32 k;
    rd[0] = '\0';
    if (c->n_rd != 0u) {
        n += (size_t)snprintf(rd, sizeof(rd), " read:");
        for (k = 0; k < g_m.n_rdone; ++k) n += (size_t)snprintf(rd + n, sizeof(rd) - n, " %02x", g_m.rd[k]);
        if (g_m.n_rdone == 0u) snprintf(rd + n, sizeof(rd) - n, " (none)");
    }
    if (g_m.status[0] == '\0') {
        snprintf(g_m.status, sizeof(g_m.status), "OK");
        g_m.n_ok++;
    }
    m_logf(g_m.x_t0, "line=%d XFER %s dur_us=%llu stretch_us=%llu holds=%u", c->line, g_m.ev,
           (unsigned long long)m_us(t - g_m.x_t0), (unsigned long long)m_us(g_m.hold_total), g_m.holds);
    if (g_m.log != 0) {
        fprintf(g_m.log, "RESULT line=%d %s -> %s%s\n", c->line, c->text, g_m.status, rd);
        fflush(g_m.log);
    }
    g_m.t = t;
    g_m.x = 0;
    g_m.run = R_NEXT;
}

/* The slave holds SCL.  Returns MM_TRUE when the master gave up. */
static mm_bool m_hold(mm_u64 t, mm_u64 now, const char *what)
{
    mm_u64 limit = g_m.stretch_ms * (mm_rp2350_cpu_hz() / 1000u);
    if (!g_m.holding) {
        g_m.holding = MM_TRUE;
        g_m.hold_t0 = t;
        g_m.holds++;
    }
    if (now - g_m.hold_t0 <= limit) return MM_FALSE;
    g_m.hold_total += now - g_m.hold_t0;
    g_m.holding = MM_FALSE;
    m_ev("TIMEOUT");
    snprintf(g_m.status, sizeof(g_m.status), "TIMEOUT %s (SCL held low by the slave for more than %llu ms)", what,
             (unsigned long long)g_m.stretch_ms);
    g_m.n_timeout++;
    g_m.ph = P_STOP;
    g_m.ph_end = now + g_m.bit;
    return MM_TRUE;
}

static void m_unhold(mm_u64 t)
{
    if (!g_m.holding) return;
    g_m.hold_total += t - g_m.hold_t0;
    g_m.holding = MM_FALSE;
    m_ev("hold=%lluus", (unsigned long long)m_us(t - g_m.hold_t0));
}

static void m_nack(const char *what, mm_u64 t)
{
    if (g_m.status[0] == '\0') {
        snprintf(g_m.status, sizeof(g_m.status), "NACK %s", what);
        g_m.n_nack++;
    }
    g_m.ph = P_STOP;
    g_m.ph_end = t + 2u * g_m.bit; /* the ACK bit, then STOP */
}

/* One step of the transfer; MM_FALSE when it has to wait for time or the slave. */
static mm_bool m_xfer_step(mm_u64 now)
{
    struct m_cmd *c = g_m.x;
    mm_u64 bit = g_m.bit;
    mm_u64 t;
    int r;
    mm_u8 b = 0xffu;
    char what[48];
    if (g_m.holding) t = now;         /* a held phase is retried: released now, if at all */
    else if (now < g_m.ph_end) return MM_FALSE;
    else t = g_m.ph_end;
    g_m.ev_t = t;
    switch (g_m.ph) {
    case P_S:
    case P_SR:
        mm_rp2350_i2c1_bus_start(g_m.ph == P_SR ? MM_TRUE : MM_FALSE);
        m_ev(g_m.ph == P_SR ? "Sr" : "S");
        g_m.ph = P_ADDR;
        g_m.ph_end = t + 9u * bit; /* START hold, then the 8 address bits */
        return MM_TRUE;
    case P_ADDR:
        r = mm_rp2350_i2c1_bus_addr(c->addr, g_m.rd_dir);
        m_ev("%02x%c:%s", c->addr, g_m.rd_dir ? 'R' : 'W', r == MM_I2C_SLV_ACK ? "ACK" : "NACK");
        if (r != MM_I2C_SLV_ACK) {
            m_nack(g_m.rd_dir && c->kind == K_WRITE_READ ? "addr (read, after Sr)" : "addr", t);
            return MM_TRUE;
        }
        g_m.i = 0u;
        if (g_m.rd_dir) {
            g_m.ph = P_RREQ;
            g_m.ph_end = t + bit; /* the ACK bit */
        } else if (c->n_wr != 0u) {
            g_m.ph = P_WDATA;
            g_m.ph_end = t + bit + 8u * bit;
        } else if (c->kind == K_WRITE_READ) {
            g_m.ph = P_SR;
            g_m.rd_dir = MM_TRUE;
            g_m.ph_end = t + 2u * bit;
        } else {
            g_m.ph = P_STOP;
            g_m.ph_end = t + 2u * bit;
        }
        return MM_TRUE;
    case P_WDATA:
        r = mm_rp2350_i2c1_bus_write(c->wr[g_m.i]);
        if (r == MM_I2C_SLV_HOLD) {
            snprintf(what, sizeof(what), "before the ACK of byte %u (RX FIFO full)", g_m.i + 1u);
            return m_hold(t, now, what);
        }
        m_unhold(t);
        m_ev("%02x:%s", c->wr[g_m.i], r == MM_I2C_SLV_ACK ? "ACK" : "NACK");
        if (r != MM_I2C_SLV_ACK) {
            snprintf(what, sizeof(what), "data byte %u (0x%02x)", g_m.i + 1u, c->wr[g_m.i]);
            m_nack(what, t);
            return MM_TRUE;
        }
        g_m.i++;
        if (g_m.i < c->n_wr) {
            g_m.ph_end = t + bit + 8u * bit;
        } else if (c->kind == K_WRITE_READ) {
            g_m.ph = P_SR;
            g_m.rd_dir = MM_TRUE;
            g_m.ph_end = t + 2u * bit;
        } else {
            g_m.ph = P_STOP;
            g_m.ph_end = t + 2u * bit;
        }
        return MM_TRUE;
    case P_RREQ:
        r = mm_rp2350_i2c1_bus_read(&b);
        if (r == MM_I2C_SLV_HOLD) {
            snprintf(what, sizeof(what), "waiting for read byte %u", g_m.i + 1u);
            return m_hold(t, now, what);
        }
        m_unhold(t);
        if (r != MM_I2C_SLV_ACK) {
            /* the slave does not drive SDA: the master reads the pull-up */
            b = 0xffu;
            m_ev("released");
            g_m.n_released++;
            if (g_m.status[0] == '\0') {
                snprintf(g_m.status, sizeof(g_m.status), "SLAVE-NOT-DRIVING from read byte %u", g_m.i + 1u);
            }
        }
        g_m.rd[g_m.i] = b;
        g_m.ph = P_RDATA;
        g_m.ph_end = t + 9u * bit; /* 8 data bits, the master's ACK bit */
        return MM_TRUE;
    case P_RDATA: {
        mm_bool ack = (g_m.i + 1u < c->n_rd) ? MM_TRUE : MM_FALSE;
        mm_rp2350_i2c1_bus_master_ack(ack);
        m_ev("rd=%02x:%s", g_m.rd[g_m.i], ack ? "ACK" : "NACK");
        g_m.i++;
        g_m.n_rdone = g_m.i;
        if (g_m.i < c->n_rd) {
            g_m.ph = P_RREQ;
            g_m.ph_end = t;
        } else {
            g_m.ph = P_STOP;
            g_m.ph_end = t + bit;
        }
        return MM_TRUE;
    }
    case P_STOP:
    default:
        mm_rp2350_i2c1_bus_stop();
        m_ev("P");
        g_m.last_stop = t;
        m_finish_xfer(t);
        return MM_TRUE;
    }
}

static mm_bool m_next_cmd(void)
{
    struct m_cmd *c;
    if (g_m.pc >= g_m.n_cmds) {
        m_logf(g_m.t, "END of script, %u commands", g_m.n_cmds);
        g_m.run = R_DONE;
        return MM_FALSE;
    }
    c = &g_m.cmds[g_m.pc++];
    m_logf(g_m.t, "line=%d %s", c->line, c->text);
    switch (c->kind) {
    case K_LOG:
        return MM_TRUE;
    case K_GAP_US:
        g_m.gap_us = c->us;
        return MM_TRUE;
    case K_IRQ_HOLD:
        g_m.irq_hold_until = g_m.t + c->us * m_cycles_per_us();
        g_m.irq_hold = MM_TRUE;
        g_m.n_irq_holds++;
        m_logf(g_m.t, "I2C1 IRQ held off until t_us=%llu", (unsigned long long)m_us(g_m.irq_hold_until));
        return MM_TRUE;
    case K_DELAY_US:
        g_m.until = g_m.t + c->us * m_cycles_per_us();
        g_m.run = R_DELAY;
        return MM_TRUE;
    case K_WAIT_LOG: {
        const char *rtt = getenv("M33MU_RTT");
        if (rtt == 0 || rtt[0] == '\0') {
            m_logf(g_m.t, "SCRIPT ERROR line=%d wait_log needs M33MU_RTT; script stopped", c->line);
            g_m.run = R_DONE;
            return MM_FALSE;
        }
        g_m.seen = MM_FALSE;
        g_m.tail_n = 0u;
        g_m.run = R_WAIT;
        return MM_TRUE;
    }
    default:
        m_start_xfer(c);
        return MM_TRUE;
    }
}

void mm_rp2350_i2c1_master_run(void)
{
    mm_u64 now;
    if (g_m.state <= 0 || g_m.busy) return;
    g_m.busy = MM_TRUE;
    now = mm_rp2350_now_cycles();
    for (;;) {
        if (g_m.run == R_NEXT) {
            if (!m_next_cmd()) break;
        } else if (g_m.run == R_DELAY) {
            if (now < g_m.until) break;
            g_m.t = g_m.until;
            g_m.run = R_NEXT;
        } else if (g_m.run == R_WAIT) {
            const struct m_cmd *c = &g_m.cmds[g_m.pc - 1u];
            if (!g_m.seen) break;
            if (g_m.seen_t > g_m.t) g_m.t = g_m.seen_t;
            m_logf(g_m.t, "line=%d seen \"%s\"", c->line, c->arg);
            g_m.run = R_NEXT;
        } else if (g_m.run == R_XFER) {
            if (!m_xfer_step(now)) break;
        } else {
            break;
        }
    }
    g_m.busy = MM_FALSE;
}

mm_bool mm_rp2350_i2c1_master_irq_held(void)
{
    if (g_m.state <= 0 || !g_m.irq_hold) return MM_FALSE;
    if (mm_rp2350_now_cycles() < g_m.irq_hold_until) return MM_TRUE;
    g_m.irq_hold = MM_FALSE;
    m_logf(g_m.irq_hold_until, "I2C1 IRQ hold over");
    return MM_FALSE;
}

mm_u64 mm_rp2350_i2c1_master_next_event(void)
{
    mm_u64 now = mm_rp2350_now_cycles();
    mm_u64 at = (mm_u64)-1; /* waiting for the log, or done: nothing of the bus */
    if (g_m.state <= 0) return (mm_u64)-1;
    if (g_m.run == R_DELAY) at = g_m.until;
    else if (g_m.run == R_XFER && g_m.holding)
        at = g_m.hold_t0 + g_m.stretch_ms * (mm_rp2350_cpu_hz() / 1000u) + 1u;
    else if (g_m.run == R_XFER) at = g_m.ph_end;
    if (g_m.irq_hold && g_m.irq_hold_until < at) at = g_m.irq_hold_until; /* the line comes back then */
    if (at == (mm_u64)-1) return (mm_u64)-1;
    return (at > now) ? at - now : 1u;
}

void mm_rp2350_i2c1_master_stats(void)
{
    if (g_m.state <= 0) return;
    fprintf(stderr, "[I2C1M] script=%s commands_run=%u/%u transfers=%llu ok=%llu nack=%llu timeout=%llu "
                    "slave_not_driving=%llu%s\n",
            g_m.script, g_m.pc, g_m.n_cmds, (unsigned long long)g_m.n_xfer, (unsigned long long)g_m.n_ok,
            (unsigned long long)g_m.n_nack, (unsigned long long)g_m.n_timeout, (unsigned long long)g_m.n_released,
            (g_m.run == R_DONE) ? "" : " (script not finished)");
    if (g_m.n_irq_holds != 0u) fprintf(stderr, "[I2C1M] irq_holds=%llu\n", (unsigned long long)g_m.n_irq_holds);
    if (g_m.log != 0 && g_m.log != stdout) fflush(g_m.log);
}

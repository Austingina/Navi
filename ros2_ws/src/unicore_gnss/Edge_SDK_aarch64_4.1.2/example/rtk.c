/**
 * 真点 Edge SDK 最小可用示例（单文件重写版）
 *
 * 数据流：GNSS 串口读 GGA -> tp_send_gga 上报 -> tp_get_rtcm 取差分 -> 写回同一串口。
 *
 * 相比开发指南 §7 的原始示例，这里修正了以下问题：
 *  - 账号/密码/挂载点一律来自配置文件（tp_sdk_init 传 NULL）。原示例硬编码
 *    "username"/"passwd"，而传入参数优先级高于配置文件（文档 §4.2.4），
 *    会导致配置文件里填好的账号被覆盖、登录必然返回 6100。
 *  - 不再用 tp_sdk_config 覆盖 block_time / loglevel。原示例把 block_time 压成 3
 *    并把日志关成 NONE，使配置文件里的设置全部失效。
 *  - 按文档 §3.2 第 9 条忽略 SIGPIPE（SDK 依赖的 OpenSSL 不处理该信号，
 *    连接被对端关闭时进程会直接被杀），并支持 SIGINT/SIGTERM 优雅退出。
 *  - tp_sdk_query 的结构体先清零并校验返回值后才读取内容。
 *  - 断线自动重连；账号类错误直接退出，不做无意义的无限重试。
 *  - 按文档 §3.1 建议，先上报 GGA 再开始取 RTCM，避免启动时刷 4031。
 *  - 错误路径返回非 0 退出码，便于脚本/systemd 判断。
 *
 * 编译（在本目录下）：
 *   make
 * 或手工：
 *   gcc -std=c99 -Wall -Wextra -I../include rtk.c -L../library \
 *       -ltpservice -lpthread -lm -lssl -lcrypto -o rtk
 *
 * 运行前需在工作目录准备好 tpservice.conf（或用 -c 指定路径）：
 *   ./rtk -d /dev/ttyUSB0   # 接真实接收机，从串口读 GGA、把 RTCM 写回去
 *   ./rtk -m                # 用内置示例 GGA 验证账号与链路，不碰串口
 */

/* _POSIX_C_SOURCE 与 SDK 其它示例一致（clock_nanosleep 等）；
 * _DEFAULT_SOURCE 额外暴露 cfmakeraw / CRTSCTS 这两个 glibc 串口扩展。 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "tp_sdk_api.h"
#include "tp_sdk_status_code.h"

#define DEFAULT_CONFIG_PATH   "tpservice.conf"
#define DEFAULT_TTY_DEV       "/dev/ttyUSB5"
#define DEFAULT_TTY_BAUD      115200L
#define DEFAULT_GGA_INTERVAL  1L

#define RTCM_BUF_SIZE         (10 * 1024)
#define GGA_LINE_SIZE         256
#define GGA_ACC_SIZE          1024

/* SDK 要求 GGA 长度在 20~200 字节之间且符合 NMEA0183，否则 tp_send_gga 回 4020。
 * 这条取自开发指南 §7，用于 -m 模式做连通性验证。 */
#define BUILTIN_MOCK_GGA \
  "$GNGGA,090230.00,4004.7061,N,11614.0472,E,4,51,0.4,66.4860,M,-8.4923,M,1.0,387*4D\r\n"

typedef struct
{
  const char * config_path;
  const char * tty_dev;
  long tty_baud;
  const char * mock_gga;      /* 非 NULL 时不打开串口，只用这条固定 GGA */
  long gga_interval_s;
  int verbose;
} options_t;

static options_t g_opt = {
  DEFAULT_CONFIG_PATH,
  DEFAULT_TTY_DEV,
  DEFAULT_TTY_BAUD,
  NULL,
  DEFAULT_GGA_INTERVAL,
  0,
};

/* 信号处理函数里只能改 sig_atomic_t */
static volatile sig_atomic_t g_stop = 0;
static volatile sig_atomic_t g_need_reconnect = 1;
static volatile sig_atomic_t g_gga_sent = 0;

static int g_tty_fd = -1;
/* 取自 SDK 生效配置，用于 RTCM 静默看门狗 */
static int g_block_time_s = 10;

/* 运行统计。RTCM 三项只由接收线程写，GGA 计数只由主线程写，无需加锁。 */
#define STAT_PERIOD_MS 5000ULL
static unsigned long long g_rtcm_bytes = 0;
static unsigned long g_rtcm_packets = 0;
static unsigned long g_gga_sent_count = 0;

/* ------------------------------------------------------------------ 基础工具 */

static unsigned long long now_ms(void)
{
  struct timespec ts = {0};
  (void)clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)ts.tv_nsec / 1000000ULL;
}

/* 被信号打断时用剩余时间续睡，保证实际睡够（除非收到退出信号） */
static void sleep_ms(unsigned long long ms)
{
  struct timespec timeout;
  struct timespec remain;
  timeout.tv_sec = (time_t)(ms / 1000ULL);
  timeout.tv_nsec = (long)(ms % 1000ULL) * 1000000L;
  while (clock_nanosleep(CLOCK_MONOTONIC, 0, &timeout, &remain) != 0) {
    if (errno != EINTR || g_stop) {
      break;
    }
    timeout = remain;
  }
}

static void on_exit_signal(int signo)
{
  (void)signo;
  g_stop = 1;
}

/* SDK 日志回调。默认只放过 ERROR：本 SDK 在正常收发时也会刷大量 WARNING 级
 * 内部消息（ipc flush、epoll del 等），放过 WARNING 会把有用输出淹掉。-v 时全量输出。 */
static void log_callback(UINT64 timestamp, INT32 level, const char * tag, const char * message)
{
  (void)timestamp;
  (void)tag;
  if (g_opt.verbose || level >= LOG_LEVEL_ERROR) {
    fputs(message, stdout);
  }
}

static void socket_callback(int fd)
{
  if (g_opt.verbose) {
    printf("[net] socket fd %d created\n", fd);
  }
}

/* 账号类错误重试没有意义，直接退出让上层（脚本/运维）介入 */
static int is_fatal_platform_error(int errcode)
{
  switch (errcode) {
    case TP_SDK_ACCOUNT_OR_PASSWORD_ERROR:  /* 6100 账号密码错误 */
    case TP_SDK_ACCOUNT_TYPE_ERROR:         /* 6105 账号类型错误 */
    case TP_SDK_ACCOUNT_POOL_USEUP:         /* 6106 账号到期或资源耗尽 */
    case TP_SDK_ACCOUNT_NO_ASSIGNABLE:      /* 6116 没有可分配的账号 */
    case TP_SDK_ACCOUNT_HAS_PROHIBIT:       /* 6330 账号已禁用 */
      return 1;
    default:
      return 0;
  }
}

/* ------------------------------------------------------------------ 串口读写 */

static speed_t baud_to_speed(long baud)
{
  switch (baud) {
    case 9600L: return B9600;
    case 19200L: return B19200;
    case 38400L: return B38400;
    case 57600L: return B57600;
    case 115200L: return B115200;
    case 230400L: return B230400;
    case 460800L: return B460800;
    case 921600L: return B921600;
    default: return (speed_t)0;
  }
}

static int tty_open(const char * dev, long baud)
{
  speed_t speed = baud_to_speed(baud);
  if (speed == (speed_t)0) {
    fprintf(stderr, "[tty] unsupported baudrate %ld\n", baud);
    return -1;
  }

  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    fprintf(stderr, "[tty] open %s failed: %s\n", dev, strerror(errno));
    return -1;
  }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) {
    fprintf(stderr, "[tty] tcgetattr failed: %s\n", strerror(errno));
    close(fd);
    return -1;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  (void)cfsetispeed(&tio, speed);
  (void)cfsetospeed(&tio, speed);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    fprintf(stderr, "[tty] tcsetattr failed: %s\n", strerror(errno));
    close(fd);
    return -1;
  }
  (void)tcflush(fd, TCIOFLUSH);
  return fd;
}

/* 串口可能一次写不完，需要循环写直到写尽 */
static int tty_write_all(int fd, const unsigned char * buf, int len)
{
  int written = 0;
  while (written < len) {
    ssize_t n = write(fd, buf + written, (size_t)(len - written));
    if (n > 0) {
      written += (int)n;
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      sleep_ms(2);
      continue;
    }
    fprintf(stderr, "[tty] write failed: %s\n", strerror(errno));
    return -1;
  }
  return written;
}

/**
 * GGA 第 6 个字段是定位质量：0 或空表示无效解。接收机未定位时输出的是
 * "$GNGGA,,,,,,0,,,,,,,,*78"（24 字节），能通过 SDK 的 20~200 字节长度检查，
 * 上报后服务端拿不到位置、给不出可用改正数，只会在日志里刷失败，容易被误判
 * 成账号或网络故障。因此这里提前挡掉。
 */
static int gga_has_fix(const char * line)
{
  const char * p = strstr(line, "GGA,");
  if (p == NULL) {
    return 0;
  }
  p += 4;   /* 落到 field 1（UTC 时间）开头 */
  /* field 1 -> field 6 需跨过 5 个逗号 */
  for (int field = 1; field < 6; field++) {
    p = strchr(p, ',');
    if (p == NULL) {
      return 0;
    }
    p++;
  }
  return (*p >= '1' && *p <= '8');
}

/**
 * 从串口取一条完整 GGA。跨调用累积半行数据，一次调用内把缓冲里所有完整行读干，
 * 只保留最后一条**有效解**的 GGA（位置最新）。返回长度，超时无数据、或只收到
 * 无定位的 GGA 时返回 0，出错返回 -1。
 */
static int read_gga(int fd, char * out, size_t out_size, int timeout_ms)
{
  static char acc[GGA_ACC_SIZE];
  static size_t acc_len = 0;
  static unsigned long long last_nofix_log_ms = 0;
  int found = 0;
  int nofix = 0;

  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  int pret = poll(&pfd, 1, timeout_ms);
  if (pret < 0) {
    if (errno == EINTR) {
      return 0;
    }
    fprintf(stderr, "[tty] poll failed: %s\n", strerror(errno));
    return -1;
  }
  if (pret > 0) {
    ssize_t n = read(fd, acc + acc_len, sizeof(acc) - acc_len);
    if (n > 0) {
      acc_len += (size_t)n;
    } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      fprintf(stderr, "[tty] read failed: %s\n", strerror(errno));
      return -1;
    }
  }

  /* 逐行切分，保留最后一条 GGA */
  size_t start = 0;
  for (size_t i = 0; i < acc_len; i++) {
    if (acc[i] != '\n') {
      continue;
    }
    size_t line_len = i + 1 - start;
    if (acc[start] == '$' && line_len < out_size &&
      memchr(acc + start, 'G', line_len) != NULL)
    {
      char line[GGA_LINE_SIZE];
      size_t copy_len = (line_len < sizeof(line) - 1) ? line_len : sizeof(line) - 1;
      memcpy(line, acc + start, copy_len);
      line[copy_len] = '\0';
      if (strstr(line, "GGA,") != NULL) {
        if (gga_has_fix(line)) {
          memcpy(out, line, copy_len + 1);
          found = (int)copy_len;
        } else {
          nofix = 1;
        }
      }
    }
    start = i + 1;
  }

  /* 丢弃已消费部分；整缓冲无换行说明是噪声或超长行，直接清掉防止死锁 */
  if (start > 0) {
    memmove(acc, acc + start, acc_len - start);
    acc_len -= start;
  } else if (acc_len == sizeof(acc)) {
    acc_len = 0;
  }

  /* 全是无定位的 GGA 时限频提示一次，否则这段时间完全没有输出，
   * 看起来像程序挂了或者串口配错了。 */
  if (found == 0 && nofix) {
    unsigned long long now = now_ms();
    if (now - last_nofix_log_ms >= 10000ULL) {
      last_nofix_log_ms = now;
      printf("[gga] 接收机尚无定位解（GGA 质量位为 0），等待其自行定位后再上报\n");
    }
  }
  return found;
}

/* ------------------------------------------------------------------ RTCM 线程 */

static void * rtcm_thread_main(void * arg)
{
  (void)arg;
  unsigned char rtcm_buf[RTCM_BUF_SIZE];
  unsigned long long last_rtcm_ms = now_ms();
  unsigned long long last_stat_ms = now_ms();

  while (!g_stop) {
    /* 文档 §3.1：连接成功且已上报 GGA 之后才可能收到 RTCM */
    if (g_need_reconnect || !g_gga_sent) {
      sleep_ms(200);
      last_rtcm_ms = now_ms();
      continue;
    }

    int rtcm_len = (int)sizeof(rtcm_buf) - 1;
    rtcm_buf[0] = 0;
    int ret = tp_get_rtcm((INT8 *)rtcm_buf, &rtcm_len);
    int errcode = tp_get_errorcode(ret);

    if (ret == 0) {
      if (rtcm_len > 0) {
        last_rtcm_ms = now_ms();
        g_rtcm_bytes += (unsigned long long)rtcm_len;
        g_rtcm_packets++;
        if (g_tty_fd >= 0) {
          (void)tty_write_all(g_tty_fd, rtcm_buf, rtcm_len);
        }
        if (g_opt.verbose) {
          printf(
            "[rtcm] %d bytes -> %s\n", rtcm_len,
            (g_tty_fd >= 0) ? g_opt.tty_dev : "(discarded, no tty)");
        }
      }
    } else if (errcode == TP_SDK_NET_NETWORK_ERROR || errcode == TP_SDK_NET_TIMEOUT) {
      fprintf(
        stderr, "[rtcm] network error %d: %s, will reconnect\n",
        errcode, tp_get_errormessage(ret));
      g_need_reconnect = 1;
    } else if (errcode != TP_SDK_NET_NOT_CONNECT && errcode != TP_SDK_SERV_RECV_RTCM_TIMEOUT) {
      fprintf(stderr, "[rtcm] tp_get_rtcm failed %d: %s\n", errcode, tp_get_errormessage(ret));
    }

    /* 静默超过 block_time+2 秒说明链路实际已断，主动触发重连 */
    if ((now_ms() - last_rtcm_ms) > ((unsigned long long)(g_block_time_s + 2) * 1000ULL)) {
      fprintf(stderr, "[rtcm] no data for %llu ms, request reconnect\n", now_ms() - last_rtcm_ms);
      g_need_reconnect = 1;
      last_rtcm_ms = now_ms();
    }

    if ((now_ms() - last_stat_ms) >= STAT_PERIOD_MS) {
      last_stat_ms = now_ms();
      printf(
        "[stat] rtcm %llu B / %lu pkt, gga %lu sent, last rtcm %llu ms ago\n",
        g_rtcm_bytes, g_rtcm_packets, g_gga_sent_count, now_ms() - last_rtcm_ms);
    }
    sleep_ms(10);
  }
  return NULL;
}

/* ------------------------------------------------------------------ 配置打印 */

static void print_effective_config(void)
{
  TPconfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  if (tp_get_current_config(&cfg) != 0) {
    fprintf(stderr, "[cfg] tp_get_current_config failed\n");
    return;
  }
  g_block_time_s = (cfg.block_time > 0) ? cfg.block_time : g_block_time_s;
  printf(
    "[cfg] account=%s mountpoint=%s device_id=%s\n"
    "[cfg] server=%s:%u proto=%s authType=%d coordsys=%d\n"
    "[cfg] rtcm_interval=%d block_time=%d retry=%d/%ds gga_offset=%d loglevel=%u\n",
    (const char *)cfg.account, (const char *)cfg.mountpoint, (const char *)cfg.device_id,
    (const char *)cfg.server_host, cfg.server_port,
    (cfg.tli_protocol == (UINT32)TLI_SSL) ? "ssl" : "tcp",
    cfg.authType, cfg.coordsys,
    cfg.rtcm_interval, cfg.block_time, cfg.retry_number, cfg.retry_interval,
    cfg.support_gga_offset, cfg.loglevel);
}

/* ------------------------------------------------------------------ 命令行 */

static void usage(const char * prog)
{
  printf(
    "Usage: %s [OPTIONS]\n"
    "  -c PATH  配置文件路径（默认 %s，相对当前工作目录）\n"
    "  -d DEV   GNSS 串口，GGA 从此口读、RTCM 写回此口（默认 %s）\n"
    "  -b RATE  串口波特率（默认 %ld）\n"
    "  -m       用内置示例 GGA 做连通性验证，不打开串口（推荐的自测方式）\n"
    "  -g GGA   用指定的 GGA 字符串，不打开串口。必须是完整合法的 NMEA0183\n"
    "           语句（20~200 字节），例如：\n"
    "             -g '%s'\n"
    "  -i SEC   GGA 上报间隔秒（默认 %ld）\n"
    "  -v       输出 SDK 全量日志与收发明细\n"
    "  -h       显示本帮助\n",
    prog, DEFAULT_CONFIG_PATH, DEFAULT_TTY_DEV, DEFAULT_TTY_BAUD,
    "$GNGGA,090230.00,4004.7061,N,11614.0472,E,4,51,0.4,66.4860,M,-8.4923,M,1.0,387*4D",
    DEFAULT_GGA_INTERVAL);
}

/* 提前挡掉不合法的 -g，避免连上服务后才被 tp_send_gga 以 4020 拒绝 */
static int check_mock_gga(const char * gga)
{
  size_t len = strlen(gga);
  if (len < 20 || len > 200) {
    fprintf(stderr, "[arg] GGA 长度 %zu 不在 SDK 允许的 20~200 字节内\n", len);
  } else if (gga[0] != '$') {
    fprintf(stderr, "[arg] GGA 必须以 '$' 开头\n");
  } else if (strstr(gga, "GGA,") == NULL || strchr(gga, '*') == NULL) {
    fprintf(stderr, "[arg] GGA 必须含 \"GGA,\" 字段和 '*' 校验和分隔符\n");
  } else {
    return 0;
  }
  fprintf(stderr, "[arg] 需要完整语句而不是省略写法，直接用 -m 可跳过手输\n");
  return -1;
}

/* 返回 0 继续运行，1 正常退出（-h），-1 参数错误 */
static int parse_args(int argc, char * argv[])
{
  int opt;
  while ((opt = getopt(argc, argv, "c:d:b:mg:i:vh")) != -1) {
    switch (opt) {
      case 'c':
        g_opt.config_path = optarg;
        break;
      case 'd':
        g_opt.tty_dev = optarg;
        break;
      case 'b':
        g_opt.tty_baud = strtol(optarg, NULL, 10);
        break;
      case 'm':
        g_opt.mock_gga = BUILTIN_MOCK_GGA;
        break;
      case 'g':
        if (check_mock_gga(optarg) != 0) {
          return -1;
        }
        g_opt.mock_gga = optarg;
        break;
      case 'i':
        g_opt.gga_interval_s = strtol(optarg, NULL, 10);
        break;
      case 'v':
        g_opt.verbose = 1;
        break;
      case 'h':
        usage(argv[0]);
        return 1;
      default:
        usage(argv[0]);
        return -1;
    }
  }
  if (g_opt.gga_interval_s <= 0) {
    fprintf(stderr, "invalid gga interval\n");
    return -1;
  }
  return 0;
}

/* ------------------------------------------------------------------ main */

int main(int argc, char * argv[])
{
  /* 输出被重定向到文件/tee 时 stdout 默认块缓冲，会和无缓冲的 stderr 错序，
   * 且崩溃时丢日志。强制行缓冲。 */
  (void)setvbuf(stdout, NULL, _IOLBF, 0);

  int rc = parse_args(argc, argv);
  if (rc != 0) {
    return (rc > 0) ? 0 : 1;
  }

  /* 文档 §3.2 第 5 条：日志回调必须在 tp_set_config_path / tp_sdk_init 之前注册 */
  tp_set_log_cb(log_callback);
  tp_set_socket_cb(socket_callback);

  /* 文档 §3.2 第 9 条：OpenSSL 不处理 SIGPIPE，必须忽略否则连接断开时进程被杀 */
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
    fprintf(stderr, "[init] ignore SIGPIPE failed: %s\n", strerror(errno));
    return 1;
  }
  (void)signal(SIGINT, on_exit_signal);
  (void)signal(SIGTERM, on_exit_signal);

  St_sdk_response_t response;
  memset(&response, 0, sizeof(response));
  if (tp_sdk_query(GET_SDK_VERSION, &response, NULL) == 0) {
    printf("[init] true-point sdk version: %s\n", response.uResult.com_result.buf);
  } else {
    fprintf(stderr, "[init] query sdk version failed\n");
  }

  int ret = tp_set_config_path((const INT8 *)g_opt.config_path);
  if (ret != 0) {
    fprintf(
      stderr, "[init] tp_set_config_path(%s) failed %d: %s\n",
      g_opt.config_path, tp_get_errorcode(ret), tp_get_errormessage(ret));
    return 1;
  }

  /* 全部传 NULL/-1：账号、密码、设备ID、挂载点、播发间隔、认证方式一律用配置文件的值。
   * 传非 NULL 会覆盖配置文件（文档 §4.2.4），这正是原示例登录失败的原因。 */
  ret = tp_sdk_init(NULL, NULL, NULL, NULL, -1, -1);
  if (ret != 0) {
    int errcode = tp_get_errorcode(ret);
    fprintf(stderr, "[init] tp_sdk_init failed %d: %s\n", errcode, tp_get_errormessage(ret));
    if (errcode == 2357) {
      fprintf(stderr, "[init] 配置文件 %s 不存在（路径相对当前工作目录）\n", g_opt.config_path);
    } else if (errcode == 2300 || errcode == 2302 || errcode == 2304) {
      fprintf(stderr, "[init] 请在 %s 中填写 account / password / mount_point\n", g_opt.config_path);
    }
    (void)tp_sdk_cleanup();
    return 1;
  }

  /* 这里故意不调用 tp_sdk_config：接口配置优先级高于配置文件，一旦调用就会把
   * 配置文件里的 block_time、loglevel 等覆盖掉。需要代码内改配置时，应先
   * tp_get_current_config 拿到当前值再改动个别字段（文档 §4.4.2）。 */
  print_effective_config();

  if (g_opt.mock_gga == NULL) {
    g_tty_fd = tty_open(g_opt.tty_dev, g_opt.tty_baud);
    if (g_tty_fd < 0) {
      (void)tp_sdk_cleanup();
      return 1;
    }
    printf("[init] gnss tty %s @ %ld opened\n", g_opt.tty_dev, g_opt.tty_baud);
  } else {
    printf("[init] mock gga mode, tty disabled\n");
  }

  pthread_t rtcm_thread;
  int thread_started = 0;
  if (pthread_create(&rtcm_thread, NULL, rtcm_thread_main, NULL) != 0) {
    fprintf(stderr, "[init] create rtcm thread failed: %s\n", strerror(errno));
  } else {
    thread_started = 1;
  }

  int exit_code = 0;
  int connected_once = 0;
  unsigned long long last_gga_ms = 0;
  char gga[GGA_LINE_SIZE];

  while (!g_stop && thread_started) {
    if (g_need_reconnect) {
      g_gga_sent = 0;
      /* 仅在确实连过之后才 stop：首次连接前调用会让 SDK 打出
       * "private disconnect rejected: service not init" 的无用告警 */
      if (connected_once) {
        (void)tp_sdk_stop();
      }
      ret = tp_connect_service(NULL, 0);
      if (ret != 0) {
        int errcode = tp_get_errorcode(ret);
        fprintf(stderr, "[net] connect failed %d: %s\n", errcode, tp_get_errormessage(ret));
        if (is_fatal_platform_error(errcode)) {
          fprintf(stderr, "[net] 账号类错误，不再重试\n");
          exit_code = 1;
          break;
        }
        /* 6114 账号已在别处登录，等久一点让服务端超时释放 */
        sleep_ms(errcode == TP_SDK_ACCOUNT_HAS_LOGIN ? 5000 : 1000);
        continue;
      }
      printf("[net] connected, network fd %d\n", tp_get_network_fd(0));
      connected_once = 1;
      g_need_reconnect = 0;
      last_gga_ms = 0;
    }

    int gga_len = 0;
    if (g_opt.mock_gga != NULL) {
      size_t n = strlen(g_opt.mock_gga);
      if (n >= sizeof(gga)) {
        n = sizeof(gga) - 1;
      }
      memcpy(gga, g_opt.mock_gga, n);
      gga[n] = '\0';
      gga_len = (int)n;
      sleep_ms(100);
    } else {
      gga_len = read_gga(g_tty_fd, gga, sizeof(gga), 200);
      if (gga_len < 0) {
        exit_code = 1;
        break;
      }
    }

    unsigned long long now = now_ms();
    if (gga_len > 0 &&
      now >= last_gga_ms + (unsigned long long)g_opt.gga_interval_s * 1000ULL)
    {
      last_gga_ms = now;
      ret = tp_send_gga((const INT8 *)gga, gga_len);
      if (ret == 0) {
        g_gga_sent = 1;
        g_gga_sent_count++;
        if (g_opt.verbose) {
          printf("[gga] sent %d bytes: %s", gga_len, gga);
        }
      } else {
        int errcode = tp_get_errorcode(ret);
        fprintf(stderr, "[gga] tp_send_gga failed %d: %s\n", errcode, tp_get_errormessage(ret));
        if (is_fatal_platform_error(errcode)) {
          fprintf(stderr, "[gga] 账号类错误，不再重试\n");
          exit_code = 1;
          break;
        }
        if (errcode == TP_SDK_NET_NETWORK_ERROR || errcode == TP_SDK_NET_TIMEOUT) {
          g_need_reconnect = 1;
        }
      }
    }
  }

  printf("\n[exit] shutting down\n");
  g_stop = 1;
  if (thread_started) {
    /* tp_get_rtcm 最长阻塞 block_time 秒，join 可能要等这么久 */
    (void)pthread_join(rtcm_thread, NULL);
  } else {
    exit_code = 1;
  }
  /* 统计放在 join 之后，保证是真正的最终值且是最后一行输出 */
  printf(
    "[exit] total rtcm %llu B / %lu pkt, gga %lu sent\n",
    g_rtcm_bytes, g_rtcm_packets, g_gga_sent_count);
  (void)tp_sdk_stop();
  (void)tp_sdk_cleanup();
  if (g_tty_fd >= 0) {
    (void)close(g_tty_fd);
  }
  return exit_code;
}

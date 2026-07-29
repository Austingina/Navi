#ifndef TP_SDK_API_H
#define TP_SDK_API_H

/**
 * Copyright © 2025 TruePoint Technology Inc. All rights reserved.
 *
 * All the code, data, software, text, documentation, API and other TruePoint Technology Inc.'s
 * material cannot be used, or spread, or reproduced, or modified under any condition in any form
 * or means without TruePoint Technology Inc.'s prior official authorization.
 */

#ifdef __cplusplus
extern "C"
{
#endif

#include "GH_GSP_Consts.h"

#ifndef SDK_MAX_BUF
#define SDK_MAX_BUF 1024
#endif

#ifndef TP_PARAM_BUF
#define TP_PARAM_BUF 64
#endif

#ifndef TP_PATH_MAX
#define TP_PATH_MAX 256
#endif

    /* 日志级别枚举 */
    typedef enum
    {
        LOG_LEVEL_VERBOSE = 0, /*verbose*/
        LOG_LEVEL_DEBUG = 1,   /*debug*/
        LOG_LEVEL_INFO = 2,    /*info*/
        LOG_LEVEL_WARNING = 3, /*warning*/
        LOG_LEVEL_ERROR = 4,   /*error*/
        LOG_LEVEL_NONE = 5     /*none*/
    } TP_LOG_LEVEL;

    /**
     * @brief 服务的类型
     * Firmware
     */
    typedef enum
    {
        SERVICE_COMMON = 0,
        SERVICE_NRTK = 1,
        SERVICE_PPPRTK = 2,
        SERVICE_RTD = 3,
        SERVICE_EPH = 4,
        SERVICE_IONOSPHERE = 5,
        SERVICE_REMOTE_DIAGNOSIS = 6,
        SERVICE_REMOTE_UPGRADE = 7,
        SERVICE_LOCAL_LOG = 8,
    } TP_SERVICE_TYPE;

    /**
     * @brief Transport Layer Interface (TLI) type
     *
     */
    typedef enum
    {
        TLI_UNSPEC = 0x00,
        TLI_TCP = 0x01,
        TLI_UDP = 0x02,
        TLI_SSL = 0x04,
        TLI_MPTCP = 0x08,
        TLI_QUIC = 0x10,
    } TP_TLI_PROTOCOL;

    /**
     * @brief TLS 配置
     *
     */
    typedef struct
    {
        UINT32 ssl_option; /* TLS配置项 TP_SSL_OPTION 组合*/
        INT8 ssl_ca_path[TP_PATH_MAX];
    } St_ssl_config_t;

    /**
     * @brief TLS 选项
     *
     */
    typedef enum
    {
        SSL_NONE = 0,
        SSL_CONFIG = 0x01,
        SSL_ENABLED = 0x02,
        SSL_VALIDATE_CERTS = 0x04,
        SSL_VALIDATE_CN = 0x08,
    } TP_SSL_OPTION;

    /**
     * @brief 配置选项
     *
     */
    typedef struct
    {
        /* 版本号，用于向前兼容 */
        UINT32 version;

        /* 认证信息（必填） */
        INT8 account[TP_PARAM_BUF];
        INT8 password[TP_PARAM_BUF];
        INT8 mountpoint[TP_PARAM_BUF];

        /* 服务器配置 */
        INT8 server_host[TP_PARAM_BUF];
        UINT32 server_port;
        UINT32 server_type; /* 服务类型 TP_SERVICE_TYPE, 目前仅支持 SERVICE_NRTK */

        /* 数据间隔配置 */
        INT32 rtcm_interval; /* 服务端数据下发时间间隔，秒 */

        /* 其他配置 */
        INT8 device_id[TP_PARAM_BUF]; /* 设备ID */
        INT32 coordsys;               /* 坐标系 */
        INT32 block_time;             /* 网络传输数据最大等待时间，秒 */
        INT32 send_gga_with_retry;    /* 是否在发送GGA时进行重试 */
        INT32 retry_interval;         /* 重试间隔时间，秒 */
        INT32 retry_number;           /* 重试次数 */
        /* 日志配置 */
        UINT32 loglevel;

        /*认证类型*/
        INT32 authType;

        /* Transport Layer Interface 配置*/
        UINT32 tli_protocol;  // TP_TLI_PROTOCOL
        St_ssl_config_t ssl_config;
        INT32
            support_gga_offset; /* 是否启用 GGA 位置随机偏移：0 关闭，1 开启（见 tpservice.conf） */
    } TPConfigFull_t;

    typedef TPConfigFull_t TPconfig;

    /**
     * @brief 播发频率间隔
     *
     */
    typedef enum
    {
        TP_Freq0_AUTO = 0, // sdk智能指定
        TP_Freq1 = 1,      // 1s
        TP_Freq2 = 10,     // 10s
        TP_Freq3 = 15,     // 15s
        TP_Freq4 = 30,     // 30s
        TP_Freq5 = 45,     // 45s
        TP_Freq6 = 60,     // 60s
    } TP_Rtcm_Freq;

    /**
     * @brief 鉴权方式
     */
    typedef enum
    {
        TP_CLOUD_CHIP_PERSONAL = 2002,    // personal account
        TP_CLOUD_CHIP_DEVICEID = 2002,    // device id
        TP_CLOUD_CHIP_MOBILEPHONE = 2002, // mobilephone pool
        TP_CLOUD_CHIP_TIMEACCOUNT = 2002, // time account
        TP_CLOUD_CHIP_CC = 2002,          // 云芯一体
        TP_CLOUD_CHIP_NTRIP = 2002,       // Ntrip
        TP_CLOUD_CHIP_PRIVATE = 2002,     // True-Point Private
        TP_CLOUD_CHIP_AKAS = 2003,
        TP_AUTHTYPE_A = TP_CLOUD_CHIP_PERSONAL,
        TP_AUTHTYPE_B = TP_CLOUD_CHIP_DEVICEID,
        TP_AUTHTYPE_C = TP_CLOUD_CHIP_MOBILEPHONE,
        TP_AUTHTYPE_E = TP_CLOUD_CHIP_TIMEACCOUNT,
    } TP_AUTHTYPE;

    /**
     * @brief 可更新的类型
     *
     */
    typedef enum
    {
        UPGRADE_AUTO = 1,
        UPGRADE_WITH_INFO = 2,
        UPGRADE_WITH_TPPKG = 3,
    } TP_UPDATE_ID;

    /**
     * @brief 可查询的类型
     * Firmware
     */
    typedef enum
    {
        GET_ACCOUND_INFO = 0,
        GET_LICENSE_ALG = 1,
        GET_SDK_VERSION = 2,
        GET_FIRMWARE_VERSION = 3,
        GET_RIRMWARE_UPGRADE_INFO = 4,
    } TP_QUERY_ID;

    struct common_result
    {
        INT8 buf[SDK_MAX_BUF];
    };

    union union_result
    {
        INT32 result;
        struct common_result com_result;
    };

    /**
     * @brief 返回类型
     *
     */
    typedef struct
    {
        INT32 type;
        INT32 len;
        union union_result uResult;

    } St_sdk_response_t;

    struct common_params
    {
        INT8 buf[SDK_MAX_BUF];
    };

    union union_param
    {
        INT32 iParams;
        struct common_params com_params;
    };

    /**
     * @brief 预留参数
     *
     */
    typedef struct
    {
        INT32 len;
        union union_param uParam;

    } St_sdk_reserved_t;

    /**
     * @brief 设置配置文件路径，在tp_sdk_init之前配置
     *
     * @param path ：配置文件路径
     * @return INT32 ：失败返回错误码，成功返回0
     */
    INT32 tp_set_config_path(const INT8* path);

    /**
     * @brief 参数初始化
     *
     * @param user ：账户名
     * @param pwd ： 密码
     * @param device_id ：设备ID
     * @param mountpoint ：挂载点
     * @param interval ： 播发间隔
     * @param authtype ： 鉴权方式
     * @return INT32 ： 失败返回错误码，成功返回0
     */
    INT32 tp_sdk_init(const INT8 *user, const INT8 *pwd, const INT8 *device_id, const INT8 *mountpoint, INT32 interval, TP_AUTHTYPE authtype);

    /**
     * @brief 通用功能配置，tp_sdk_init之后，tp_connect_service之前使用
     *
     * @param config ：配置参数
     * @return INT32 ：失败返回错误码，成功返回0
     */
    INT32 tp_sdk_config(const TPconfig *config);
    /**
     * @brief 获取当前 SDK 正在使用的配置。 tp_sdk_init后使用
     *
     * 将当前 SDK 内部配置拷贝到调用方提供的缓冲区中。
     *
     * @param config ：调用方提供的 TPconfig 缓冲区，不可为 NULL。
     * @return INT32 ：成功返回 0；config 为 NULL 返回错误码；未初始化时返回 TP_SDK_NOT_INIT。
     */
    INT32 tp_get_current_config(TPconfig *config);
#ifndef tp_get_config
#define tp_get_config tp_get_current_config
#endif

    /**
     * @brief 连接服务,默认使用tp_sdk_config时指定的超时时间
     *
     * @param addr : 服务的域名或者IP
     * @param port : 端口(坐标系类型)
     *
     * @return INT32 : 失败返回错误码，成功返回0
     */
    INT32 tp_connect_service(const INT8 *addr, const INT32 port);

    /**
     * @brief 发送gga数据
     *
     * @param buf : gga数据
     * @param len : gga数据长度
     * @return INT32 : 失败返回错误码，成功返回0
     */
    INT32 tp_send_gga(const INT8 *buf, INT32 len);

    /**
     * @brief 获取rtcm改正数
     *
     * @param buf : 存放rtcm数据
     * @param len : 获取到的rtcm数据长度
     * @return INT32 : 失败返回错误码，成功返回0
     */
    INT32 tp_get_rtcm(INT8 *buf, INT32 *len);

    /**
     * @brief 资源释放
     *
     * @return INT32 失败返回错误码，成功返回0
     */
    INT32 tp_sdk_cleanup(void);

    /**
     * @brief 断开服务连接
     *
     * @return INT32 : 失败返回错误码，成功返回0
     */
    INT32 tp_sdk_stop(void);

    /**
     * @brief 获取网络文件描述符
     *
     * @param fdType 网络类型
     * @return INT32 : 失败返回错误码，成功返回文件描述符
     */
    INT32 tp_get_network_fd(INT32 fdType);

    /**
     * @brief sdk查询
     *
     * @param id : 查询类型
     * @param stResponse : 返回结果
     * @param reserved : 预留位，默认为null
     * @return INT32 : 失败返回错误码，成功返回0
     */
    INT32 tp_sdk_query(TP_QUERY_ID id, St_sdk_response_t *stResponse, St_sdk_reserved_t *reserved);

    /**
     * @brief 注册socket配置回调，用于对socket进行配置
     *
     * @param pcb_fun ：回调函数，调用时传入socket描述符
     * @return void ：无返回值
     */
    void tp_set_socket_cb(void(*pcb_fun)(int fd));

    /**
     * @brief 注册日志输出回调；传入 NULL 则不进行输出。
     *
     * 回调在每条通过过滤的日志格式化完成后调用，参数为：时间戳、级别、标签、完整格式化后的单行消息缓冲区（只读使用，勿长期保存指针）。
     *
     * @param callback 回调函数。
     */
     void tp_set_log_cb(void (*callback)(UINT64 timestamp, INT32 level, const char *tag, const char *message));

     /**
      * @brief 兼容占位：链式日志下不再使用变参 formatter；仅接受 NULL。
      *
      * @param reserved_null_only 必须为 NULL。
      */
     void tp_set_log_formatter(const void *reserved_null_only);

#ifdef __cplusplus
}
#endif

#endif /* TP_SDK_API_H */

#ifndef TP_SDK_STATUS_CODE_H
#define TP_SDK_STATUS_CODE_H

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

    typedef enum
    {
        // 0-999        系统错误吗
        // 1000-1099    SDK通用错误码
        // 2000-2999    SDK功能错误码
        // 3000-3999    SDK OSAL错误码
        // 4000-4999    SDK Service错误码
        // 6000-6999    映射平台错误码

        TP_SDK_FUNC_INVOKE_OK = 0,                       // 执行成功
        TP_SDK_FUNC_INVOKE_ERROR = 1010,                 // 执行错误
        TP_SDK_FUNC_PARA_ERROR = 1020,                   // 参数错误
        TP_SDK_FUNC_MISSING_TLS = 1021,                  // 需要打开TLS配置
        TP_SDK_FUNC_UNKNOW_ERROR = 1999,                 // SDK未知错误
        
        TP_SDK_NOT_INIT = 2000,                          // SDK 尚未初始化
        TP_SDK_ALREADY_INIT = 2001,                      // SDK 已经初始化
        TP_SDK_PROTOBUF_MEMORY_NOT_MEET = 2010,          // Protobuf传入内存不足
        TP_SDK_PROTOBUF_INVOKE_ERROR = 2020,             // Protobuf处理数据错误
        TP_SDK_PROTOBUF_ERROR_RESPONSE = 2022,           // Protobuf处理数据错误：解析服务响应
        TP_SDK_PROTOBUF_ERROR_NRTK_ENABLE = 2024,        // Protobuf处理数据错误：封装开启NRTK服务请求
        TP_SDK_PROTOBUF_ERROR_GGA = 2026,                // Protobuf处理数据错误：封装上报GGA
        TP_SDK_PROTOBUF_ERROR_RTCM = 2028,               // Protobuf处理数据错误：解析差分数据
        TP_SDK_PROTOBUF_ERROR_LOGIN = 2030,              // Protobuf处理数据错误：封装登录请求
        TP_SDK_PROTOBUF_ERROR_SALT = 2032,               // Protobuf处理数据错误：获取随机数
        TP_SDK_MSM4_MEMORY_NOT_MEET = 2100,              // MSM4传入内存不足
        TP_SDK_SHA256_ERROR = 2200,                      // SHA256错误
        TP_SDK_CONFIG_ACCOUNT_EMPTY = 2300,              // 账号为空
        TP_SDK_CONFIG_PASSWORD_EMPTY = 2302,             // 密码为空
        TP_SDK_CONFIG_MOUNTPOINT_EMPTY = 2304,           // 挂载点为空
        TP_SDK_CONFIG_SERVER_EMPTY = 2306,               // 服务地址为空或端口无效
        TP_SDK_CONFIG_BLOCKTIME_LESS_FREQ = 2308,        // 阻塞时间小于播发间隔
        TP_SDK_CONFIG_PORT_PROTOCOL_MISMATCH = 2309,   // 服务端口与传输协议不匹配（如 TLS 须配合约定端口）
        TP_SDK_CONFIG_LOAD_FAILED = 2310,                // 加载配置文件失败
        TP_SDK_CONFIG_INVALID_VALUE = 2312,              // 无效配置
        TP_SDK_CONFIG_INVALID_SUPPORT_GGA_OFFSET = 2313, // 无效配置：支持GGA偏移计算
        TP_SDK_CONFIG_INVALID_LOG_LEVEL = 2314,          // 无效配置：日志等级
        TP_SDK_CONFIG_INVALID_SERVER_ADDRESS = 2316,     // 无效配置：服务地址
        TP_SDK_CONFIG_INVALID_MSM2_SUPPORT = 2317,       // 无效配置：MSM2支持
        TP_SDK_CONFIG_INVALID_TRANSPORT_PROTOCOL = 2318, // 无效配置：传输协议
        TP_SDK_CONFIG_INVALID_SSL_OPTION = 2320,         // 无效配置：SSL验证项
        TP_SDK_CONFIG_INVALID_SSL_CERTPATH = 2322,       // 无效配置：SSL证书路径
        TP_SDK_CONFIG_INVALID_ACCOUNT = 2324,            // 无效配置：账号
        TP_SDK_CONFIG_INVALID_PASSWORD = 2326,           // 无效配置：密码
        TP_SDK_CONFIG_INVALID_MOUNTPOINT = 2328,         // 无效配置：挂载点
        TP_SDK_CONFIG_INVALID_DEVICEID = 2330,           // 无效配置：设备ID
        TP_SDK_CONFIG_INVALID_GPSDEV_PATH = 2332,        // 无效配置：GPS设备路径
        TP_SDK_CONFIG_INVALID_GPSDEV_BAUDRATE = 2334,    // 无效配置：GPS设备波特率
        TP_SDK_CONFIG_INVALID_RTCM_INTERVAL = 2336,      // 无效配置：差分数据播发间隔
        TP_SDK_CONFIG_INVALID_COORDSYS = 2338,           // 无效配置：坐标系
        TP_SDK_CONFIG_INVALID_BLOCKTIME = 2340,          // 无效配置：网络阻塞等待时间
        TP_SDK_CONFIG_INVALID_SERVER_TYPE = 2342,        // 无效配置：服务类型
        TP_SDK_CONFIG_INVALID_RETRY_WITH_GGA = 2344,     // 无效配置：自动重连
        TP_SDK_CONFIG_INVALID_RETRY_INTERVAL = 2346,     // 无效配置：重连间隔
        TP_SDK_CONFIG_INVALID_AUTH_TYPE = 2348,          // 无效配置：认证方式
        TP_SDK_CONFIG_INVALID_GGA_INTERVAL = 2350,       // 无效配置：GGA上报间隔
        TP_SDK_CONFIG_INVALID_RETRY_NUMBER = 2352,       // 无效配置：重试次数
        TP_SDK_CONFIG_DUPLICATE = 2354,                  // 服务已连接，配置无效
        TP_SDK_CONFIG_INVALID_CONFIG_PATH = 2356,        // 已经初始化，配置路径无效
        TP_SDK_CONFIG_CONFIG_PATH_NOT_FOUND = 2357,        // 配置路径不存在
        TP_SDK_CONFIG_INVALID_CONFIG_PATH_TOO_LONG = 2358, // 配置路径过长
        TP_SDK_CONFIG_MISSING_TLS = TP_SDK_FUNC_MISSING_TLS,    // 需要打开TLS配置
        TP_SDK_MUTEX_LOCK_FAILED = 2500,                 // 获取资源锁失败
        TP_SDK_RECONNECT_BEYOND_MAX = 2503,              // 重连次数超过最大重连次数
        
        TP_SDK_NET_NETWORK_ERROR = 3010,                 // 网络错误
        TP_SDK_NET_CONNECT_ERROR = 3020,                 // 连接错误
        TP_SDK_NET_NOT_CONNECT = 3030,                   // TCP链接尚在建立中
        TP_SDK_NET_ALREADY_CONNECTED = 3031,             // 连接已建立
        TP_SDK_NET_TIMEOUT = 3040,                       // 超时
        TP_SDK_NET_PROTOCOL_UNSUPPORT = 3050,            // 不支持的传输协议
        TP_SDK_NET_CONNECTION_INTERRUPTED = 3060,        // 连接被中断
        TP_SDK_NET_NOT_CONNECTED = 3061,                 // 服务未连接
        TP_SDK_NET_NEED_RETRY = 3070,                    // 需要重新尝试接收或发送数据
        TP_SDK_NET_DNS_RESOLVE_ERROR = 3080,             // DNS解析错误
        TP_SDK_SSL_SETUP_ERROR = 3400,                   // SSL初始化错误
        TP_SDK_SSL_HANDSHAKE_IN_PROGRESS = 3410,         // SSL握手进行中
        TP_SDK_SSL_HANDSHAKE_ERROR = 3420,               // SSL握手错误
        TP_SDK_SSL_LOAD_CERT_ERROR = 3430,               // SSL加载证书错误
        TP_SDK_SSL_CONTEXT_INVALID = 3440,               // SSL上下文无效
        TP_SDK_SSL_ERRORP_SSL = 3450,                    // SSL内部错误
        TP_SDK_SSL_SYSCALL_ERROR = 3460,                 // SSL底层系统调用错误
        TP_SDK_OSAL_UNKNOW_ERROR = 3999,                 // OSAL未知错误
     
        TP_SDK_SERV_TYPE_INVALID = 4000,                 // 服务类型无效
        TP_SDK_SERV_NOT_READY = 4005,                    // 服务尚未准备好，（还未完成登录激活）
        TP_SDK_SERV_NOT_INIT = 4011,                     // private 服务未初始化（参见 private-state.md）
        TP_SDK_SERV_ALREADY_INIT = 4012,                  // private 服务已初始化，禁止重复 init（拼写与文档一致）
        TP_SDK_SERV_ALREADY_LOGIN = 4013,                // 已登录，禁止重复登录
        TP_SDK_SERV_ALREADY_CONNECTED = 4014,            // 已激活连接，禁止重复 connect
        TP_SDK_SERV_NOT_CONNECTED = 4015,                // 未激活，禁止 send/recv
        TP_SDK_SERV_SEND_AUTH_ERROR = 4010,              // 发送认证信息错误
        TP_SDK_SERV_GGA_INVALID = 4020,                  // 无效GGA
        TP_SDK_SERV_SEND_GGA_ERROR = 4030,               // 发送GGA错误
        TP_SDK_SERV_RECV_RTCM_TIMEOUT = 4031,            // 接收RTCM超时
        TP_SDK_SERV_WRAP_ERROR = 4040,                   // 封装错误
        TP_SDK_SERV_FREQ_ILLEGAL = 4100,                 // 不支持的播发间隔
        TP_SDK_SERV_BLOCKTIME_LESS_FREQ = 4110,          // 阻塞时间小于播发间隔
        TP_SDK_SERV_COORDINATE_INVALID = 4120,           // 坐标无效
        TP_SDK_SERV_PARAM_TOOLONG = 4125,                // 参数过长
        TP_SDK_SERV_INVALID_STATE = 4130,                // 状态无效
        TP_SDK_SERV_ALREADY_TERMINATED = 4135,           // 服务已终止
        TP_SDK_SERV_INVALID_SERVER_ADDRESS= 4140,        // 无效的服务地址
        TP_SDK_SERV_INVALID_CONFIG = 4145,               // 无效的配置
        TP_SDK_SERV_INVALID_FRAME = 4150,                // 无效的帧
        TP_SDK_SERV_EPOLL_ERROR = 4200,                  // epoll错误
        TP_SDK_SERV_EPOLL_ADD_ERROR = 4205,              // epoll添加事件错误
        TP_SDK_SERV_EPOLL_DEL_ERROR = 4210,              // epoll删除事件错误
        TP_SDK_SERV_EPOLL_WAIT_ERROR = 4215,              // epoll等待事件错误
        TP_SDK_SERV_EPOLL_CREATE_ERROR = 4220,           // epoll创建错误
        TP_SDK_SERV_IPC_ERROR = 4250,                    // IPC错误
        TP_SDK_SERV_IPC_FRAME_ERROR = 4255,              // IPC发送接收错误
        TP_SDK_SERV_IPC_POLL_ERROR = 4260,               // IPC轮询错误
        TP_SDK_SERV_IPC_READ_ERROR = 4265,               // IPC读取错误
        TP_SDK_SERV_IPC_WRITE_ERROR = 4245,               // IPC写入错误
        TP_SDK_SERV_IPC_CREATE_ERROR = 4270,              // IPC创建错误
        TP_SDK_SERV_DNS_ERROR = 4300,                    // DNS错误
        TP_SDK_SERV_DNS_FRAME_ERROR = 4305,              // DNS发送接收错误
        TP_SDK_SERV_DNS_THREAD_START_ERROR = 4310,       // DNS线程启动错误
        TP_SDK_SERV_WORKER_ERROR = 4350,                 // 未处理的命令码
        TP_SDK_SERV_WORKER_START_ERROR = 4355,           // 工作线程启动错误
        TP_SDK_SERV_WORKER_UNHANDLED_OPCODE = 4360,      // 未处理的命令码
        TP_SDK_SERV_WORKER_INVALID_FRAME = 4365,         // 无效的帧
        TP_SDK_SERV_UNKNOW_ERROR = 4999,                 // 未知服务错误
       
        TP_SDK_ACCOUNT_OR_PASSWORD_ERROR = 6100,         // 账号密码错误
        TP_SDK_ACCOUNT_TYPE_ERROR = 6105,                // 账号类型错误
        TP_SDK_ACCOUNT_POOL_USEUP = 6106,                // 账号到期或者资源耗尽
        TP_SDK_ACCOUNT_HAS_LOGIN = 6114,                 // 账号已经登录，重复登录
        TP_SDK_ACCOUNT_NO_ASSIGNABLE = 6116,             // 没有可分配的账号
        TP_SDK_ACCOUNT_MOUNTPOINT_INVALID = 6305,        // 接入点不存在
        TP_SDK_ACCOUNT_NOT_ACTIVE = 6311,                // 账号未激活
        TP_SDK_ACCOUNT_HAS_PROHIBIT = 6330,              // 账号已经禁用
        TP_SDK_SERVICE_NOT_COVER = 6046,                 // 服务未覆盖
        TP_SDK_ACCOUNT_EXPIRE = 6510,                    // 请求已过期，请重新连接
        TP_SDK_CLOUD_STATE_MIN = 6000,                   // 平台返回码的最小值
        TP_SDK_CLOUD_STATE_MAX = 6999,                   // 平台返回码的最大值
        
        TP_SDK_UNKNOWN_ERROR = 9999,                     // 未知错误

        // 兼容旧版本（V2），已废弃
        TP_SDK_FUNC_INVOKE_NO_CONNECT = TP_SDK_NET_NOT_CONNECT,
        TP_CONNECT_ERROR = TP_SDK_NET_CONNECT_ERROR,
        TP_NET_WORK_ERROR = TP_SDK_NET_NETWORK_ERROR,
        TP_SEND_GGA_ERROR = TP_SDK_SERV_SEND_GGA_ERROR,
        TP_GGA_INVALID_ERROR = TP_SDK_SERV_GGA_INVALID,
        TP_SEND_AUTH_ERROR = TP_SDK_SERV_SEND_AUTH_ERROR,
        TP_SDK_BLOCKTIME_LESS_FREQ = TP_SDK_SERV_BLOCKTIME_LESS_FREQ,
        TP_SDK_COORDINATE_INVALID = TP_SDK_SERV_COORDINATE_INVALID,
        TP_SDK_PARAMS_ILLEGAL = TP_SDK_SERV_UNKNOW_ERROR,
        TP_SDK_MOUNTPOINT_INVALID = TP_SDK_ACCOUNT_MOUNTPOINT_INVALID,
        TP_UNKNOWN_ERROR = TP_SDK_UNKNOWN_ERROR,

    } St_tp_status_code_t;

    /**
     * @brief 从 API 返回值中取出「语义错误码」（与 @ref St_tp_status_code_t 中数值同量级）。
     *
     * 对 @ref tp_make_returncode / @ref tp_make_returncode_ext 等生成的打包负数返回值，取绝对值后对 10000 取模；
     * 非负返回值则直接对 10000 取模。用于与枚举常量或云端子码比较。
     *
     * @param retcode 某次调用的原始返回 int（可为打包负数或普通错误码）。
     * @return 落在实现约定范围内的错误码数值（成功场景通常为 0）。
     */
    int tp_get_errorcode(int retcode);

    /**
     * @brief 根据返回值解析人类可读错误说明字符串。
     *
     * 若 @a retcode 解出的错误码与当前线程最近一次 @ref tp_make_returncode_imp 存储的码一致，
     * 且存在扩展消息（例如通过 @ref tp_make_returncode_ext 写入），则返回该线程局部缓冲中的文本；
     * 否则在静态表中查找 @ref St_tp_status_code_t 对应说明，云端区间 \ref TP_SDK_CLOUD_STATE_MIN～\ref TP_SDK_CLOUD_STATE_MAX
     * 返回固定提示，未知码返回通用「Unknown error」类文案。返回指针指向只读/线程局部数据，勿 @c free。
     *
     * @param retcode 与 @ref tp_get_errorcode 相同的原始返回 int。
     * @return 以 @c '\\0' 结尾的说明字符串，生命周期由实现管理。
     */
    const char *tp_get_errormessage(int retcode);

#ifdef __cplusplus
}
#endif

#endif /* TP_SDK_STATUS_CODE_H */

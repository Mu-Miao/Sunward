#ifndef CAR_BUILD_H
#define CAR_BUILD_H

/* 可读版本名固定描述控制方案；Git/配置指纹由tools/build_firmware.py注入。
 * IDE直接编译时使用LOCAL占位，避免把未知版本伪装成正式构建。 */
#define CAR_FIRMWARE_VERSION "ARC16"

#ifndef CAR_BUILD_GIT
#define CAR_BUILD_GIT "LOCAL"
#endif

#ifndef CAR_CONFIG_CRC
#define CAR_CONFIG_CRC "00000000"
#endif

#ifndef CAR_BUILD_MODE
#define CAR_BUILD_MODE "LOCAL"
#endif

#endif

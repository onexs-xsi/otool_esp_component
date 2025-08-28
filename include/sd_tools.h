/*
 * SPDX-FileCopyrightText: 2025 exia
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __SD_TOOLS_H__
#define __SD_TOOLS_H__

#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_timer.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

// SD卡测试模式枚举
typedef enum {
    SD_MODE_MMC_1BIT = 0,  // MMC 1位模式
    SD_MODE_MMC_4BIT = 1,  // MMC 4位模式 (注意：当前硬件仅支持1位，会回退到1位模式)
    SD_MODE_SPI = 2        // SPI 模式
} sd_test_mode_t;

#ifdef __cplusplus
}
#endif

/**
 * @brief sd_tools 类
 * 
 * 提供 SD 卡操作的功能，支持多种工作模式和配置
 */
class sd_tools {
private:
    // SD卡状态管理
    bool initialized;                    ///< SD卡初始化状态
    bool mounted;                        ///< 文件系统挂载状态
    sdmmc_card_t *card;                  ///< SD卡信息指针
    sd_test_mode_t current_mode;         ///< 当前工作模式
    
    // 配置参数
    sd_test_mode_t mode;                 ///< SD卡工作模式
    std::string mount_point;             ///< 挂载点路径
    uint32_t max_freq_khz;               ///< 最大频率 (kHz)
    uint16_t max_files;                  ///< 最大文件数
    uint32_t allocation_unit_size;       ///< 分配单元大小
    bool format_if_mount_failed;         ///< 挂载失败时是否格式化
    
    // MMC模式引脚配置
    gpio_num_t mmc_clk_pin;              ///< MMC CLK引脚
    gpio_num_t mmc_cmd_pin;              ///< MMC CMD引脚
    gpio_num_t mmc_d0_pin;               ///< MMC D0引脚
    gpio_num_t mmc_d1_pin;               ///< MMC D1引脚 (4位模式)
    gpio_num_t mmc_d2_pin;               ///< MMC D2引脚 (4位模式)
    gpio_num_t mmc_d3_pin;               ///< MMC D3引脚 (4位模式)
    bool mmc_internal_pullup;            ///< MMC内部上拉使能
    
    // SPI模式引脚配置
    gpio_num_t spi_cs_pin;               ///< SPI CS引脚
    gpio_num_t spi_mosi_pin;             ///< SPI MOSI引脚
    gpio_num_t spi_miso_pin;             ///< SPI MISO引脚
    gpio_num_t spi_clk_pin;              ///< SPI CLK引脚
    spi_host_device_t spi_host;          ///< SPI主机设备
    uint32_t spi_max_transfer_sz;        ///< SPI最大传输大小
    
    // 内部辅助函数
    esp_err_t init_mmc_mode();           ///< 初始化MMC模式
    esp_err_t init_spi_mode();           ///< 初始化SPI模式
    esp_err_t cleanup_spi_mode();        ///< 清理SPI模式资源
    bool is_4bit_supported() const;      ///< 检查是否支持4位模式

public:
    /**
     * @brief 构造函数
     * 
     * 使用默认配置创建 sd_tools 对象
     */
    sd_tools();

    /**
     * @brief 构造函数（带模式参数）
     * 
     * @param mode SD卡工作模式
     */
    sd_tools(sd_test_mode_t mode);

    /**
     * @brief 析构函数
     * 
     * 销毁 sd_tools 对象，自动清理资源
     */
    ~sd_tools();

    // === 配置方法 (在初始化前调用) ===
    
    /**
     * @brief 设置SD卡工作模式
     * 
     * @param mode SD卡工作模式
     */
    void set_mode(sd_test_mode_t mode);

    /**
     * @brief 获取当前工作模式
     * 
     * @return sd_test_mode_t 当前工作模式
     */
    sd_test_mode_t get_mode() const;

    /**
     * @brief 设置挂载点路径
     * 
     * @param mount_point 挂载点路径
     */
    void set_mount_point(const char* mount_point);

    /**
     * @brief 获取挂载点路径
     * 
     * @return const char* 挂载点路径
     */
    const char* get_mount_point() const;

    /**
     * @brief 设置最大工作频率
     * 
     * @param freq_khz 最大频率 (kHz)
     */
    void set_max_frequency(uint32_t freq_khz);

    /**
     * @brief 获取最大工作频率
     * 
     * @return uint32_t 最大频率 (kHz)
     */
    uint32_t get_max_frequency() const;

    /**
     * @brief 设置文件系统配置
     * 
     * @param max_files 最大文件数
     * @param allocation_unit_size 分配单元大小
     * @param format_if_mount_failed 挂载失败时是否格式化
     */
    void set_filesystem_config(uint16_t max_files, uint32_t allocation_unit_size, bool format_if_mount_failed);

    /**
     * @brief 设置MMC模式引脚配置
     * 
     * @param clk_pin CLK引脚
     * @param cmd_pin CMD引脚
     * @param d0_pin D0引脚
     * @param internal_pullup 是否启用内部上拉
     */
    void set_mmc_pins(gpio_num_t clk_pin, gpio_num_t cmd_pin, gpio_num_t d0_pin, bool internal_pullup = true);

    /**
     * @brief 设置MMC 4位模式引脚配置
     * 
     * @param clk_pin CLK引脚
     * @param cmd_pin CMD引脚
     * @param d0_pin D0引脚
     * @param d1_pin D1引脚
     * @param d2_pin D2引脚
     * @param d3_pin D3引脚
     * @param internal_pullup 是否启用内部上拉
     */
    void set_mmc_4bit_pins(gpio_num_t clk_pin, gpio_num_t cmd_pin, gpio_num_t d0_pin,
                          gpio_num_t d1_pin, gpio_num_t d2_pin, gpio_num_t d3_pin, 
                          bool internal_pullup = true);

    /**
     * @brief 设置SPI模式引脚配置
     * 
     * @param cs_pin CS引脚
     * @param mosi_pin MOSI引脚
     * @param miso_pin MISO引脚
     * @param clk_pin CLK引脚
     * @param host SPI主机设备
     * @param max_transfer_sz 最大传输大小
     */
    void set_spi_pins(gpio_num_t cs_pin, gpio_num_t mosi_pin, gpio_num_t miso_pin, 
                     gpio_num_t clk_pin, spi_host_device_t host = SPI3_HOST, 
                     uint32_t max_transfer_sz = 2048);

    // === 初始化和控制方法 ===
    
    /**
     * @brief 初始化SD卡
     * 
     * @return esp_err_t 初始化结果
     */
    esp_err_t init();

    /**
     * @brief 反初始化SD卡
     * 
     * @return esp_err_t 反初始化结果
     */
    esp_err_t deinit();

    /**
     * @brief 挂载文件系统
     * 
     * @return esp_err_t 挂载结果
     */
    esp_err_t mount();

    /**
     * @brief 卸载文件系统
     * 
     * @return esp_err_t 卸载结果
     */
    esp_err_t unmount();

    // === 文件操作方法 ===
    
    /**
     * @brief 列出指定目录的文件
     * 
     * @param path 目录路径 (nullptr表示根目录)
     * @return esp_err_t 操作结果
     */
    esp_err_t list_files(const char* path = nullptr);

    /**
     * @brief 创建目录
     * 
     * @param path 目录路径
     * @return esp_err_t 操作结果
     */
    esp_err_t create_directory(const char* path);

    /**
     * @brief 删除文件
     * 
     * @param filepath 文件路径
     * @return esp_err_t 操作结果
     */
    esp_err_t delete_file(const char* filepath);

    /**
     * @brief 写入文件
     * 
     * @param filepath 文件路径
     * @param data 数据指针
     * @param size 数据大小
     * @return esp_err_t 操作结果
     */
    esp_err_t write_file(const char* filepath, const void* data, size_t size);

    /**
     * @brief 读取文件
     * 
     * @param filepath 文件路径
     * @param buffer 缓冲区指针
     * @param size 缓冲区大小
     * @param bytes_read 实际读取字节数 (输出参数)
     * @return esp_err_t 操作结果
     */
    esp_err_t read_file(const char* filepath, void* buffer, size_t size, size_t* bytes_read);

    // === 测试和性能方法 ===
    
    /**
     * @brief SD卡性能测试
     * 
     * @param test_size_mb 测试大小(MB)
     * @param test_filepath 测试文件路径 (nullptr使用默认路径)
     * @return esp_err_t 测试结果
     */
    esp_err_t performance_test(uint32_t test_size_mb, const char* test_filepath = nullptr);

    /**
     * @brief 写入性能测试
     * 
     * @param test_size_mb 测试大小(MB)
     * @param test_filepath 测试文件路径
     * @param chunk_size 写入块大小
     * @return esp_err_t 测试结果
     */
    esp_err_t write_performance_test(uint32_t test_size_mb, const char* test_filepath, size_t chunk_size = 16 * 1024);

    /**
     * @brief 读取性能测试
     * 
     * @param test_filepath 测试文件路径
     * @param chunk_size 读取块大小
     * @return esp_err_t 测试结果
     */
    esp_err_t read_performance_test(const char* test_filepath, size_t chunk_size = 16 * 1024);

    /**
     * @brief 综合测试
     * 
     * 包含初始化、文件操作、性能测试的完整测试流程
     * @param test_size_mb 性能测试大小(MB)
     * @return esp_err_t 测试结果
     */
    esp_err_t comprehensive_test(uint32_t test_size_mb = 128);

    // === 状态查询方法 ===
    
    /**
     * @brief 检查SD卡是否已初始化
     * 
     * @return bool 初始化状态
     */
    bool is_initialized() const;

    /**
     * @brief 检查文件系统是否已挂载
     * 
     * @return bool 挂载状态
     */
    bool is_mounted() const;

    /**
     * @brief 获取SD卡信息
     * 
     * @return sdmmc_card_t* SD卡信息指针
     */
    const sdmmc_card_t* get_card_info() const;

    /**
     * @brief 打印SD卡信息
     * 
     * @return esp_err_t 操作结果
     */
    esp_err_t print_card_info() const;

    /**
     * @brief 获取SD卡容量信息
     * 
     * @param total_bytes 总容量 (输出参数)
     * @param free_bytes 可用容量 (输出参数)
     * @return esp_err_t 操作结果
     */
    esp_err_t get_capacity_info(uint64_t* total_bytes, uint64_t* free_bytes) const;

    /**
     * @brief 获取工作模式描述字符串
     * 
     * @return const char* 模式描述字符串
     */
    const char* get_mode_description() const;

    /**
     * @brief 获取工作模式描述字符串（静态方法）
     * 
     * @param mode SD卡工作模式
     * @return const char* 模式描述字符串
     */
    static const char* get_mode_description(sd_test_mode_t mode);
};

#endif // __SD_TOOLS_H__

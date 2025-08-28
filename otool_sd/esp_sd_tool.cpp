#include "esp_sd_tool.h"
#include "esp_log.h"
#include "ff.h"
#include <string.h>

#ifdef __cplusplus
#include <algorithm>
#endif

static const char *TAG = "esp_sd_tool";

// === C++ 类实现 ===

#ifdef __cplusplus

sd_tools::sd_tools() 
    : initialized(false), mounted(false), card(nullptr), current_mode(SD_MODE_MMC_1BIT),
      mode(SD_MODE_MMC_1BIT), mount_point("/sdcard"), max_freq_khz(20000),
      max_files(5), allocation_unit_size(16 * 1024), format_if_mount_failed(false),
      mmc_clk_pin(GPIO_NUM_NC), mmc_cmd_pin(GPIO_NUM_NC), mmc_d0_pin(GPIO_NUM_NC),
      mmc_d1_pin(GPIO_NUM_NC), mmc_d2_pin(GPIO_NUM_NC), mmc_d3_pin(GPIO_NUM_NC),
      mmc_internal_pullup(true),
      spi_cs_pin(GPIO_NUM_NC), spi_mosi_pin(GPIO_NUM_NC), spi_miso_pin(GPIO_NUM_NC),
      spi_clk_pin(GPIO_NUM_NC), spi_host(SPI3_HOST), spi_max_transfer_sz(2048)
{
    ESP_LOGD(TAG, "sd_tools constructor called");
}

sd_tools::~sd_tools()
{
    ESP_LOGD(TAG, "sd_tools destructor called");
    if (initialized) {
        deinit();
    }
}

// === 配置方法 ===

void sd_tools::set_mode(sd_test_mode_t mode, gpio_num_t clk_pin, gpio_num_t cmd_pin, gpio_num_t d0_pin, bool internal_pullup)
{
    if (initialized) {
        ESP_LOGW(TAG, "Cannot change mode while initialized");
        return;
    }
    
    if (mode != SD_MODE_MMC_1BIT) {
        ESP_LOGE(TAG, "Invalid mode for this function. Expected SD_MODE_MMC_1BIT");
        return;
    }
    
    // 设置为 MMC 1位模式
    this->mode = mode;
    this->mmc_clk_pin = clk_pin;
    this->mmc_cmd_pin = cmd_pin;
    this->mmc_d0_pin = d0_pin;
    this->mmc_internal_pullup = internal_pullup;
    
    ESP_LOGD(TAG, "Mode set to MMC 1-bit - CLK: %d, CMD: %d, D0: %d, pullup: %s", 
             clk_pin, cmd_pin, d0_pin, internal_pullup ? "enabled" : "disabled");
}

void sd_tools::set_mode(sd_test_mode_t mode, gpio_num_t clk_pin, gpio_num_t cmd_pin, gpio_num_t d0_pin,
                        gpio_num_t d1_pin, gpio_num_t d2_pin, gpio_num_t d3_pin, 
                        bool internal_pullup)
{
    if (initialized) {
        ESP_LOGW(TAG, "Cannot change mode while initialized");
        return;
    }
    
    if (mode != SD_MODE_MMC_4BIT) {
        ESP_LOGE(TAG, "Invalid mode for this function. Expected SD_MODE_MMC_4BIT");
        return;
    }
    
    // 设置为 MMC 4位模式
    this->mode = mode;
    this->mmc_clk_pin = clk_pin;
    this->mmc_cmd_pin = cmd_pin;
    this->mmc_d0_pin = d0_pin;
    this->mmc_d1_pin = d1_pin;
    this->mmc_d2_pin = d2_pin;
    this->mmc_d3_pin = d3_pin;
    this->mmc_internal_pullup = internal_pullup;
    
    ESP_LOGD(TAG, "Mode set to MMC 4-bit - CLK: %d, CMD: %d, D0: %d, D1: %d, D2: %d, D3: %d, pullup: %s", 
             clk_pin, cmd_pin, d0_pin, d1_pin, d2_pin, d3_pin, internal_pullup ? "enabled" : "disabled");
}

void sd_tools::set_mode(sd_test_mode_t mode, gpio_num_t cs_pin, gpio_num_t mosi_pin, gpio_num_t miso_pin, 
                        gpio_num_t clk_pin, spi_host_device_t host, uint32_t max_transfer_sz)
{
    if (initialized) {
        ESP_LOGW(TAG, "Cannot change mode while initialized");
        return;
    }
    
    if (mode != SD_MODE_SPI) {
        ESP_LOGE(TAG, "Invalid mode for this function. Expected SD_MODE_SPI");
        return;
    }
    
    // 设置为 SPI 模式
    this->mode = mode;
    this->spi_cs_pin = cs_pin;
    this->spi_mosi_pin = mosi_pin;
    this->spi_miso_pin = miso_pin;
    this->spi_clk_pin = clk_pin;
    this->spi_host = host;
    this->spi_max_transfer_sz = max_transfer_sz;
    
    ESP_LOGD(TAG, "Mode set to SPI - CS: %d, MOSI: %d, MISO: %d, CLK: %d, host: %d, max_transfer: %lu", 
             cs_pin, mosi_pin, miso_pin, clk_pin, host, max_transfer_sz);
}

sd_test_mode_t sd_tools::get_mode() const
{
    return mode;
}

void sd_tools::set_mount_point(const char* mount_point)
{
    if (mounted) {
        ESP_LOGW(TAG, "Cannot change mount point while mounted");
        return;
    }
    this->mount_point = mount_point ? mount_point : "/sdcard";
    ESP_LOGD(TAG, "Mount point set to: %s", this->mount_point.c_str());
}

const char* sd_tools::get_mount_point() const
{
    return mount_point.c_str();
}

void sd_tools::set_max_frequency(uint32_t freq_khz)
{
    if (initialized) {
        ESP_LOGW(TAG, "Cannot change frequency while initialized");
        return;
    }
    this->max_freq_khz = freq_khz;
    ESP_LOGD(TAG, "Max frequency set to: %lu kHz", freq_khz);
}

uint32_t sd_tools::get_max_frequency() const
{
    return max_freq_khz;
}

void sd_tools::set_filesystem_config(uint16_t max_files, uint32_t allocation_unit_size, bool format_if_mount_failed)
{
    if (mounted) {
        ESP_LOGW(TAG, "Cannot change filesystem config while mounted");
        return;
    }
    this->max_files = max_files;
    this->allocation_unit_size = allocation_unit_size;
    this->format_if_mount_failed = format_if_mount_failed;
    ESP_LOGD(TAG, "Filesystem config set - max_files: %d, allocation_unit: %lu, format_on_fail: %s", 
             max_files, allocation_unit_size, format_if_mount_failed ? "true" : "false");
}

// === 初始化和控制方法 ===

esp_err_t sd_tools::init()
{
    if (initialized) {
        ESP_LOGW(TAG, "SD card already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SD card - Mode: %s", get_mode_description());

    esp_err_t ret;
    if (mode == SD_MODE_MMC_1BIT || mode == SD_MODE_MMC_4BIT) {
        ret = init_mmc_mode();
    } else {
        ret = init_spi_mode();
    }

    if (ret == ESP_OK) {
        initialized = true;
        current_mode = mode;
        ESP_LOGI(TAG, "SD card initialized successfully");
        
        // 自动挂载文件系统
        ret = mount();
    }

    return ret;
}

esp_err_t sd_tools::deinit()
{
    if (!initialized) {
        ESP_LOGW(TAG, "SD card not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing SD card");

    // 先卸载文件系统
    if (mounted) {
        unmount();
    }

    // 清理资源
    if (current_mode == SD_MODE_SPI) {
        cleanup_spi_mode();
    }

    initialized = false;
    card = nullptr;
    ESP_LOGI(TAG, "SD card deinitialized");
    
    return ESP_OK;
}

esp_err_t sd_tools::mount()
{
    if (!initialized) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mounted) {
        ESP_LOGW(TAG, "Filesystem already mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Mounting filesystem to %s", mount_point.c_str());

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = max_files,
        .allocation_unit_size = allocation_unit_size
    };

    esp_err_t ret;
    if (current_mode == SD_MODE_MMC_1BIT || current_mode == SD_MODE_MMC_4BIT) {
        ret = init_mmc_mode();
    } else {
        ret = init_spi_mode();
    }

    if (ret == ESP_OK) {
        mounted = true;
        ESP_LOGI(TAG, "Filesystem mounted successfully");
        
        // 打印SD卡信息
        if (card != nullptr) {
            print_card_info();
        }
    }

    return ret;
}

esp_err_t sd_tools::unmount()
{
    if (!mounted) {
        ESP_LOGW(TAG, "Filesystem not mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Unmounting filesystem");

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point.c_str(), card);
    
    if (current_mode == SD_MODE_SPI) {
        cleanup_spi_mode();
    }

    mounted = false;
    ESP_LOGI(TAG, "Filesystem unmounted");
    
    return ret;
}

// === 内部辅助函数 ===

esp_err_t sd_tools::init_mmc_mode()
{
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    
    // 检查必要的引脚是否已配置
    if (mmc_clk_pin == GPIO_NUM_NC || mmc_cmd_pin == GPIO_NUM_NC || mmc_d0_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "MMC pins not configured. Please call set_mmc_pins() first.");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (mode == SD_MODE_MMC_4BIT && !is_4bit_supported()) {
        ESP_LOGW(TAG, "4-bit mode requested, but hardware only supports 1-bit. Falling back to 1-bit mode.");
    }
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = max_freq_khz;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = (mode == SD_MODE_MMC_4BIT && is_4bit_supported()) ? 4 : 1;
    slot_config.clk = mmc_clk_pin;
    slot_config.cmd = mmc_cmd_pin;
    slot_config.d0 = mmc_d0_pin;
    
    if (slot_config.width == 4) {
        slot_config.d1 = mmc_d1_pin;
        slot_config.d2 = mmc_d2_pin;
        slot_config.d3 = mmc_d3_pin;
    }
    
    slot_config.flags = mmc_internal_pullup ? SDMMC_SLOT_FLAG_INTERNAL_PULLUP : 0;

    ESP_LOGI(TAG, "Mounting filesystem (MMC %d-bit mode)", slot_config.width);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = max_files,
        .allocation_unit_size = allocation_unit_size
    };

    return esp_vfs_fat_sdmmc_mount(mount_point.c_str(), &host, &slot_config, &mount_config, &card);
}

esp_err_t sd_tools::init_spi_mode()
{
    ESP_LOGI(TAG, "Using SPI peripheral");
    
    // 检查必要的引脚是否已配置
    if (spi_cs_pin == GPIO_NUM_NC || spi_mosi_pin == GPIO_NUM_NC || 
        spi_miso_pin == GPIO_NUM_NC || spi_clk_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "SPI pins not configured. Please call set_spi_pins() first.");
        return ESP_ERR_INVALID_ARG;
    }
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = spi_host;
    host.max_freq_khz = max_freq_khz;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = spi_mosi_pin,
        .miso_io_num = spi_miso_pin,
        .sclk_io_num = spi_clk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)spi_max_transfer_sz,
    };
    
    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = spi_cs_pin;
    slot_config.host_id = spi_host;

    ESP_LOGI(TAG, "Mounting filesystem (SPI mode)");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = max_files,
        .allocation_unit_size = allocation_unit_size
    };

    return esp_vfs_fat_sdspi_mount(mount_point.c_str(), &host, &slot_config, &mount_config, &card);
}

esp_err_t sd_tools::cleanup_spi_mode()
{
    return spi_bus_free(spi_host);
}

bool sd_tools::is_4bit_supported() const
{
    // 检查是否定义了所有4位模式需要的引脚
    return (mmc_d1_pin != GPIO_NUM_NC && mmc_d2_pin != GPIO_NUM_NC && mmc_d3_pin != GPIO_NUM_NC);
}

// === 文件操作方法 ===

esp_err_t sd_tools::list_files(const char* path)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    const char* dir_path = path ? path : mount_point.c_str();
    ESP_LOGI(TAG, "Listing files in directory: %s", dir_path);
    
    DIR *dir = opendir(dir_path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory: %s", dir_path);
        return ESP_FAIL;
    }
    
    struct dirent *entry;
    int file_count = 0;
    int dir_count = 0;
    
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            ESP_LOGI(TAG, "[DIR]  %s", entry->d_name);
            dir_count++;
        } else {
            ESP_LOGI(TAG, "[FILE] %s", entry->d_name);
            file_count++;
        }
    }
    closedir(dir);
    
    ESP_LOGI(TAG, "Total: %d files, %d directories", file_count, dir_count);
    return ESP_OK;
}

esp_err_t sd_tools::create_directory(const char* path)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    int result = mkdir(path, 0755);
    if (result == 0) {
        ESP_LOGI(TAG, "Directory created: %s", path);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to create directory: %s", path);
        return ESP_FAIL;
    }
}

esp_err_t sd_tools::delete_file(const char* filepath)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    int result = remove(filepath);
    if (result == 0) {
        ESP_LOGI(TAG, "File deleted: %s", filepath);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", filepath);
        return ESP_FAIL;
    }
}

esp_err_t sd_tools::write_file(const char* filepath, const void* data, size_t size)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    FILE* file = fopen(filepath, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, size, file);
    fclose(file);

    if (written == size) {
        ESP_LOGI(TAG, "File written successfully: %s (%zu bytes)", filepath, size);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to write complete file: %s (%zu/%zu bytes)", filepath, written, size);
        return ESP_FAIL;
    }
}

esp_err_t sd_tools::read_file(const char* filepath, void* buffer, size_t size, size_t* bytes_read)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    FILE* file = fopen(filepath, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", filepath);
        return ESP_FAIL;
    }

    size_t read_count = fread(buffer, 1, size, file);
    fclose(file);

    if (bytes_read) {
        *bytes_read = read_count;
    }

    ESP_LOGI(TAG, "File read: %s (%zu bytes)", filepath, read_count);
    return ESP_OK;
}

// === 测试和性能方法 ===

esp_err_t sd_tools::performance_test(uint32_t test_size_mb, const char* test_filepath)
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    std::string filepath = test_filepath ? test_filepath : 
                          (mount_point + "/" + std::to_string(test_size_mb) + "MB_test.bin");

    ESP_LOGI(TAG, "Starting %lu MB performance test", test_size_mb);
    
    // 写入测试
    esp_err_t ret = write_performance_test(test_size_mb, filepath.c_str());
    if (ret != ESP_OK) {
        return ret;
    }

    // 读取测试
    ret = read_performance_test(filepath.c_str());
    
    // 清理测试文件
    delete_file(filepath.c_str());
    
    return ret;
}

esp_err_t sd_tools::write_performance_test(uint32_t test_size_mb, const char* test_filepath, size_t chunk_size)
{
    ESP_LOGI(TAG, "Starting %lu MB write performance test", test_size_mb);
    
    // 删除可能存在的旧文件
    remove(test_filepath);

    int fd = open(test_filepath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", test_filepath);
        return ESP_FAIL;
    }
    
    uint32_t max_counts = (test_size_mb * 1024 * 1024) / chunk_size;
    esp_err_t result = ESP_OK;

    char *buffer = (char *)malloc(chunk_size);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for performance test");
        close(fd);
        return ESP_ERR_NO_MEM;
    }
    
    memset(buffer, 0x55, chunk_size); // 填充测试数据
    
    uint64_t start_time = esp_timer_get_time();
    bool success = true;

    for (uint32_t i = 0; i < max_counts && success; i++) {
        ssize_t written = write(fd, buffer, chunk_size);
        if (written != chunk_size) {
            ESP_LOGE(TAG, "Write error at chunk %lu", i);
            success = false;
            result = ESP_FAIL;
        } else if ((i + 1) % 1000 == 0) {
            ESP_LOGI(TAG, "Write progress: %lu/%lu chunks", i + 1, max_counts);
        }
    }

    if (success) {
        fsync(fd);
        uint64_t end_time = esp_timer_get_time();
        uint64_t total_time_ms = (end_time - start_time) / 1000;
        
        ESP_LOGI(TAG, "%lu MB written in %llu ms", test_size_mb, total_time_ms);
        double seconds = total_time_ms / 1000.0;
        double speed_mb_s = test_size_mb / seconds;
        ESP_LOGI(TAG, "Write speed: %.2f MB/s", speed_mb_s);
    }
    
    free(buffer);
    close(fd);
    
    return result;
}

esp_err_t sd_tools::read_performance_test(const char* test_filepath, size_t chunk_size)
{
    ESP_LOGI(TAG, "Starting read performance test for: %s", test_filepath);
    
    int fd = open(test_filepath, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", test_filepath);
        return ESP_FAIL;
    }

    // 获取文件大小
    struct stat file_stat;
    if (fstat(fd, &file_stat) != 0) {
        ESP_LOGE(TAG, "Failed to get file stats");
        close(fd);
        return ESP_FAIL;
    }

    size_t file_size = file_stat.st_size;
    uint32_t max_counts = file_size / chunk_size;
    uint32_t file_size_mb = file_size / (1024 * 1024);
    
    char *buffer = (char *)malloc(chunk_size);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for read test");
        close(fd);
        return ESP_ERR_NO_MEM;
    }
    
    uint64_t start_time = esp_timer_get_time();
    bool success = true;
    esp_err_t result = ESP_OK;

    for (uint32_t i = 0; i < max_counts && success; i++) {
        ssize_t read_bytes = read(fd, buffer, chunk_size);
        if (read_bytes != chunk_size) {
            ESP_LOGE(TAG, "Read error at chunk %lu", i);
            success = false;
            result = ESP_FAIL;
        } else if ((i + 1) % 1000 == 0) {
            ESP_LOGI(TAG, "Read progress: %lu/%lu chunks", i + 1, max_counts);
        }
    }

    if (success) {
        uint64_t end_time = esp_timer_get_time();
        uint64_t total_time_ms = (end_time - start_time) / 1000;
        
        ESP_LOGI(TAG, "%u MB read in %llu ms", file_size_mb, total_time_ms);
        double seconds = total_time_ms / 1000.0;
        double speed_mb_s = file_size_mb / seconds;
        ESP_LOGI(TAG, "Read speed: %.2f MB/s", speed_mb_s);
    }
    
    free(buffer);
    close(fd);
    
    return result;
}

esp_err_t sd_tools::comprehensive_test(uint32_t test_size_mb)
{
    ESP_LOGI(TAG, "Starting comprehensive SD card test");
    
    // 初始化（如果还未初始化）
    esp_err_t ret = init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 列出文件
    list_files();
    
    // 创建测试目录
    std::string test_dir = mount_point + "/test";
    create_directory(test_dir.c_str());
    
    // 性能测试
    ret = performance_test(test_size_mb);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Performance test failed, but continuing...");
    }
    
    // 简单文件操作测试
    std::string test_file = test_dir + "/hello.txt";
    const char* test_data = "Hello, SD Card!";
    ret = write_file(test_file.c_str(), test_data, strlen(test_data));
    if (ret == ESP_OK) {
        char read_buffer[64];
        size_t bytes_read;
        ret = read_file(test_file.c_str(), read_buffer, sizeof(read_buffer), &bytes_read);
        if (ret == ESP_OK) {
            read_buffer[bytes_read] = '\0';
            ESP_LOGI(TAG, "File content: %s", read_buffer);
        }
        delete_file(test_file.c_str());
    }
    
    ESP_LOGI(TAG, "Comprehensive SD card test completed");
    return ESP_OK;
}

// === 状态查询方法 ===

bool sd_tools::is_initialized() const
{
    return initialized;
}

bool sd_tools::is_mounted() const
{
    return mounted;
}

const sdmmc_card_t* sd_tools::get_card_info() const
{
    return card;
}

esp_err_t sd_tools::print_card_info() const
{
    if (card == nullptr) {
        ESP_LOGE(TAG, "No card information available");
        return ESP_ERR_INVALID_STATE;
    }

    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

esp_err_t sd_tools::get_capacity_info(uint64_t* total_bytes, uint64_t* free_bytes) const
{
    if (!mounted) {
        ESP_LOGE(TAG, "Filesystem not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    FATFS *fs;
    DWORD free_clusters;
    int res = f_getfree("0:", &free_clusters, &fs);
    
    if (res == FR_OK) {
        uint64_t total_sectors = (fs->n_fatent - 2) * fs->csize;
        uint64_t free_sectors = free_clusters * fs->csize;
        
        if (total_bytes) {
            *total_bytes = total_sectors * fs->ssize;
        }
        if (free_bytes) {
            *free_bytes = free_sectors * fs->ssize;
        }
        
        ESP_LOGI(TAG, "Card capacity - Total: %llu MB, Free: %llu MB", 
                 (*total_bytes) / (1024 * 1024), (*free_bytes) / (1024 * 1024));
        
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to get capacity info: %d", res);
        return ESP_FAIL;
    }
}

const char* sd_tools::get_mode_description() const
{
    return get_mode_description(mode);
}

const char* sd_tools::get_mode_description(sd_test_mode_t mode)
{
    switch (mode) {
        case SD_MODE_MMC_1BIT:
            return "MMC 1-bit";
        case SD_MODE_MMC_4BIT:
            return "MMC 4-bit";
        case SD_MODE_SPI:
            return "SPI";
        default:
            return "Unknown";
    }
}

#endif // __cplusplus

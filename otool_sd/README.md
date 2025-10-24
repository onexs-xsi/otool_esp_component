# ESP SD Tool

ESP32-P4 的 SD 卡工具库，支持多种 SD 卡工作模式。

## 功能特性

- 支持 MMC 1位模式
- 支持 MMC 4位模式（硬件限制会回退到1位）
- 支持 SPI 模式
- SD 卡初始化和反初始化
- 文件系统挂载和卸载
- 文件列表功能
- 性能测试功能
- 综合测试功能

## 支持的模式

```c
typedef enum {
    SD_MODE_MMC_1BIT = 0,  // MMC 1位模式
    SD_MODE_MMC_4BIT = 1,  // MMC 4位模式 (当前硬件仅支持1位，会回退)
    SD_MODE_SPI = 2        // SPI 模式
} sd_test_mode_t;
```

## 引脚配置

### MMC 模式引脚
- CMD: GPIO_7
- CLK: GPIO_10  
- D0:  GPIO_8

### SPI 模式引脚
- CS:   GPIO_50
- MOSI: GPIO_7
- MISO: GPIO_8
- CLK:  GPIO_10

## API 接口

### 初始化和反初始化
```c
// 初始化 SD 卡
esp_err_t esp_sd_tool_init(sd_test_mode_t mode, sdmmc_card_t **card);

// 反初始化 SD 卡
esp_err_t esp_sd_tool_deinit(sd_test_mode_t mode, sdmmc_card_t *card);
```

### 文件操作
```c
// 列出根目录文件
esp_err_t esp_sd_tool_list_files(void);
```

### 性能测试
```c
// 性能测试，参数为测试大小(MB)
esp_err_t esp_sd_tool_performance_test(uint32_t test_size_mb);
```

### 综合测试
```c
// 综合测试，包含初始化、文件列表、性能测试和反初始化
esp_err_t esp_sd_tool_comprehensive_test(sd_test_mode_t mode);
```

## 使用示例

### 基本使用
```c
#include "sd_tool.h"

void app_main(void) {
    // 运行 MMC 1位模式综合测试
    esp_sd_tool_comprehensive_test(SD_MODE_MMC_1BIT);
    
    // 运行 SPI 模式综合测试  
    esp_sd_tool_comprehensive_test(SD_MODE_SPI);
}
```

### 高级使用
```c
#include "sd_tool.h"

void advanced_sd_test(void) {
    sdmmc_card_t *card = NULL;
    
    // 初始化
    if (esp_sd_tool_init(SD_MODE_MMC_1BIT, &card) == ESP_OK) {
        // 列出文件
        esp_sd_tool_list_files();
        
        // 性能测试 32MB
        esp_sd_tool_performance_test(32);
        
        // 反初始化
        esp_sd_tool_deinit(SD_MODE_MMC_1BIT, card);
    }
}
```

## 注意事项

1. **硬件限制**: 当前硬件只支持 MMC 1位模式，选择 4位模式会自动回退到1位模式。

2. **引脚复用**: MMC 模式和 SPI 模式使用相同的物理引脚，但配置不同。

3. **内存管理**: 使用 `esp_sd_tool_init()` 后必须调用 `esp_sd_tool_deinit()` 来释放资源。

4. **错误处理**: 所有函数都返回 `esp_err_t`，请检查返回值以确保操作成功。

## 编译配置

在 `CMakeLists.txt` 中添加以下依赖：

```cmake
REQUIRES driver esp_driver_i2c i2c_bus esp_codec_dev fatfs sdmmc
```

## 故障排除

### 常见错误

1. **挂载失败**: 检查 SD 卡是否正确插入，引脚连接是否正确。

2. **SPI 总线初始化失败**: 确保 SPI 总线没有被其他设备占用。

3. **文件系统错误**: SD 卡可能需要格式化为 FAT32 文件系统。

### 调试信息

启用调试日志：
```c
esp_log_level_set("esp_sd_tool", ESP_LOG_DEBUG);
```

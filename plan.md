# otool_esp_component 开发计划

> 最后更新：2026-03-02

## 项目概述

`otool_esp_component` 是一个多功能 ESP-IDF 组件工具箱，包含多个可选子组件。
通过 `otool_esp_component_toolkit.py` 管理工具实现：
- 子组件按需启用/禁用
- 音频嵌入参数管理
- 多项目配置持久化（`.toolkit_history`）
- 推送/拉取工作流自动化

## 架构

```
otool_esp_component/
├── CMakeLists.txt               # 主构建文件（option 开关 + 音频配置）
├── otool_esp_component_toolkit.py  # 管理工具（本文件重点）
├── otool_toolbox.cpp            # 框架基础（始终编译）
├── .toolkit_history/            # 项目配置历史（gitignore）
│   └── component_config_history.json
├── include/                     # 公共头文件
├── otool_i2c_bus/               # I2C 总线工具（框架基础，始终编译）
├── otool_audio/                 # 子组件：音频 (ES8311/ES7210/SR/DSP)
├── otool_bmi270/                # 子组件：IMU (BMI270 + BMM150)
├── otool_ir/                    # 子组件：红外 (NEC/RC5)
├── otool_power_ic/              # 子组件：电源IC (IP2315)
├── otool_sd/                    # 子组件：SD卡 (MMC/SPI)
└── otool_ic_rx8130/             # 子组件：RTC (RX8130)
```

## 子组件开关

| CMake Option               | 子组件        | 编译定义                   |
|---------------------------|--------------|--------------------------|
| ENABLE_OTOOL_AUDIO        | otool_audio  | OTOOL_ENABLE_AUDIO       |
| ENABLE_OTOOL_BMI270       | otool_bmi270 | OTOOL_ENABLE_BMI270      |
| ENABLE_OTOOL_IR           | otool_ir     | OTOOL_ENABLE_IR          |
| ENABLE_OTOOL_POWER_IC     | otool_power_ic | OTOOL_ENABLE_POWER_IC  |
| ENABLE_OTOOL_SD           | otool_sd     | OTOOL_ENABLE_SD          |
| ENABLE_OTOOL_RTC_RX8130   | otool_ic_rx8130 | OTOOL_ENABLE_RTC_RX8130 |

**默认状态**：拉取后所有子组件 **OFF**（仅框架 + I2C 基础）

## toolkit.py 功能矩阵

### 已完成 ✓

- [x] **CMake 状态读写** - 正则解析 option() 和 AUDIO_FILE_CONFIGS
- [x] **子组件开关编辑** - 交互式 toggle（msvcrt 箭头键 / 数字序号回退）
- [x] **音频参数开关编辑** - 同上
- [x] **全部禁用** - 一键恢复到拉取默认状态
- [x] **`.toolkit_history` 持久化** - JSON 格式，按项目存储
- [x] **项目识别** - 支持 `--project-id` 或自动按路径识别
- [x] **保存/恢复配置** - save / restore 命令
- [x] **Push 工作流** - 保存配置 → 全部禁用 → 交互式 git add/commit/push
- [x] **Pull 工作流** - 交互式 git pull → 恢复历史配置
- [x] **交互式菜单** - 带 ANSI 颜色、分组布局、状态概览
- [x] **历史管理** - 查看列表、查看详情、删除记录
- [x] **Git 状态查看** - 分支名、工作区状态
- [x] **CLI 子命令** - menu/status/save/restore/disable-all/push/pull/init/list-history
- [x] **dry-run 模式** - 预览不写文件
- [x] **确认对话框** - 破坏性操作前确认
- [x] **ANSI 颜色输出** - Windows ANSI 支持
- [x] **.gitignore 已配置** - .toolkit_history/ 已排除
- [x] **Submodule 管理** - 解析 .gitmodules、状态检查、init/update
- [x] **Init 命令** - 首次 git clone 后的初始化（submodule + 恢复历史配置）

### 待完成 / 可考虑

- [ ] **批量项目操作** - 同时恢复多个项目配置（低优先级）
- [ ] **配置导入/导出** - 导出为独立 JSON 文件供分享
- [ ] **子组件依赖检查** - 自动提示某些子组件的前置依赖
- [ ] **构建验证** - 修改配置后自动执行 idf.py build 验证

## 工作流说明

### Push 流程（推送组件到远程仓库）

```
python otool_esp_component_toolkit.py push
```

1. **保存**当前项目的启用配置到 `.toolkit_history/`
2. **禁用**所有子组件和音频参数（确保推送状态干净）
3. **交互式 Git**：提示执行 git add → commit → push

> 目的：推送时确保组件仓库中所有开关为 OFF，拉取方不会意外编译不需要的模块

### Pull 流程（拉取组件更新并恢复配置）

```
python otool_esp_component_toolkit.py pull
```

1. **交互式 Git**：提示执行 git pull
2. **恢复**当前项目在 `.toolkit_history/` 中保存的启用配置

> 目的：拉取后自动还原该项目之前启用的子组件，无需手动重新配置

### Init 流程（首次 git clone 后初始化）

```
python otool_esp_component_toolkit.py init
```

1. **初始化子模块** `git submodule init` + `git submodule update --recursive`
2. **检查组件状态**：确认所有开关处于 OFF（拉取默认）
3. **恢复历史**：如果 `.toolkit_history` 中有该项目记录，提示是否恢复

> 目的：一条命令完成首次 clone 后的所有准备工作（子模块拉取 + 配置恢复）
> 典型场景：新开发者 clone 仓库后，只需 `python toolkit.py init --project-id xxx` 即可

## 开发日志

### 2026-03-02
- 完成 toolkit.py v1.0 全部功能
  - ANSI 颜色输出 + Windows 兼容
  - msvcrt 箭头键交互式 toggle 编辑器
  - 完整的 push/pull 工作流（含交互式 git 操作）
  - 历史管理（查看/删除/详情）
  - Git 状态集成（分支、工作区状态、远程检测）
  - CLI 子命令系统 + 交互式菜单
- 添加 Submodule 管理
  - 解析 .gitmodules 获取子模块定义
  - git submodule status 检查初始化状态
  - 子模块状态展示（commit hash、tag）
- 添加 Init 命令
  - 首次 clone 后一键初始化
  - 自动 submodule init + update --recursive
  - 自动检查 + 提示恢复历史配置
- 创建 plan.md 项目文档

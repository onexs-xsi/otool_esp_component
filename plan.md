# otool_esp_component Toolkit 优化计划

> 最后更新：2026-03-03

---

## 一、现有功能清单

| 模块 | 功能 | 位置 |
|------|------|------|
| CMake读写 | 解析/写入 option() 开关和 AUDIO_FILE_CONFIGS | `CMakeStateManager` |
| 历史持久化 | JSON格式按项目key存储配置 | `ToolkitHistory` |
| 开关编辑器 | Windows箭头键/数字序号交互式toggle | `interactive_toggle` |
| Git工具 | status/branch/remote/submodule状态/init-update | `git_*` 函数群 |
| Push工作流 | 保存→禁用→git push→恢复 | `workflow_push` |
| Pull工作流 | git pull→恢复 | `workflow_pull` |
| Init工作流 | submodule init→恢复 | `workflow_init` |
| 子工具启动 | subprocess调用子脚本 | `launch_subtool` |
| 交互式菜单 | 主菜单 + 历史子菜单 | `run_menu` / `_menu_history_list` |
| CLI子命令 | menu/status/save/restore/disable-all/push/pull/init/list-history | `build_parser` / `main` |
| dry-run | 预览不写文件 | 各工作流 |

---

## 二、问题诊断

### 2.1 重复代码（需消除）

| 问题 | 位置 | 说明 |
|------|------|------|
| 历史项目格式化 | `_menu_history_list` L1464 + `list-history` CLI L1730 | 两处都有 `e_sub/e_aud` 计算和打印逻辑 |
| Git上下文获取 | `_interactive_git_push` L1011 + `_interactive_git_pull` L1056 | 都有 branch/status 获取和打印 |
| 交互式git操作 | `_interactive_git_push` / `_interactive_git_pull` | 结构相同，仅操作名不同 |
| Push逻辑分裂 | `workflow_push` + `main()` L1671 | main中重复了git add/commit/push逻辑 |

### 2.2 菜单问题（需重设计）

- 键位混乱：数字 `1-9` + 字母 `i/g/a` + `0` 退出，无规律
- 分组标题冗余：每组用3行（divider + title + divider），视觉噪音多
- 描述过长：如 `Push 工作流（保存 → 全部禁用 → git push → 恢复）`
- 子工具单独一组显得冗余（只有1个条目）
- 状态行颜色语义不清晰

### 2.3 跨文件重复

- `check_and_update_material.py` 有独立的颜色/UI系统，未复用 `C` 类和 `wait_key`/`confirm` 等工具

---

## 三、优化方案

### 3.1 消除内部重复

- [x] **A** 提取 `format_project_entry(meta) -> str` — 供菜单和CLI共用
- [x] **B** 提取 `_print_git_context(root, show_status)` — 供 push/pull 交互共用
- [x] **C** 合并 `_interactive_git_op(root, op: "push"|"pull")` — 统一交互式git逻辑
- [x] **D** 整合 `main()` 中的push git逻辑到 `workflow_push`（新增 `commit_msg` 参数）

### 3.2 菜单重设计

新设计原则：
- 全字母键，无数字（更专业，更易记）
- 小写 = 常用操作，大写 = 历史/工作流
- 分组标题单行（`── 分组名 ──────`），去掉上下divider
- 状态信息内联到header行
- 子工具并入工具分组

```
  ╔══════════════════════════════════════════════════════╗
  ║   otool_esp_component Toolkit  v2.0                 ║
  ╚══════════════════════════════════════════════════════╝

  项目: corep4  │  分支: main  │  子组件: 3/6  │  音频: 2/5

  ── 配置 ──────────────────────────────────────────────
  s   状态总览
  e   编辑子组件开关
  a   编辑音频参数
  x   全部禁用

  ── 历史 ──────────────────────────────────────────────
  S   保存当前配置
  R   恢复配置
  H   历史列表

  ── 工作流 ────────────────────────────────────────────
  p   Push  （保存 → 禁用 → git push → 恢复）
  l   Pull  （git pull → 恢复）
  I   Init  （子模块初始化 → 恢复）

  ── 工具 ──────────────────────────────────────────────
  m   音频素材管理
  g   Git 状态 / 子模块

  q   退出
  ──────────────────────────────────────────────────────
  >
```

- [x] **E** 实现新菜单布局

### 3.3 跨文件复用（低优先级）

- [x] **F** `check_and_update_material.py` 尝试 `from otool_esp_component_toolkit import C, wait_key, confirm`，失败时 fallback 到自身定义

---

## 四、实施顺序

1. **A+B+C+D** — 内部重复消除（不影响功能，纯重构）
2. **E** — 菜单重设计（影响UX）
3. **F** — 跨文件复用（可选，独立改动）

---

## 五、开发日志

### 2026-03-02
- 完成 toolkit.py v1.0 全部功能
- 添加 Submodule 管理 + Init 命令
- 创建 plan.md

### 2026-03-03（续）
- [x] G: 提取 `format_component_tags()` + `format_audio_tags()` 可复用显示块
  - 主菜单 header 显示 `[AUDIO] [IR] [SD]` 绿色标签 + 启用音频文件名
  - 历史列表每项复用同一显示块，展示子组件标签和音频详情
  - 菜单输入统一 `.lower()` 支持大小写模糊（s/S 均有效）
  - 键位调整：S→w（保存），R→r（恢复），H→h（历史），I→i（Init）
  - 新增 `SUBCOMPONENT_SHORT` 短标签字典

### 2026-03-03（续 2）
- [x] H: 子组件脱耦架构 — 每个子组件独立 CMakeLists.txt

  **设计方案：CMake `include()` 编排模式**

  | 层级 | 文件 | 职责 |
  |------|------|------|
  | 编排层 | `CMakeLists.txt` | option 声明 + AUDIO_FILE_CONFIGS + include 子组件 + idf_component_register |
  | 子组件层 | `otool_xxx/CMakeLists.txt` | 自包含：用 `CMAKE_CURRENT_LIST_DIR` 追加 SRCS/INCLUDES/REQUIRES/COMPILE_DEFS |

  **新增文件：**
  - `otool_i2c_bus/CMakeLists.txt` — I2C 总线（框架基础，始终启用）
  - `otool_audio/CMakeLists.txt` — 音频（含 AUDIO_FILE_CONFIGS 处理，条件启用）
  - `otool_bmi270/CMakeLists.txt` — IMU（条件启用）
  - `otool_ir/CMakeLists.txt` — 红外（条件启用）
  - `otool_power_ic/CMakeLists.txt` — 电源IC（条件启用）
  - `otool_sd/CMakeLists.txt` — SD卡（条件启用）
  - `otool_ic_rx8130/CMakeLists.txt` — RTC（条件启用）

  **根 CMakeLists.txt 变化：** 192 行 → 103 行（-46%），消除重复，结构清晰

  **添加新子组件步骤（只需 3 步）：**
  1. 创建 `otool_new/CMakeLists.txt`（参照 otool_sd 为模板）
  2. 在根 cmake 加 `option(ENABLE_OTOOL_NEW ...)`
  3. 在根 cmake 加 `include(otool_new/CMakeLists.txt)`

  **约束说明：**
  - `option()` 和 `AUDIO_FILE_CONFIGS` 保留在根 cmake → toolkit.py 无需修改
  - 子组件用 `return()` 提前退出（未启用时），不使用大括号嵌套
  - 路径使用 `CMAKE_CURRENT_LIST_DIR`（绝对路径），对 include() 调用安全

### 待办
<!-- 暂无 -->

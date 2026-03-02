#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
otool_esp_component_toolkit.py

多组件管理工具：
1. 管理子组件开关（默认全部关闭，仅保留框架）
2. 管理音频参数默认开关（AUDIO_FILE_CONFIGS）
3. 使用 .toolkit_history 记录"某个项目"的启用配置
4. 提供 push/pull 流程：
   - push: 保存当前项目配置 -> 全部禁用 -> 可选 git push
   - pull: 可选执行 git pull -> 从历史恢复当前项目配置
5. 提供交互式终端菜单（参考 check_and_update_material.py 风格）

使用方式：
  python otool_esp_component_toolkit.py              # 交互式菜单
  python otool_esp_component_toolkit.py status        # 查看状态
  python otool_esp_component_toolkit.py push          # push 工作流
  python otool_esp_component_toolkit.py pull          # pull 工作流
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Windows 键盘输入（用于箭头键交互式菜单）
_HAS_MSVCRT = False
if os.name == "nt":
    try:
        import msvcrt
        _HAS_MSVCRT = True
    except ImportError:
        pass


# ============================================================================
# ANSI 颜色工具
# ============================================================================
class C:
    """ANSI 颜色常量，用于终端输出美化"""
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN    = "\033[36m"
    WHITE   = "\033[37m"

    @staticmethod
    def ok(msg: str) -> str:
        return f"{C.GREEN}✓{C.RESET} {msg}"

    @staticmethod
    def err(msg: str) -> str:
        return f"{C.RED}✗{C.RESET} {msg}"

    @staticmethod
    def warn(msg: str) -> str:
        return f"{C.YELLOW}⚠{C.RESET} {msg}"

    @staticmethod
    def info(msg: str) -> str:
        return f"{C.CYAN}ℹ{C.RESET} {msg}"

    @staticmethod
    def header(msg: str) -> str:
        return f"{C.BOLD}{C.CYAN}{msg}{C.RESET}"

    @staticmethod
    def step(num: int, total: int, msg: str) -> str:
        return f"{C.BOLD}步骤 {num}/{total}:{C.RESET} {msg}"

    @staticmethod
    def on_off(enabled: bool, label: str) -> str:
        if enabled:
            return f"  {C.GREEN}{C.BOLD}[ON ]{C.RESET} {label}"
        return f"  {C.DIM}[OFF]{C.RESET} {label}"

    @staticmethod
    def enable_win_ansi() -> None:
        """启用 Windows 终端 ANSI 支持"""
        if os.name == "nt":
            os.system("")  # 触发 Windows 的 ANSI 支持


# ============================================================================
# 常量定义
# ============================================================================
SUBCOMPONENT_OPTIONS: List[str] = [
    "ENABLE_OTOOL_AUDIO",
    "ENABLE_OTOOL_BMI270",
    "ENABLE_OTOOL_IR",
    "ENABLE_OTOOL_POWER_IC",
    "ENABLE_OTOOL_SD",
    "ENABLE_OTOOL_RTC_RX8130",
]

# 子组件中文描述
SUBCOMPONENT_LABELS: Dict[str, str] = {
    "ENABLE_OTOOL_AUDIO":      "音频 (ES8311/ES7210/SR/DSP)",
    "ENABLE_OTOOL_BMI270":     "IMU (BMI270 + BMM150)",
    "ENABLE_OTOOL_IR":         "红外 (NEC/RC5 编解码)",
    "ENABLE_OTOOL_POWER_IC":   "电源IC (IP2315)",
    "ENABLE_OTOOL_SD":         "SD卡 (MMC/SPI)",
    "ENABLE_OTOOL_RTC_RX8130": "RTC (RX8130)",
}

HISTORY_DIR_NAME = ".toolkit_history"
HISTORY_FILE_NAME = "component_config_history.json"
HISTORY_VERSION = 1

BANNER = f"""{C.CYAN}{C.BOLD}\
  ╔══════════════════════════════════════════════════════════════╗
  ║        otool_esp_component Toolkit  v1.0                    ║
  ║        多组件配置管理 · 推送拉取 · 历史记录                 ║
  ╚══════════════════════════════════════════════════════════════╝{C.RESET}
"""

LINE_W = 64


# ============================================================================
# 数据结构
# ============================================================================
@dataclass
class ComponentState:
    subcomponents: Dict[str, bool]
    audio_defaults: Dict[str, bool]

    def clone(self) -> "ComponentState":
        return ComponentState(
            subcomponents=dict(self.subcomponents),
            audio_defaults=dict(self.audio_defaults),
        )

    def all_disabled(self) -> "ComponentState":
        return ComponentState(
            subcomponents={k: False for k in self.subcomponents.keys()},
            audio_defaults={k: False for k in self.audio_defaults.keys()},
        )

    def enabled_sub_count(self) -> int:
        return sum(1 for v in self.subcomponents.values() if v)

    def enabled_audio_count(self) -> int:
        return sum(1 for v in self.audio_defaults.values() if v)

    def is_all_disabled(self) -> bool:
        return self.enabled_sub_count() == 0 and self.enabled_audio_count() == 0


@dataclass
class ProjectContext:
    key: str
    name: str
    path: str


# ============================================================================
# CMakeLists.txt 状态管理
# ============================================================================
class CMakeStateManager:
    OPTION_PATTERN = re.compile(
        r'^\s*option\(\s*(ENABLE_OTOOL_[A-Z0-9_]+)\s+"[^"]*"\s+(ON|OFF)\s*\)\s*$',
        re.MULTILINE,
    )
    AUDIO_BLOCK_PATTERN = re.compile(
        r"set\(\s*AUDIO_FILE_CONFIGS(?P<body>.*?)\n\)",
        re.DOTALL,
    )

    def __init__(self, cmake_file: Path) -> None:
        self.cmake_file = cmake_file

    def load_state(self) -> ComponentState:
        text = self._read_text()

        subcomponents = {name: False for name in SUBCOMPONENT_OPTIONS}
        for match in self.OPTION_PATTERN.finditer(text):
            option_name = match.group(1).strip()
            option_value = match.group(2).strip().upper() == "ON"
            if option_name in subcomponents:
                subcomponents[option_name] = option_value

        audio_defaults = self._parse_audio_defaults(text)

        return ComponentState(
            subcomponents=subcomponents,
            audio_defaults=audio_defaults,
        )

    def apply_state(self, state: ComponentState, dry_run: bool = False) -> None:
        text = self._read_text()

        for option_name, enabled in state.subcomponents.items():
            text = self._replace_subcomponent_option(text, option_name, enabled)

        text = self._replace_audio_defaults(text, state.audio_defaults)

        if dry_run:
            return

        self._write_text(text)

    def _parse_audio_defaults(self, text: str) -> Dict[str, bool]:
        match = self.AUDIO_BLOCK_PATTERN.search(text)
        if not match:
            return {}

        body = match.group("body")
        audio_defaults: Dict[str, bool] = {}

        for raw_line in body.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            quoted = re.match(r'^"([^"]+)"', line)
            if not quoted:
                continue

            payload = quoted.group(1)
            parts = payload.split(":")
            if len(parts) < 3:
                continue

            audio_id = parts[0].strip()
            default_val = parts[2].strip().upper()
            if default_val in ("ON", "OFF"):
                audio_defaults[audio_id] = default_val == "ON"

        return audio_defaults

    def _replace_subcomponent_option(self, text: str, option_name: str, enabled: bool) -> str:
        target = "ON" if enabled else "OFF"
        pattern = re.compile(
            rf'(^\s*option\(\s*{re.escape(option_name)}\s+"[^"]*"\s+)(ON|OFF)(\s*\)\s*$)',
            re.MULTILINE,
        )

        def repl(match: re.Match[str]) -> str:
            return f"{match.group(1)}{target}{match.group(3)}"

        new_text, _ = pattern.subn(repl, text, count=1)
        return new_text

    def _replace_audio_defaults(self, text: str, audio_defaults: Dict[str, bool]) -> str:
        if not audio_defaults:
            return text

        match = self.AUDIO_BLOCK_PATTERN.search(text)
        if not match:
            return text

        body = match.group("body")
        rewritten_lines: List[str] = []

        for line in body.splitlines(keepends=True):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                rewritten_lines.append(line)
                continue

            m = re.match(r'^(\s*)"([^"]+)"(.*)$', line)
            if not m:
                rewritten_lines.append(line)
                continue

            indent, payload, suffix = m.group(1), m.group(2), m.group(3)
            parts = payload.split(":")
            if len(parts) >= 3:
                audio_id = parts[0].strip()
                if audio_id in audio_defaults:
                    parts[2] = "ON" if audio_defaults[audio_id] else "OFF"
                    payload = ":".join(parts)

            rewritten_lines.append(f'{indent}"{payload}"{suffix}')

        new_body = "".join(rewritten_lines)
        start = match.start("body")
        end = match.end("body")
        return text[:start] + new_body + text[end:]

    def _read_text(self) -> str:
        if not self.cmake_file.exists():
            raise FileNotFoundError(f"CMakeLists.txt 不存在: {self.cmake_file}")
        return self.cmake_file.read_text(encoding="utf-8")

    def _write_text(self, content: str) -> None:
        self.cmake_file.write_text(content, encoding="utf-8")


# ============================================================================
# .toolkit_history 历史管理
# ============================================================================
class ToolkitHistory:
    def __init__(self, history_file: Path) -> None:
        self.history_file = history_file

    def save_project_state(self, project: ProjectContext, state: ComponentState) -> None:
        doc = self._load_doc()
        projects = doc.setdefault("projects", {})

        projects[project.key] = {
            "project_name": project.name,
            "project_path": project.path,
            "saved_at": now_str(),
            "subcomponents": dict(state.subcomponents),
            "audio_defaults": dict(state.audio_defaults),
        }

        doc["version"] = HISTORY_VERSION
        doc["updated_at"] = now_str()

        self._save_doc(doc)

    def load_project_state(self, project: ProjectContext) -> Optional[ComponentState]:
        doc = self._load_doc()
        projects = doc.get("projects", {})
        entry = projects.get(project.key)
        if not entry:
            return None

        sub_entry = entry.get("subcomponents", {})
        subcomponents = {name: bool(sub_entry.get(name, False)) for name in SUBCOMPONENT_OPTIONS}

        # 兼容扩展字段
        for key, value in sub_entry.items():
            if key not in subcomponents:
                subcomponents[key] = bool(value)

        audio_defaults = {
            k: bool(v)
            for k, v in entry.get("audio_defaults", {}).items()
        }

        return ComponentState(
            subcomponents=subcomponents,
            audio_defaults=audio_defaults,
        )

    def list_projects(self) -> List[Tuple[str, dict]]:
        doc = self._load_doc()
        projects = doc.get("projects", {})
        return sorted(projects.items(), key=lambda item: item[1].get("saved_at", ""), reverse=True)

    def delete_project(self, project_key: str) -> bool:
        """删除一条历史记录"""
        doc = self._load_doc()
        projects = doc.get("projects", {})
        if project_key in projects:
            del projects[project_key]
            doc["updated_at"] = now_str()
            self._save_doc(doc)
            return True
        return False

    def _default_doc(self) -> dict:
        return {
            "version": HISTORY_VERSION,
            "updated_at": now_str(),
            "projects": {},
        }

    def _load_doc(self) -> dict:
        if not self.history_file.exists():
            return self._default_doc()

        try:
            return json.loads(self.history_file.read_text(encoding="utf-8"))
        except Exception:
            return self._default_doc()

    def _save_doc(self, doc: dict) -> None:
        self.history_file.parent.mkdir(parents=True, exist_ok=True)
        self.history_file.write_text(
            json.dumps(doc, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


# ============================================================================
# 辅助函数
# ============================================================================
def now_str() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def resolve_project_context(project_id: Optional[str], project_path: Optional[str]) -> ProjectContext:
    if project_id:
        path_obj = Path(project_path).resolve() if project_path else Path.cwd().resolve()
        return ProjectContext(
            key=f"id:{project_id}",
            name=project_id,
            path=str(path_obj),
        )

    path_obj = Path(project_path).resolve() if project_path else Path.cwd().resolve()
    normalized = str(path_obj).replace("\\", "/").lower()
    return ProjectContext(
        key=f"path:{normalized}",
        name=path_obj.name,
        path=str(path_obj),
    )


def clear_terminal() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def wait_key(msg: str = "按任意键继续...") -> None:
    """等待用户按键"""
    print(f"\n{C.DIM}{msg}{C.RESET}", end="", flush=True)
    if _HAS_MSVCRT:
        msvcrt.getch()
        print()
    else:
        input()


def confirm(msg: str, default_yes: bool = False) -> bool:
    """确认操作"""
    hint = "[Y/n]" if default_yes else "[y/N]"
    try:
        raw = input(f"{msg} {hint}: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    if not raw:
        return default_yes
    return raw in ("y", "yes")


def print_divider(char: str = "─", width: int = LINE_W) -> None:
    print(f"{C.DIM}{char * width}{C.RESET}")


def print_section(title: str) -> None:
    print()
    print(f"  {C.BOLD}{C.CYAN}── {title} ──{C.RESET}")
    print()


# ============================================================================
# Git 工具函数
# ============================================================================
def git_run(component_root: Path, *args: str, capture: bool = False) -> subprocess.CompletedProcess:
    """执行 git 命令"""
    cmd = ["git"] + list(args)
    print(f"  {C.DIM}$ {' '.join(cmd)}{C.RESET}")
    if capture:
        return subprocess.run(cmd, cwd=str(component_root), capture_output=True, text=True, encoding="utf-8")
    return subprocess.run(cmd, cwd=str(component_root))


def git_status_summary(component_root: Path) -> Optional[str]:
    """获取 git 状态摘要"""
    try:
        result = subprocess.run(
            ["git", "status", "--short"],
            cwd=str(component_root),
            capture_output=True, text=True, encoding="utf-8", timeout=10,
        )
        if result.returncode != 0:
            return None
        return result.stdout.strip() if result.stdout.strip() else "(工作区干净)"
    except Exception:
        return None


def git_current_branch(component_root: Path) -> Optional[str]:
    """获取当前分支名"""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=str(component_root),
            capture_output=True, text=True, encoding="utf-8", timeout=10,
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None


def git_has_remote(component_root: Path) -> bool:
    """检查是否有远程仓库"""
    try:
        result = subprocess.run(
            ["git", "remote"],
            cwd=str(component_root),
            capture_output=True, text=True, encoding="utf-8", timeout=10,
        )
        return bool(result.stdout.strip())
    except Exception:
        return False


# ============================================================================
# Git Submodule 管理
# ============================================================================
@dataclass
class SubmoduleInfo:
    """Git submodule 信息"""
    name: str
    path: str
    url: str
    initialized: bool
    commit: Optional[str] = None
    tag: Optional[str] = None


def git_parse_gitmodules(component_root: Path) -> List[SubmoduleInfo]:
    """解析 .gitmodules 文件，获取所有子模块定义"""
    gitmodules_file = component_root / ".gitmodules"
    if not gitmodules_file.exists():
        return []

    text = gitmodules_file.read_text(encoding="utf-8")
    submodules: List[SubmoduleInfo] = []

    # 解析 [submodule "xxx"] 块
    pattern = re.compile(
        r'\[submodule\s+"([^"]+)"\]\s*'
        r'path\s*=\s*(.+?)\s*'
        r'url\s*=\s*(.+?)\s*(?=\[|$)',
        re.DOTALL,
    )
    for match in pattern.finditer(text):
        name = match.group(1).strip()
        path = match.group(2).strip()
        url = match.group(3).strip()
        submodules.append(SubmoduleInfo(
            name=name, path=path, url=url, initialized=False,
        ))

    return submodules


def git_submodule_status(component_root: Path) -> List[SubmoduleInfo]:
    """
    获取子模块状态（结合 .gitmodules 定义和 git submodule status）。
    返回带有 initialized/commit/tag 信息的列表。
    """
    defined = git_parse_gitmodules(component_root)
    if not defined:
        return []

    # 检查每个子模块目录是否已初始化
    try:
        result = subprocess.run(
            ["git", "submodule", "status"],
            cwd=str(component_root),
            capture_output=True, text=True, encoding="utf-8", timeout=30,
        )
        status_lines = result.stdout.strip().splitlines() if result.returncode == 0 else []
    except Exception:
        status_lines = []

    # 解析 status 输出: " <commit> <path> (<tag>)" 或 "-<commit> <path>"
    status_map: Dict[str, Tuple[bool, str, str]] = {}
    for line in status_lines:
        line = line.strip()
        if not line:
            continue
        # 未初始化: 以 '-' 开头
        uninit = line.startswith("-")
        parts = line.lstrip("- +").split()
        if len(parts) >= 2:
            commit = parts[0]
            path = parts[1]
            tag = parts[2].strip("()") if len(parts) >= 3 else ""
            status_map[path] = (not uninit, commit, tag)

    for sm in defined:
        if sm.path in status_map:
            init, commit, tag = status_map[sm.path]
            sm.initialized = init
            sm.commit = commit
            sm.tag = tag
        else:
            # 目录存在且有内容也视为已初始化
            sm_dir = component_root / sm.path
            sm.initialized = sm_dir.is_dir() and any(sm_dir.iterdir()) if sm_dir.exists() else False

    return defined


def git_submodule_init_update(component_root: Path, recursive: bool = True) -> int:
    """执行 git submodule init + update"""
    print(C.info("初始化子模块..."))
    result = git_run(component_root, "submodule", "init")
    if result.returncode != 0:
        print(C.err("git submodule init 失败"))
        return result.returncode

    print(C.info("更新子模块内容..."))
    cmd_args = ["submodule", "update"]
    if recursive:
        cmd_args.append("--recursive")
    result = git_run(component_root, *cmd_args)
    if result.returncode != 0:
        print(C.err("git submodule update 失败"))
        return result.returncode

    print(C.ok("子模块初始化/更新完成"))
    return 0


def print_submodule_status(component_root: Path) -> None:
    """打印子模块状态"""
    submodules = git_submodule_status(component_root)

    if not submodules:
        print(C.info("未定义任何子模块（.gitmodules 不存在或为空）"))
        return

    print(f"  {C.BOLD}Git 子模块 ({len(submodules)} 个):{C.RESET}")
    for sm in submodules:
        if sm.initialized:
            tag_info = f"  {C.DIM}({sm.tag}){C.RESET}" if sm.tag else ""
            commit_info = f"  {C.DIM}[{sm.commit[:8]}]{C.RESET}" if sm.commit else ""
            print(f"  {C.GREEN}✓{C.RESET} {sm.path}{commit_info}{tag_info}")
        else:
            print(f"  {C.RED}✗{C.RESET} {sm.path}  {C.DIM}(未初始化){C.RESET}")
        print(f"    {C.DIM}url: {sm.url}{C.RESET}")


# ============================================================================
# 状态显示
# ============================================================================
def print_state_summary(state: ComponentState, compact: bool = False) -> None:
    """打印当前配置状态"""
    e_sub = state.enabled_sub_count()
    e_aud = state.enabled_audio_count()

    if not compact:
        print_divider("═")
        print(f"  {C.BOLD}当前配置状态{C.RESET}")
        print_divider("═")
    print(f"  子组件: {C.BOLD}{e_sub}/{len(state.subcomponents)}{C.RESET} 已启用")
    print(f"  音频:   {C.BOLD}{e_aud}/{len(state.audio_defaults)}{C.RESET} 已启用")
    print()

    print(f"  {C.BOLD}子组件:{C.RESET}")
    for name in SUBCOMPONENT_OPTIONS:
        enabled = state.subcomponents.get(name, False)
        label = SUBCOMPONENT_LABELS.get(name, name)
        display = f"{name}  {C.DIM}({label}){C.RESET}"
        print(C.on_off(enabled, display))
    print()

    print(f"  {C.BOLD}音频嵌入参数:{C.RESET}")
    if not state.audio_defaults:
        print(f"    {C.DIM}(未发现 AUDIO_FILE_CONFIGS 项){C.RESET}")
    else:
        for name in sorted(state.audio_defaults.keys()):
            print(C.on_off(state.audio_defaults[name], f"ENABLE_{name}"))

    if not compact:
        print()
        print_divider("═")


# ============================================================================
# 交互式开关编辑器（msvcrt 箭头键 + 空格切换）
# ============================================================================
def interactive_toggle(
    title: str,
    options: Dict[str, bool],
    ordered_keys: Optional[List[str]] = None,
    labels: Optional[Dict[str, str]] = None,
) -> Optional[Dict[str, bool]]:
    """
    交互式开关编辑器。
    - Windows: 使用箭头键移动、空格切换、回车保存、ESC 取消
    - 其他: 使用数字序号切换
    """
    work = dict(options)

    if ordered_keys:
        keys = [k for k in ordered_keys if k in work] + [k for k in work.keys() if k not in ordered_keys]
    else:
        keys = sorted(work.keys())

    if not keys:
        print(C.warn(f"{title} 没有可编辑项。"))
        wait_key()
        return None

    # Windows: 箭头键交互
    if _HAS_MSVCRT:
        return _toggle_msvcrt(title, work, keys, labels)

    # 非 Windows: 数字序号模式
    return _toggle_input(title, work, keys, labels)


def _get_toggle_label(key: str, labels: Optional[Dict[str, str]]) -> str:
    if labels and key in labels:
        return f"{key}  {C.DIM}({labels[key]}){C.RESET}"
    return key


def _toggle_msvcrt(
    title: str,
    work: Dict[str, bool],
    keys: List[str],
    labels: Optional[Dict[str, str]],
) -> Optional[Dict[str, bool]]:
    """Windows msvcrt 箭头键交互式开关编辑"""
    cursor = 0

    while True:
        clear_terminal()
        print_divider("═")
        print(f"  {C.BOLD}{title}{C.RESET}")
        print_divider("─")
        print(f"  {C.DIM}↑↓ 移动  |  空格 切换  |  Enter 保存  |  ESC 取消{C.RESET}")
        print(f"  {C.DIM}A 全部启用  |  D 全部禁用{C.RESET}")
        print_divider("─")

        for idx, key in enumerate(keys):
            prefix = f"{C.CYAN}▸{C.RESET}" if idx == cursor else " "
            label = _get_toggle_label(key, labels)
            if work[key]:
                tag = f"{C.GREEN}{C.BOLD}[ON ]{C.RESET}"
            else:
                tag = f"{C.DIM}[OFF]{C.RESET}"
            print(f"  {prefix} {tag} {label}")

        print_divider("─")
        enabled = sum(1 for v in work.values() if v)
        print(f"  已启用: {C.BOLD}{enabled}/{len(keys)}{C.RESET}")
        print_divider("═")

        # 读取按键
        ch = msvcrt.getch()

        if ch == b"\xe0" or ch == b"\x00":
            # 特殊键前缀
            arrow = msvcrt.getch()
            if arrow == b"H":  # 上
                cursor = (cursor - 1) % len(keys)
            elif arrow == b"P":  # 下
                cursor = (cursor + 1) % len(keys)
        elif ch == b" ":
            # 空格切换
            key = keys[cursor]
            work[key] = not work[key]
        elif ch == b"\r":
            # 回车保存
            return work
        elif ch == b"\x1b":
            # ESC 取消
            return None
        elif ch in (b"a", b"A"):
            # 全部启用
            for k in keys:
                work[k] = True
        elif ch in (b"d", b"D"):
            # 全部禁用
            for k in keys:
                work[k] = False

    return work


def _toggle_input(
    title: str,
    work: Dict[str, bool],
    keys: List[str],
    labels: Optional[Dict[str, str]],
) -> Optional[Dict[str, bool]]:
    """非 Windows 回退: 数字序号模式"""
    while True:
        clear_terminal()
        print_divider("═")
        print(f"  {C.BOLD}{title}{C.RESET}  （输入序号切换，s 保存，q 取消，a 全开，d 全关）")
        print_divider("─")

        for idx, key in enumerate(keys, start=1):
            label = _get_toggle_label(key, labels)
            if work[key]:
                tag = f"{C.GREEN}{C.BOLD}[ON ]{C.RESET}"
            else:
                tag = f"{C.DIM}[OFF]{C.RESET}"
            print(f"  {idx:2d}. {tag} {label}")

        print_divider("─")
        raw = input("  输入: ").strip().lower()

        if raw in ("s", "save", ""):
            return work
        if raw in ("q", "quit", "cancel"):
            return None
        if raw in ("a", "all"):
            for k in keys:
                work[k] = True
            continue
        if raw in ("d", "disable"):
            for k in keys:
                work[k] = False
            continue
        if raw.isdigit():
            num = int(raw)
            if 1 <= num <= len(keys):
                key = keys[num - 1]
                work[key] = not work[key]


# ============================================================================
# 核心操作
# ============================================================================
def save_current_project_state(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
) -> None:
    state = cmake.load_state()
    history.save_project_state(project, state)
    print(C.ok(f"已保存当前项目配置到历史: {C.BOLD}{project.name}{C.RESET}"))


def restore_project_state(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
    dry_run: bool = False,
) -> bool:
    saved = history.load_project_state(project)
    if not saved:
        print(C.warn(f"未找到项目历史配置: {project.name}"))
        return False

    # 仅恢复当前 CMake 中存在的音频项
    current = cmake.load_state()
    filtered_audio = {
        k: saved.audio_defaults.get(k, False)
        for k in current.audio_defaults.keys()
    }

    merged = ComponentState(
        subcomponents={
            k: saved.subcomponents.get(k, False)
            for k in current.subcomponents.keys()
        },
        audio_defaults=filtered_audio,
    )

    cmake.apply_state(merged, dry_run=dry_run)
    if dry_run:
        print(C.ok(f"[dry-run] 已模拟恢复项目配置: {project.name}"))
    else:
        print(C.ok(f"已恢复项目配置: {C.BOLD}{project.name}{C.RESET}"))
    return True


def disable_all(cmake: CMakeStateManager, dry_run: bool = False) -> None:
    current = cmake.load_state()
    cmake.apply_state(current.all_disabled(), dry_run=dry_run)
    if dry_run:
        print(C.ok("[dry-run] 已模拟设置全部开关为 OFF"))
    else:
        print(C.ok("已设置全部开关为 OFF"))


# ============================================================================
# Push / Pull 工作流
# ============================================================================
def workflow_push(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
    dry_run: bool,
    run_git: bool,
    git_args: str,
    component_root: Path,
    interactive: bool = False,
) -> int:
    """
    Push 工作流：
    1. 保存当前项目配置到 .toolkit_history
    2. 将所有组件与参数设置为 OFF
    3. (可选) 执行 git add + commit + push
    """
    total_steps = 4 if run_git or interactive else 3

    print()
    print_divider("═")
    print(f"  {C.BOLD}{C.CYAN}Push 工作流{C.RESET}")
    print_divider("═")

    # 步骤 1: 保存配置
    print(C.step(1, total_steps, "保存当前项目配置到历史"))
    if not dry_run:
        save_current_project_state(project, cmake, history)
    else:
        print(C.ok("[dry-run] 跳过历史写入"))

    # 步骤 2: 全部禁用
    print()
    print(C.step(2, total_steps, "将所有组件与参数设置为 OFF"))
    disable_all(cmake, dry_run=dry_run)

    # 步骤 3: Git 操作
    if interactive and not dry_run:
        print()
        print(C.step(3, total_steps, "Git 操作"))
        print()
        _interactive_git_push(component_root)
    elif run_git:
        print()
        print(C.step(3, total_steps, "执行 git push"))
        if dry_run:
            print(C.ok("[dry-run] 跳过 git 执行"))
        else:
            args_list = shlex.split(git_args, posix=False) if git_args.strip() else []
            result = git_run(component_root, "push", *args_list)
            if result.returncode != 0:
                print(C.err(f"git push 失败，退出码: {result.returncode}"))
                return result.returncode

    # 步骤 N: 恢复配置（push 后自动恢复到 push 前的状态）
    step_restore = total_steps
    print()
    print(C.step(step_restore, total_steps, "恢复项目配置（push 后还原到工作状态）"))
    if dry_run:
        print(C.ok("[dry-run] 跳过配置恢复"))
    else:
        restored = restore_project_state(project, cmake, history, dry_run=False)
        if not restored:
            print(C.warn("未能恢复配置（无历史记录），当前保持全 OFF 状态"))

    print()
    print(C.ok(f"{C.BOLD}Push 工作流完成{C.RESET}"))
    print_divider("═")
    return 0


def workflow_pull(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
    dry_run: bool,
    run_git: bool,
    git_args: str,
    component_root: Path,
    interactive: bool = False,
) -> int:
    """
    Pull 工作流：
    1. (可选) 执行 git pull
    2. 从历史恢复当前项目配置
    """
    total_steps = 2

    print()
    print_divider("═")
    print(f"  {C.BOLD}{C.CYAN}Pull 工作流{C.RESET}")
    print_divider("═")

    # 步骤 1: Git 操作
    print(C.step(1, total_steps, "Git 操作"))
    if interactive and not dry_run:
        _interactive_git_pull(component_root)
    elif run_git:
        if dry_run:
            print(C.ok("[dry-run] 跳过 git 执行"))
        else:
            args_list = shlex.split(git_args, posix=False) if git_args.strip() else []
            result = git_run(component_root, "pull", *args_list)
            if result.returncode != 0:
                print(C.err(f"git pull 失败，退出码: {result.returncode}"))
                return result.returncode
    else:
        print(C.info("跳过 git pull（未指定 --run-git）"))

    # 步骤 2: 询问并恢复配置
    print()
    print(C.step(2, total_steps, "检查历史配置"))
    saved = history.load_project_state(project)
    if saved:
        print(C.info(f"发现项目 [{C.BOLD}{project.name}{C.RESET}] 的历史配置:"))
        print(f"    子组件启用: {saved.enabled_sub_count()}/{len(saved.subcomponents)}")
        print(f"    音频启用:   {saved.enabled_audio_count()}/{len(saved.audio_defaults)}")
        if dry_run:
            print(C.ok("[dry-run] 跳过配置恢复"))
        else:
            if confirm(f"  {C.CYAN}是否从历史恢复配置?{C.RESET}", default_yes=True):
                restore_project_state(project, cmake, history, dry_run=False)
            else:
                print(C.info("保持当前状态，可稍后通过菜单恢复"))
    else:
        print(C.info("未找到历史配置，保持当前默认状态（通常为全 OFF）"))

    print()
    print(C.ok(f"{C.BOLD}Pull 工作流完成{C.RESET}"))
    print_divider("═")
    return 0


def _interactive_git_push(component_root: Path) -> None:
    """交互式 git push 流程"""
    # 显示 git 状态
    status = git_status_summary(component_root)
    branch = git_current_branch(component_root)

    if branch:
        print(f"  当前分支: {C.BOLD}{branch}{C.RESET}")
    if status:
        print(f"  工作区状态:")
        for line in status.splitlines():
            print(f"    {line}")
    print()

    if not git_has_remote(component_root):
        print(C.warn("未检测到远程仓库，跳过 git 操作"))
        return

    # 1. git add
    if confirm(f"  {C.CYAN}是否执行 git add -A ?{C.RESET}", default_yes=True):
        result = git_run(component_root, "add", "-A")
        if result.returncode != 0:
            print(C.err("git add 失败"))
            return

    # 2. git commit
    if confirm(f"  {C.CYAN}是否执行 git commit ?{C.RESET}", default_yes=True):
        msg = input(f"  commit message (留空使用默认): ").strip()
        if not msg:
            msg = f"toolkit: push - disable all components ({now_str()})"
        result = git_run(component_root, "commit", "-m", msg)
        if result.returncode != 0:
            print(C.warn("git commit 返回非零（可能没有变更需要提交）"))

    # 3. git push
    if confirm(f"  {C.CYAN}是否执行 git push ?{C.RESET}", default_yes=True):
        remote_branch = input(f"  push 参数 (留空使用默认): ").strip()
        if remote_branch:
            args = shlex.split(remote_branch, posix=False)
            result = git_run(component_root, "push", *args)
        else:
            result = git_run(component_root, "push")
        if result.returncode != 0:
            print(C.err(f"git push 失败，退出码: {result.returncode}"))


def _interactive_git_pull(component_root: Path) -> None:
    """交互式 git pull 流程"""
    branch = git_current_branch(component_root)
    if branch:
        print(f"  当前分支: {C.BOLD}{branch}{C.RESET}")

    if not git_has_remote(component_root):
        print(C.warn("未检测到远程仓库，跳过 git pull"))
        return

    if confirm(f"  {C.CYAN}是否执行 git pull ?{C.RESET}", default_yes=True):
        remote_branch = input(f"  pull 参数 (留空使用默认): ").strip()
        if remote_branch:
            args = shlex.split(remote_branch, posix=False)
            result = git_run(component_root, "pull", *args)
        else:
            result = git_run(component_root, "pull")
        if result.returncode != 0:
            print(C.err(f"git pull 失败，退出码: {result.returncode}"))
    else:
        print(C.info("跳过 git pull"))


# ============================================================================
# Init 工作流
# ============================================================================
def workflow_init(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
    component_root: Path,
    dry_run: bool = False,
) -> int:
    """
    Init 工作流 — 首次 git clone 后的初始化：
    1. 初始化并拉取 git submodules
    2. 确认所有子组件处于 OFF 状态（拉取默认）
    3. 如有历史配置，提示是否恢复
    """
    total_steps = 3

    print()
    print_divider("═")
    print(f"  {C.BOLD}{C.CYAN}Init 工作流 — 首次初始化{C.RESET}")
    print_divider("═")

    # 步骤 1: 子模块
    print(C.step(1, total_steps, "初始化 Git 子模块"))
    submodules = git_submodule_status(component_root)
    if not submodules:
        print(C.info("未定义子模块，跳过"))
    else:
        uninit = [sm for sm in submodules if not sm.initialized]
        if not uninit:
            print(C.ok(f"所有 {len(submodules)} 个子模块已初始化"))
            print_submodule_status(component_root)
        else:
            print(C.info(f"发现 {len(uninit)} 个未初始化的子模块:"))
            for sm in uninit:
                print(f"    {C.RED}✗{C.RESET} {sm.path}  {C.DIM}({sm.url}){C.RESET}")
            print()
            if dry_run:
                print(C.ok("[dry-run] 跳过子模块初始化"))
            else:
                code = git_submodule_init_update(component_root)
                if code != 0:
                    print(C.err("子模块初始化失败，请检查网络连接后重试"))
                    print(C.info("可手动执行: git submodule init && git submodule update --recursive"))
                    return code
                # 刷新状态
                print()
                print_submodule_status(component_root)

    # 步骤 2: 确认组件状态
    print()
    print(C.step(2, total_steps, "检查组件开关状态"))
    state = cmake.load_state()
    if state.is_all_disabled():
        print(C.ok("所有子组件和音频参数已处于 OFF 状态（拉取默认）"))
    else:
        print(C.warn(f"当前有 {state.enabled_sub_count()} 个子组件和 {state.enabled_audio_count()} 个音频参数处于启用状态"))
        print(C.info("预期拉取后应为全 OFF，可能是仓库内残留修改"))

    # 步骤 3: 恢复历史
    print()
    print(C.step(3, total_steps, "检查历史配置"))
    saved = history.load_project_state(project)
    if saved:
        print(C.info(f"发现项目 [{C.BOLD}{project.name}{C.RESET}] 的历史配置:"))
        print(f"    子组件启用: {saved.enabled_sub_count()}")
        print(f"    音频启用:   {saved.enabled_audio_count()}")
        if not dry_run:
            if confirm(f"  {C.CYAN}是否从历史恢复配置?{C.RESET}", default_yes=True):
                restore_project_state(project, cmake, history)
            else:
                print(C.info("保持当前全 OFF 状态，可稍后通过菜单恢复"))
        else:
            print(C.ok("[dry-run] 跳过配置恢复"))
    else:
        print(C.info(f"未找到项目 [{project.name}] 的历史配置"))
        print(C.info("提示: 启用需要的子组件后，使用 save 命令保存配置"))

    print()
    print(C.ok(f"{C.BOLD}Init 完成{C.RESET}"))
    print_divider("═")
    return 0


# ============================================================================
# 子工具启动器
# ============================================================================
def launch_subtool(component_root: Path, rel_script: str, extra_args: Optional[List[str]] = None) -> int:
    """
    启动子工具脚本（作为子菜单打开）。
    子工具独立维护，通过 subprocess 调用。
    """
    script_path = component_root / rel_script
    if not script_path.exists():
        print(C.err(f"子工具脚本不存在: {script_path}"))
        return 1

    cmd = [sys.executable, str(script_path)]
    if extra_args:
        cmd.extend(extra_args)

    print()
    print_divider("═")
    print(f"  {C.BOLD}{C.CYAN}启动子工具: {rel_script}{C.RESET}")
    print(f"  {C.DIM}$ {' '.join(cmd)}{C.RESET}")
    print_divider("═")
    print()

    try:
        result = subprocess.run(cmd, cwd=str(component_root))
        return result.returncode
    except Exception as e:
        print(C.err(f"启动子工具失败: {e}"))
        return 1


# 子工具注册表（名称, 相对路径, 描述）
SUBTOOLS: List[Tuple[str, str, str]] = [
    ("a", "otool_audio/check_and_update_material.py", "音频素材管理（扫描/配置/代码生成）"),
]


# ============================================================================
# 交互式菜单
# ============================================================================
def run_menu(
    project: ProjectContext,
    cmake: CMakeStateManager,
    history: ToolkitHistory,
    component_root: Path,
) -> int:
    while True:
        clear_terminal()
        state = cmake.load_state()
        branch = git_current_branch(component_root)

        print(BANNER)

        # 项目信息
        print(f"  {C.BOLD}项目:{C.RESET}  {C.CYAN}{project.name}{C.RESET}")
        print(f"  {C.BOLD}路径:{C.RESET}  {C.DIM}{project.path}{C.RESET}")
        if branch:
            print(f"  {C.BOLD}分支:{C.RESET}  {C.MAGENTA}{branch}{C.RESET}")

        e_sub = state.enabled_sub_count()
        e_aud = state.enabled_audio_count()
        status_color = C.GREEN if state.is_all_disabled() else C.YELLOW
        print(f"  {C.BOLD}状态:{C.RESET}  {status_color}子组件 {e_sub}/{len(state.subcomponents)}  |  音频 {e_aud}/{len(state.audio_defaults)}{C.RESET}")

        print()
        print_divider("─")
        print(f"  {C.BOLD}配置管理{C.RESET}")
        print_divider("─")
        print(f"  {C.BOLD}1{C.RESET})  查看当前配置详情")
        print(f"  {C.BOLD}2{C.RESET})  编辑子组件开关")
        print(f"  {C.BOLD}3{C.RESET})  编辑音频参数开关")
        print(f"  {C.BOLD}4{C.RESET})  全部禁用（恢复到拉取默认状态）")

        print()
        print_divider("─")
        print(f"  {C.BOLD}历史记录{C.RESET}")
        print_divider("─")
        print(f"  {C.BOLD}5{C.RESET})  保存当前配置到历史")
        print(f"  {C.BOLD}6{C.RESET})  从历史恢复当前项目配置")
        print(f"  {C.BOLD}7{C.RESET})  查看/管理历史项目列表")

        print()
        print_divider("─")
        print(f"  {C.BOLD}推送 / 拉取 / 初始化{C.RESET}")
        print_divider("─")
        print(f"  {C.BOLD}8{C.RESET})  {C.YELLOW}Push{C.RESET} 工作流（保存 → 全部禁用 → git push → 恢复）")
        print(f"  {C.BOLD}9{C.RESET})  {C.GREEN}Pull{C.RESET} 工作流（git pull → 询问恢复配置）")
        print(f"  {C.BOLD}i{C.RESET})  {C.CYAN}Init{C.RESET} 首次初始化（submodule init → 恢复配置）")

        print()
        print_divider("─")
        print(f"  {C.BOLD}子工具{C.RESET}")
        print_divider("─")
        for key, rel_path, desc in SUBTOOLS:
            script_exists = (component_root / rel_path).exists()
            status_mark = "" if script_exists else f"  {C.RED}(缺失){C.RESET}"
            print(f"  {C.BOLD}{key}{C.RESET})  {C.MAGENTA}{desc}{C.RESET}{status_mark}")

        print()
        print_divider("─")
        print(f"  {C.BOLD}g{C.RESET})  查看 Git 状态 / 子模块")
        print(f"  {C.BOLD}0{C.RESET})  退出")
        print_divider("═")

        try:
            choice = input(f"  {C.BOLD}选择操作:{C.RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0

        # ── 1: 查看当前配置 ──
        if choice == "1":
            clear_terminal()
            print_state_summary(state)
            wait_key()
            continue

        # ── 2: 编辑子组件开关 ──
        if choice == "2":
            updated = interactive_toggle(
                title="子组件开关编辑",
                options=state.subcomponents,
                ordered_keys=SUBCOMPONENT_OPTIONS,
                labels=SUBCOMPONENT_LABELS,
            )
            if updated is not None:
                state.subcomponents = updated
                cmake.apply_state(state)
                print(C.ok("子组件开关已写入 CMakeLists.txt"))
                wait_key()
            continue

        # ── 3: 编辑音频参数开关 ──
        if choice == "3":
            if not state.audio_defaults:
                print(C.warn("当前未发现 AUDIO_FILE_CONFIGS 配置项"))
                print(C.info("提示: 需要先启用 ENABLE_OTOOL_AUDIO 子组件"))
                wait_key()
                continue

            updated = interactive_toggle(
                title="音频嵌入参数开关编辑",
                options=state.audio_defaults,
            )
            if updated is not None:
                state.audio_defaults = updated
                cmake.apply_state(state)
                print(C.ok("音频参数开关已写入 CMakeLists.txt"))
                wait_key()
            continue

        # ── 4: 全部禁用 ──
        if choice == "4":
            if state.is_all_disabled():
                print(C.info("当前已是全部禁用状态"))
                wait_key()
                continue
            print()
            print(C.warn("即将把所有子组件和音频参数设为 OFF"))
            if confirm("  确认操作?"):
                disable_all(cmake, dry_run=False)
            else:
                print(C.info("操作已取消"))
            wait_key()
            continue

        # ── 5: 保存配置到历史 ──
        if choice == "5":
            print()
            save_current_project_state(project, cmake, history)
            wait_key()
            continue

        # ── 6: 从历史恢复配置 ──
        if choice == "6":
            print()
            saved = history.load_project_state(project)
            if not saved:
                print(C.warn(f"未找到项目 [{project.name}] 的历史配置"))
                print(C.info("提示: 可先用选项 5 保存当前配置，或从选项 7 选择其他项目"))
                wait_key()
                continue

            print(C.info(f"即将从历史恢复项目: {C.BOLD}{project.name}{C.RESET}"))
            print(f"  历史中启用的子组件: {saved.enabled_sub_count()}")
            print(f"  历史中启用的音频:   {saved.enabled_audio_count()}")
            if confirm("  确认恢复?", default_yes=True):
                restore_project_state(project, cmake, history, dry_run=False)
            else:
                print(C.info("操作已取消"))
            wait_key()
            continue

        # ── 7: 查看/管理历史列表 ──
        if choice == "7":
            _menu_history_list(history, project)
            continue

        # ── 8: Push 工作流 ──
        if choice == "8":
            clear_terminal()
            workflow_push(
                project=project,
                cmake=cmake,
                history=history,
                dry_run=False,
                run_git=False,
                git_args="",
                component_root=component_root,
                interactive=True,
            )
            wait_key()
            continue

        # ── 9: Pull 工作流 ──
        if choice == "9":
            clear_terminal()
            workflow_pull(
                project=project,
                cmake=cmake,
                history=history,
                dry_run=False,
                run_git=False,
                git_args="",
                component_root=component_root,
                interactive=True,
            )
            wait_key()
            continue

        # ── i: Init 工作流 ──
        if choice == "i":
            clear_terminal()
            workflow_init(
                project=project,
                cmake=cmake,
                history=history,
                component_root=component_root,
            )
            wait_key()
            continue

        # ── g: Git 状态 / 子模块 ──
        if choice == "g":
            clear_terminal()
            print_section("Git 状态")
            branch = git_current_branch(component_root)
            status = git_status_summary(component_root)
            if branch:
                print(f"  分支: {C.BOLD}{branch}{C.RESET}")
            if status:
                print(f"  状态:")
                for line in status.splitlines():
                    print(f"    {line}")
            else:
                print(C.warn("无法获取 git 状态（可能不是 git 仓库）"))

            # 显示子模块状态
            print()
            print_submodule_status(component_root)
            wait_key()
            continue

        # ── 子工具快捷键 ──
        subtool_match = [t for t in SUBTOOLS if t[0] == choice]
        if subtool_match:
            _, rel_path, desc = subtool_match[0]
            clear_terminal()
            launch_subtool(component_root, rel_path)
            wait_key()
            continue

        # ── 0: 退出 ──
        if choice in ("0", "q", "quit", "exit"):
            return 0


def _menu_history_list(history: ToolkitHistory, current_project: ProjectContext) -> None:
    """历史项目列表管理子菜单"""
    while True:
        clear_terminal()
        projects = history.list_projects()

        print_section("历史项目列表")

        if not projects:
            print(f"  {C.DIM}(暂无历史记录){C.RESET}")
            print()
            print(f"  {C.DIM}提示: 使用主菜单选项 5 保存当前配置{C.RESET}")
            wait_key()
            return

        for idx, (key, meta) in enumerate(projects, start=1):
            pname = meta.get("project_name", "<unknown>")
            ppath = meta.get("project_path", "<unknown>")
            saved_at = meta.get("saved_at", "<unknown>")

            # 标记当前项目
            is_current = (key == current_project.key)
            marker = f"  {C.GREEN}◂ 当前{C.RESET}" if is_current else ""

            e_sub = sum(1 for v in meta.get("subcomponents", {}).values() if v)
            e_aud = sum(1 for v in meta.get("audio_defaults", {}).values() if v)

            print(f"  {C.BOLD}{idx:2d}{C.RESET}. {C.CYAN}{pname}{C.RESET}{marker}")
            print(f"      路径:    {C.DIM}{ppath}{C.RESET}")
            print(f"      保存于:  {saved_at}")
            print(f"      子组件: {e_sub} ON  |  音频: {e_aud} ON")
            print()

        print_divider("─")
        print(f"  {C.DIM}输入序号查看详情  |  d+序号 删除记录  |  q 返回{C.RESET}")
        print_divider("─")

        try:
            raw = input(f"  输入: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return

        if raw in ("q", "quit", ""):
            return

        # 删除记录
        if raw.startswith("d") and raw[1:].strip().isdigit():
            num = int(raw[1:].strip())
            if 1 <= num <= len(projects):
                key, meta = projects[num - 1]
                pname = meta.get("project_name", "<unknown>")
                if confirm(f"  确认删除历史记录 [{pname}] ?"):
                    if history.delete_project(key):
                        print(C.ok(f"已删除: {pname}"))
                    else:
                        print(C.err("删除失败"))
                    wait_key()
            continue

        # 查看详情
        if raw.isdigit():
            num = int(raw)
            if 1 <= num <= len(projects):
                key, meta = projects[num - 1]
                clear_terminal()
                pname = meta.get("project_name", "<unknown>")
                print_section(f"历史详情: {pname}")
                print(f"  Key:     {C.DIM}{key}{C.RESET}")
                print(f"  路径:    {meta.get('project_path', '?')}")
                print(f"  保存于:  {meta.get('saved_at', '?')}")
                print()

                sub = meta.get("subcomponents", {})
                print(f"  {C.BOLD}子组件:{C.RESET}")
                for name in SUBCOMPONENT_OPTIONS:
                    enabled = sub.get(name, False)
                    label = SUBCOMPONENT_LABELS.get(name, "")
                    display = f"{name}  {C.DIM}({label}){C.RESET}" if label else name
                    print(C.on_off(enabled, display))
                print()

                aud = meta.get("audio_defaults", {})
                print(f"  {C.BOLD}音频参数:{C.RESET}")
                if not aud:
                    print(f"    {C.DIM}(无){C.RESET}")
                else:
                    for name in sorted(aud.keys()):
                        print(C.on_off(aud[name], f"ENABLE_{name}"))

                wait_key()
            continue


# ============================================================================
# CLI 参数解析
# ============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="otool_esp_component 多组件管理工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python otool_esp_component_toolkit.py                       # 交互式菜单
  python otool_esp_component_toolkit.py status                # 查看当前状态
  python otool_esp_component_toolkit.py save                  # 保存当前项目配置
  python otool_esp_component_toolkit.py restore               # 从历史恢复配置
  python otool_esp_component_toolkit.py push                  # push 工作流（交互式 git）
  python otool_esp_component_toolkit.py push --run-git        # push 工作流（自动 git push）
  python otool_esp_component_toolkit.py pull                  # pull 工作流（交互式 git）
  python otool_esp_component_toolkit.py pull --run-git        # pull 工作流（自动 git pull）
  python otool_esp_component_toolkit.py init                  # 首次初始化（拉取子模块+恢复配置）
  python otool_esp_component_toolkit.py disable-all           # 全部禁用
  python otool_esp_component_toolkit.py list-history          # 查看历史
  python otool_esp_component_toolkit.py --project-id corep4   # 指定项目ID
""",
    )

    parser.add_argument(
        "--component-root",
        type=str,
        default=None,
        help="组件根目录（默认：脚本所在目录）",
    )
    parser.add_argument(
        "--project-id",
        type=str,
        default=None,
        help="项目ID（可选，建议用于区分多项目）",
    )
    parser.add_argument(
        "--project-path",
        type=str,
        default=None,
        help="项目路径（默认当前工作目录）",
    )

    sub = parser.add_subparsers(dest="command")

    sub.add_parser("menu", help="启动交互式菜单（默认）")
    sub.add_parser("status", help="查看当前配置状态")
    sub.add_parser("save", help="保存当前配置到 .toolkit_history")

    restore = sub.add_parser("restore", help="从 .toolkit_history 恢复当前项目配置")
    restore.add_argument("--dry-run", action="store_true", help="仅预览，不写文件")

    disable = sub.add_parser("disable-all", help="将子组件和音频参数全部设为 OFF")
    disable.add_argument("--dry-run", action="store_true", help="仅预览，不写文件")

    push = sub.add_parser("push", help="push 工作流：保存历史 + 全部禁用 + 可选 git push")
    push.add_argument("--dry-run", action="store_true", help="仅预览，不写文件")
    push.add_argument("--run-git", action="store_true", help="自动执行 git add/commit/push（非交互）")
    push.add_argument("--git-args", type=str, default="", help="git push 额外参数")
    push.add_argument("--commit-msg", type=str, default="", help="git commit 消息")

    pull = sub.add_parser("pull", help="pull 工作流：可选 git pull + 恢复历史")
    pull.add_argument("--dry-run", action="store_true", help="仅预览，不写文件")
    pull.add_argument("--run-git", action="store_true", help="自动执行 git pull（非交互）")
    pull.add_argument("--git-args", type=str, default="", help="git pull 额外参数")

    init_cmd = sub.add_parser("init", help="首次初始化：拉取子模块 + 恢复历史配置")
    init_cmd.add_argument("--dry-run", action="store_true", help="仅预览，不执行操作")

    sub.add_parser("list-history", help="列出历史项目记录")

    return parser


# ============================================================================
# 入口
# ============================================================================
def main() -> int:
    C.enable_win_ansi()

    parser = build_parser()
    args = parser.parse_args()

    component_root = Path(args.component_root).resolve() if args.component_root else Path(__file__).resolve().parent
    cmake_file = component_root / "CMakeLists.txt"
    history_file = component_root / HISTORY_DIR_NAME / HISTORY_FILE_NAME

    if not cmake_file.exists():
        print(C.err(f"未找到 CMakeLists.txt: {cmake_file}"))
        return 1

    project = resolve_project_context(args.project_id, args.project_path)
    cmake = CMakeStateManager(cmake_file)
    history = ToolkitHistory(history_file)

    command = args.command or "menu"

    # ── menu ──
    if command == "menu":
        return run_menu(project, cmake, history, component_root)

    # ── status ──
    if command == "status":
        state = cmake.load_state()
        print_state_summary(state)
        return 0

    # ── save ──
    if command == "save":
        save_current_project_state(project, cmake, history)
        return 0

    # ── restore ──
    if command == "restore":
        ok = restore_project_state(project, cmake, history, dry_run=bool(args.dry_run))
        return 0 if ok else 1

    # ── disable-all ──
    if command == "disable-all":
        disable_all(cmake, dry_run=bool(args.dry_run))
        return 0

    # ── init ──
    if command == "init":
        return workflow_init(
            project=project,
            cmake=cmake,
            history=history,
            component_root=component_root,
            dry_run=bool(args.dry_run),
        )

    # ── push ──
    if command == "push":
        run_git = bool(args.run_git)
        # 非 --run-git 模式下，CLI push 也进入交互式 git 提示
        interactive_git = not run_git and sys.stdin.isatty()

        if run_git and not args.dry_run:
            # 自动 git push: 先 add + commit
            commit_msg = args.commit_msg or f"toolkit: push - disable all components ({now_str()})"

            code = workflow_push(
                project=project, cmake=cmake, history=history,
                dry_run=bool(args.dry_run), run_git=False,
                git_args="", component_root=component_root,
            )
            if code != 0:
                return code

            print()
            print(C.step(3, 3, "自动 git 操作"))
            git_run(component_root, "add", "-A")
            git_run(component_root, "commit", "-m", commit_msg)
            args_list = shlex.split(args.git_args, posix=False) if args.git_args.strip() else []
            result = git_run(component_root, "push", *args_list)
            if result.returncode != 0:
                print(C.err(f"git push 失败: {result.returncode}"))
                return result.returncode
            print(C.ok("git push 完成"))
            return 0

        return workflow_push(
            project=project, cmake=cmake, history=history,
            dry_run=bool(args.dry_run),
            run_git=False, git_args="",
            component_root=component_root,
            interactive=interactive_git,
        )

    # ── pull ──
    if command == "pull":
        run_git = bool(args.run_git)
        interactive_git = not run_git and sys.stdin.isatty()

        return workflow_pull(
            project=project, cmake=cmake, history=history,
            dry_run=bool(args.dry_run),
            run_git=run_git,
            git_args=str(args.git_args),
            component_root=component_root,
            interactive=interactive_git,
        )

    # ── list-history ──
    if command == "list-history":
        projects = history.list_projects()
        if not projects:
            print(C.info("暂无历史记录"))
            return 0

        print()
        print_section("历史项目记录")
        for idx, (key, meta) in enumerate(projects, start=1):
            pname = meta.get("project_name", "<unknown>")
            ppath = meta.get("project_path", "<unknown>")
            saved_at = meta.get("saved_at", "<unknown>")
            e_sub = sum(1 for v in meta.get("subcomponents", {}).values() if v)
            e_aud = sum(1 for v in meta.get("audio_defaults", {}).values() if v)
            print(f"  {idx:2d}. {C.BOLD}{pname}{C.RESET}  |  {saved_at}")
            print(f"      {C.DIM}{ppath}{C.RESET}")
            print(f"      子组件: {e_sub} ON  |  音频: {e_aud} ON")
        print()
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())

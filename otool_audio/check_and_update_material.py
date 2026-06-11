#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
音频文件自动更新脚本

功能：
1. 扫描 audio_playback_material 目录下的所有 PCM 文件
2. 读取 CMakeLists.txt 获取当前启用的音频文件
3. 提供交互式菜单供用户选择要启用的音频文件
4. 自动生成/更新以下内容：
   - 头文件中的枚举定义
   - CPP 文件中的外部声明、数据获取函数和表项
   - CMakeLists.txt 中的音频文件配置和 CMake 选项

设计原则：
- 使用正则表达式精确匹配，避免字符串硬编码
- 交互式菜单，用户友好
- 自动读取现有配置，保留用户选择
- 完整的错误处理和备份机制
"""

import os
import re
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass, field
import shutil
from datetime import datetime

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        try:
            _stream.reconfigure(errors="replace")
        except Exception:
            pass

# 尝试复用 toolkit 的 ANSI 颜色工具和 msvcrt 支持
try:
    _toolkit_dir = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(_toolkit_dir))
    from otool_esp_component_toolkit import C, wait_key, confirm, _HAS_MSVCRT
    if _HAS_MSVCRT:
        import msvcrt
except ImportError:
    # fallback: 独立运行时使用内联定义
    import msvcrt  # Windows 键盘输入
    _HAS_MSVCRT = True

    class C:
        RESET = "\033[0m"; BOLD = "\033[1m"; DIM = "\033[2m"
        RED = "\033[31m"; GREEN = "\033[32m"; YELLOW = "\033[33m"
        CYAN = "\033[36m"
        @staticmethod
        def ok(msg): return f"{C.GREEN}✓{C.RESET} {msg}"
        @staticmethod
        def err(msg): return f"{C.RED}✗{C.RESET} {msg}"
        @staticmethod
        def warn(msg): return f"{C.YELLOW}⚠{C.RESET} {msg}"
        @staticmethod
        def info(msg): return f"{C.CYAN}ℹ{C.RESET} {msg}"

    def wait_key(msg="按任意键继续..."):
        print(f"\n{msg}", end="", flush=True)
        msvcrt.getch(); print()

    def confirm(msg, default_yes=False):
        hint = "[Y/n]" if default_yes else "[y/N]"
        try:
            raw = input(f"{msg} {hint}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return False
        return (raw in ("y", "yes")) if raw else default_yes


@dataclass
class AudioFileInfo:
    """音频文件信息"""
    filename: str           # 完整文件名，如 "candy_wind_pcm_1ch_16k_16bit_9s.pcm"
    base_name: str          # 基础名称，如 "candy_wind"
    channels: int           # 声道数 (1 或 2)
    sample_rate: int        # 采样率 (Hz)
    bits: int              # 位宽 (8, 16, 24, 32)
    duration_sec: float    # 时长 (秒)
    enum_name: str         # 枚举名称，如 "AUDIO_FILE_CANDY_WIND_1CH_16K_16B_9S"
    macro_name: str        # 宏名称，如 "USE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S"
    cmake_option_name: str # CMake 选项名，如 "ENABLE_AUDIO_CANDY_WIND_1CH_16K_16BIT_9S"
    getter_func_name: str  # 数据获取函数名，如 "get_candy_wind_1ch_16k_data"
    binary_start_symbol: str  # 二进制起始符号
    binary_end_symbol: str    # 二进制结束符号
    enabled: bool = False   # 是否启用（从 CMakeLists.txt 读取）
    file_size: int = 0      # 文件大小（字节）
    
    @property
    def expected_size(self) -> int:
        """计算理论文件大小（字节）"""
        bytes_per_sample = self.bits // 8
        return int(self.sample_rate * self.channels * bytes_per_sample * self.duration_sec)
    
    @property
    def size_error_percent(self) -> float:
        """计算文件大小误差百分比"""
        if self.expected_size == 0:
            return 0.0
        return abs(self.file_size - self.expected_size) / self.expected_size * 100.0
    
    @property
    def size_error_seconds(self) -> float:
        """计算文件大小误差对应的时长（秒）"""
        bytes_per_sample = self.bits // 8
        bytes_per_second = self.sample_rate * self.channels * bytes_per_sample
        if bytes_per_second == 0:
            return 0.0
        return abs(self.file_size - self.expected_size) / bytes_per_second
    
    def validate_size(self, max_error_seconds: float = 0.5) -> Tuple[bool, str]:
        """
        验证文件大小是否符合预期
        
        Args:
            max_error_seconds: 允许的最大时长误差（秒）
            
        Returns:
            (是否通过, 错误信息)
        """
        error_sec = self.size_error_seconds
        
        if error_sec > max_error_seconds:
            error_msg = (
                f"文件大小校验失败: {self.filename}\n"
                f"  实际大小: {self.file_size:,} bytes ({self.file_size / 1024:.2f} KB)\n"
                f"  理论大小: {self.expected_size:,} bytes ({self.expected_size / 1024:.2f} KB)\n"
                f"  时长误差: {error_sec:.2f} 秒 (超过阈值 {max_error_seconds} 秒)\n"
                f"  百分比误差: {self.size_error_percent:.2f}%\n"
                f"  建议: 请检查文件名中的参数（采样率、声道、位宽、时长）是否与实际内容匹配"
            )
            return (False, error_msg)
        
        return (True, "")


class PCMFileParser:
    """PCM 文件名解析器"""
    
    # 通用 PCM 文件名模式
    # 格式: <name>_pcm_<channels>ch_<sample_rate>_<bits>bit_<duration>.<ext>
    PATTERN = re.compile(
        r'^(?P<base_name>.+?)_pcm'           # 基础名称
        r'_(?P<channels>\d+)ch'              # 声道数
        r'_(?P<sample_rate>[\d.]+)k'        # 采样率（支持小数点，如 44.1k）
        r'_(?P<bits>\d+)bit'                # 位宽
        r'_(?P<duration>[\d.]+)s'           # 时长
        r'\.pcm$',                          # 扩展名
        re.IGNORECASE
    )
    
    @staticmethod
    def parse(filename: str) -> Optional[AudioFileInfo]:
        """解析 PCM 文件名"""
        match = PCMFileParser.PATTERN.match(filename)
        if not match:
            return None
        
        groups = match.groupdict()
        base_name = groups['base_name']
        channels = int(groups['channels'])
        
        # 解析采样率（处理 44.1k 这种情况）
        sample_rate_str = groups['sample_rate']
        sample_rate_khz = float(sample_rate_str)
        sample_rate = int(sample_rate_khz * 1000)
        
        bits = int(groups['bits'])
        duration_sec = float(groups['duration'])
        
        # 生成枚举名称
        enum_name = PCMFileParser._generate_enum_name(
            base_name, channels, sample_rate, bits, duration_sec
        )
        
        # 生成宏名称
        macro_name = PCMFileParser._generate_macro_name(
            base_name, channels, sample_rate, bits, duration_sec
        )
        
        # 生成 CMake 选项名（去掉 USE_ 前缀，改为 ENABLE_ 前缀）
        cmake_option_name = "ENABLE_" + macro_name[4:]  # 去掉 "USE_"
        
        # 生成数据获取函数名
        getter_func_name = PCMFileParser._generate_getter_func_name(
            base_name, channels, sample_rate
        )
        
        # 生成二进制符号名（用于链接器）
        binary_symbols = PCMFileParser._generate_binary_symbols(filename)
        
        return AudioFileInfo(
            filename=filename,
            base_name=base_name,
            channels=channels,
            sample_rate=sample_rate,
            bits=bits,
            duration_sec=duration_sec,
            enum_name=enum_name,
            macro_name=macro_name,
            cmake_option_name=cmake_option_name,
            getter_func_name=getter_func_name,
            binary_start_symbol=binary_symbols[0],
            binary_end_symbol=binary_symbols[1]
        )
    
    @staticmethod
    def _generate_enum_name(base: str, channels: int, rate: int, bits: int, duration: float) -> str:
        """生成枚举名称"""
        # 将基础名称转换为大写并替换特殊字符
        base_upper = base.upper().replace('-', '_').replace('.', '_')
        
        # 采样率简化表示
        if rate == 44100:
            rate_str = "44K"
        elif rate >= 1000:
            rate_str = f"{rate // 1000}K"
        else:
            rate_str = str(rate)
        
        # 时长简化（整数秒）
        duration_str = str(int(duration))
        
        return f"AUDIO_FILE_{base_upper}_{channels}CH_{rate_str}_{bits}B_{duration_str}S"
    
    @staticmethod
    def _generate_macro_name(base: str, channels: int, rate: int, bits: int, duration: float) -> str:
        """生成宏名称"""
        base_upper = base.upper().replace('-', '_').replace('.', '_')
        
        # 采样率表示（保留小数点信息）
        if rate == 44100:
            rate_str = "44K"
        elif rate >= 1000:
            rate_str = f"{rate // 1000}K"
        else:
            rate_str = str(rate)
        
        # 时长处理（整数秒）
        duration_str = str(int(duration))
        
        return f"USE_AUDIO_{base_upper}_{channels}CH_{rate_str}_{bits}BIT_{duration_str}S"
    
    @staticmethod
    def _generate_getter_func_name(base: str, channels: int, rate: int) -> str:
        """生成数据获取函数名"""
        base_lower = base.lower().replace('-', '_').replace('.', '_')
        
        if rate >= 1000:
            rate_str = f"{rate // 1000}k"
        else:
            rate_str = str(rate)
        
        return f"get_{base_lower}_{channels}ch_{rate_str}_data"
    
    @staticmethod
    def _generate_binary_symbols(filename: str) -> Tuple[str, str]:
        """生成二进制链接符号名"""
        # 将文件名转换为符号名：替换特殊字符为下划线
        symbol_base = filename.replace('.', '_').replace('-', '_')
        start_symbol = f"_binary_{symbol_base}_start"
        end_symbol = f"_binary_{symbol_base}_end"
        return (start_symbol, end_symbol)


class CMakeConfigParser:
    """CMake 配置文件解析器"""
    
    @staticmethod
    def parse_audio_configs(cmake_file: Path) -> Dict[str, bool]:
        """
        解析 CMakeLists.txt 中的音频文件配置
        返回: {cmake_option_name: enabled_status}
        """
        if not cmake_file.exists():
            return {}
        
        content = cmake_file.read_text(encoding='utf-8')
        enabled_files = {}
        
        # 查找 AUDIO_FILE_CONFIGS 配置块
        config_pattern = re.compile(
            r'set\(AUDIO_FILE_CONFIGS\s+(.*?)\)',
            re.DOTALL
        )
        
        match = config_pattern.search(content)
        if not match:
            return {}
        
        config_block = match.group(1)
        
        # 解析每一行配置
        # 格式: "ID:filename:ON/OFF:rate:channels:bits:description"
        line_pattern = re.compile(r'"([^"]+)"')
        
        for line_match in line_pattern.finditer(config_block):
            config_line = line_match.group(1)
            parts = config_line.split(':')
            
            if len(parts) >= 3:
                audio_id = parts[0].strip()
                default_val = parts[2].strip().upper()
                
                # CMake 选项名格式: ENABLE_{audio_id}
                cmake_option = f"ENABLE_{audio_id}"
                enabled_files[cmake_option] = (default_val == "ON")
        
        return enabled_files


class CodeUpdater:
    """代码自动更新器"""
    
    def __init__(self, project_root: Path = None):
        """
        初始化代码更新器
        
        Args:
            project_root: 项目根目录（可选）
                        如果不提供，将自动检测为脚本所在目录
        """
        if project_root is None:
            # 自动检测：使用脚本所在目录
            script_dir = Path(__file__).parent.resolve()
            project_root = script_dir
        else:
            project_root = project_root.resolve()
        
        self.project_root = project_root
        self.audio_dir = project_root / "audio_playback_material"
        self.header_file = project_root / "include" / "audio_types.h"
        self.cpp_file = project_root / "audio_playback.cpp"
        self.cmake_file = project_root.parent / "CMakeLists.txt"
        
        # 备份目录
        self.backup_dir = project_root / ".backups" / datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def scan_audio_files(self, show_details: bool = True, validate_files: bool = True) -> List[AudioFileInfo]:
        """
        扫描音频文件目录（全量扫描，不保留已删除文件的配置）
        
        Args:
            show_details: 是否显示详细信息
            validate_files: 是否验证文件大小
            
        Returns:
            音频文件信息列表
            
        Raises:
            FileNotFoundError: 音频目录不存在
            ValueError: 文件大小校验失败
        """
        if not self.audio_dir.exists():
            raise FileNotFoundError(f"音频目录不存在: {self.audio_dir}")
        
        # 读取现有的 CMake 配置
        existing_config = CMakeConfigParser.parse_audio_configs(self.cmake_file)
        
        audio_files = []
        validation_errors = []
        
        for file_path in self.audio_dir.glob("*.pcm"):
            info = PCMFileParser.parse(file_path.name)
            if info:
                # 获取文件大小
                info.file_size = file_path.stat().st_size
                
                # 从现有配置中读取启用状态（如果存在）
                info.enabled = existing_config.get(info.cmake_option_name, False)
                
                # 文件大小校验
                if validate_files:
                    is_valid, error_msg = info.validate_size(max_error_seconds=0.5)
                    if not is_valid:
                        validation_errors.append(error_msg)
                
                audio_files.append(info)
                
                if show_details:
                    status = "[X]" if info.enabled else "[ ]"
                    size_kb = info.file_size / 1024
                    expected_kb = info.expected_size / 1024
                    error_sec = info.size_error_seconds
                    
                    # 显示校验状态
                    if error_sec > 0.5:
                        verify_status = "⚠ FAIL"
                    elif error_sec > 0.1:
                        verify_status = "⚠ WARN"
                    else:
                        verify_status = "✓ OK  "
                    
                    print(f"{status} {verify_status} {file_path.name}")
                    print(f"           → 大小: {size_kb:.1f} KB (理论: {expected_kb:.1f} KB, 误差: {error_sec:.3f}s)")
                    print(f"           → 选项: {info.cmake_option_name}")
            else:
                if show_details:
                    print(f"✗ 跳过: {file_path.name} (文件名格式不符合规范)")
        
        # 如果有校验错误，抛出异常
        if validation_errors:
            error_summary = "\n\n".join(validation_errors)
            raise ValueError(f"\n{'=' * 70}\n文件大小校验失败！\n{'=' * 70}\n\n{error_summary}\n\n{'=' * 70}")
        
        return sorted(audio_files, key=lambda x: x.filename)
    
    def show_selection_menu(self, audio_files: List[AudioFileInfo]) -> Optional[List[AudioFileInfo]]:
        """
        显示交互式选择菜单（类似 menuconfig 风格）
        
        键盘操作:
        - 上/下箭头: 移动光标
        - 空格键: 切换选中/不选中
        - 回车键: 完成选择
        - ESC/Ctrl+C: 取消操作
        """
        if not audio_files:
            print("⚠ 没有可选择的音频文件")
            return None
        
        current_index = 0  # 当前光标位置
        
        try:
            while True:
                # 清屏并显示菜单
                self._display_menu(audio_files, current_index)
                
                # 读取键盘输入
                key = self._get_key()
                
                if key == 'UP':
                    current_index = (current_index - 1) % len(audio_files)
                elif key == 'DOWN':
                    current_index = (current_index + 1) % len(audio_files)
                elif key == 'SPACE':
                    # 切换当前项的选中状态
                    audio_files[current_index].enabled = not audio_files[current_index].enabled
                elif key == 'ENTER':
                    # 完成选择
                    break
                elif key == 'ESC' or key == 'CTRL_C':
                    print("\n\n已取消操作")
                    return None
        
        except KeyboardInterrupt:
            print("\n\n已取消操作")
            return None
        
        print("\n\n选择完成！")
        return audio_files
    
    def _display_menu(self, audio_files: List[AudioFileInfo], current_index: int):
        """显示菜单界面"""
        # 使用 ANSI 转义码清屏（Windows 10+ 支持）
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("=" * 80)
        print("音频文件选择 (类似 menuconfig)")
        print("=" * 80)
        print("操作: ↑↓ 移动光标 | 空格 切换选择 | 回车 完成 | ESC 取消")
        print("=" * 80)
        print()
        
        # 显示文件列表
        for i, info in enumerate(audio_files):
            # 选中标记
            check = "[X]" if info.enabled else "[ ]"
            
            # 验证状态（调用验证方法）
            is_valid, _ = info.validate_size(max_error_seconds=0.5)
            validation = "✓ OK" if is_valid else "✗ 错误"
            
            # 当前光标位置高亮显示
            if i == current_index:
                # 使用反色显示当前行（ANSI 转义码）
                print(f"\033[7m{check} {validation}   {info.filename}\033[0m")
            else:
                print(f"{check} {validation}   {info.filename}")
            
            # 显示详细信息
            expected_kb = info.expected_size / 1024
            actual_kb = info.file_size / 1024
            print(f"           → 大小: {actual_kb:.1f} KB (理论: {expected_kb:.1f} KB, 误差: {info.size_error_seconds:.3f}s)")
            print(f"           → 选项: ENABLE_{info.enum_name}")
            print()
        
        # 统计信息
        enabled_count = sum(1 for f in audio_files if f.enabled)
        total_size = sum(f.file_size for f in audio_files if f.enabled)
        print("=" * 80)
        print(f"已选择: {enabled_count}/{len(audio_files)} 个文件")
        print(f"总大小: {total_size / 1024:.1f} KB ({total_size / 1024 / 1024:.2f} MB)")
        print("=" * 80)
    
    def _get_key(self) -> str:
        """
        获取键盘输入（Windows）
        
        返回值:
        - 'UP': 上箭头
        - 'DOWN': 下箭头
        - 'SPACE': 空格键
        - 'ENTER': 回车键
        - 'ESC': ESC键
        - 'CTRL_C': Ctrl+C
        """
        try:
            # 读取第一个字节
            ch = msvcrt.getch()
            
            # 普通按键
            if ch == b'\r':  # 回车键
                return 'ENTER'
            elif ch == b' ':  # 空格键
                return 'SPACE'
            elif ch == b'\x1b':  # ESC键
                return 'ESC'
            elif ch == b'\x03':  # Ctrl+C
                return 'CTRL_C'
            elif ch in (b'\x00', b'\xe0'):  # 扩展键（箭头键等）
                # 读取第二个字节
                ch2 = msvcrt.getch()
                if ch2 == b'H':  # 上箭头
                    return 'UP'
                elif ch2 == b'P':  # 下箭头
                    return 'DOWN'
            
            # 未识别的键，返回空
            return ''
        
        except Exception:
            return ''
    
    def backup_files(self):
        """备份相关文件"""
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        
        files_to_backup = [
            self.header_file,
            self.cpp_file,
            self.cmake_file
        ]
        
        for file_path in files_to_backup:
            if file_path.exists():
                backup_path = self.backup_dir / file_path.name
                shutil.copy2(file_path, backup_path)
                print(f"✓ 已备份: {file_path.name} → {backup_path}")
    
    def update_header_enum(self, audio_files: List[AudioFileInfo]) -> bool:
        """
        更新头文件中的枚举定义（全量更新）
        
        注意：此方法会完全替换现有枚举，删除不存在的文件定义
        """
        if not self.header_file.exists():
            print(f"✗ 头文件不存在: {self.header_file}")
            return False
        
        content = self.header_file.read_text(encoding='utf-8')
        
        # 查找 typedef enum { ... } audio_file_type_t; 块
        enum_pattern = re.compile(
            r'(typedef\s+enum\s*\{\s*)([^{}]*?)(\s*AUDIO_FILE_MAX[^\n]*\n\}\s*audio_file_type_t\s*;)',
            re.DOTALL
        )
        
        match = enum_pattern.search(content)
        if not match:
            print("✗ 未找到 audio_file_type_t 枚举定义")
            return False
        
        # 生成新的枚举项（仅包含当前存在的文件）
        enum_items = []
        for i, info in enumerate(audio_files):
            comment = f"///< {info.base_name} {info.channels}通道 {info.sample_rate}Hz {info.bits}bit {info.duration_sec}秒 ({info.filename})"
            if i == 0:
                enum_items.append(f"    {info.enum_name} = 0,  {comment}")
            else:
                enum_items.append(f"    {info.enum_name},     {comment}")
        
        new_enum_body = "\n".join(enum_items)
        new_content = (
            content[:match.start(1)]
            + match.group(1).rstrip()
            + "\n"
            + new_enum_body
            + "\n    "
            + match.group(3).lstrip()
            + content[match.end(3):]
        )
        
        self.header_file.write_text(new_content, encoding='utf-8')
        print(f"✓ 已更新头文件枚举: {len(audio_files)} 个音频文件（全量更新）")
        return True
    
    def update_cpp_file(self, audio_files: List[AudioFileInfo]) -> bool:
        """
        更新 CPP 文件（全量更新）
        
        注意：此方法会完全替换现有配置，删除不存在的文件定义
        """
        if not self.cpp_file.exists():
            print(f"✗ CPP 文件不存在: {self.cpp_file}")
            return False
        
        content = self.cpp_file.read_text(encoding='utf-8')
        
        # 1. 更新外部声明区域
        content = self._update_playback_extern_declarations(content, audio_files)
        
        # 2. 更新数据获取函数区域
        content = self._update_playback_getter_functions(content, audio_files)
        
        # 3. 更新 AUDIO_FILE_TABLE 定义
        content = self._update_file_table(content, audio_files)
        
        self.cpp_file.write_text(content, encoding='utf-8')
        print(f"✓ 已更新 CPP 文件: {len(audio_files)} 个音频文件（全量更新）")
        return True
    
    def _update_extern_declarations(self, content: str, audio_files: List[AudioFileInfo]) -> str:
        """更新外部声明区域"""
        # 查找音频文件嵌入声明区域的边界
        start_marker = "// ============================================================================\n// 音频文件嵌入声明"
        end_marker = "static const char *TAG"
        
        start_pos = content.find(start_marker)
        end_pos = content.find(end_marker)
        
        if start_pos == -1 or end_pos == -1:
            print("⚠ 未找到外部声明区域标记，跳过更新")
            return content
        
        # 生成新的声明
        declarations = [start_marker.split('\n')[0], "// 音频文件嵌入声明 - 使用统一命名规范", "// ============================================================================"]
        
        for info in audio_files:
            declarations.append(f"#ifdef {info.macro_name}")
            declarations.append(f"extern const uint8_t {info.binary_start_symbol}[];")
            declarations.append(f"extern const uint8_t {info.binary_end_symbol}[];")
            declarations.append("#endif")
            declarations.append("")
        
        new_section = "\n".join(declarations)
        return content[:start_pos] + new_section + "\n" + content[end_pos:]
    
    def _update_getter_functions(self, content: str, audio_files: List[AudioFileInfo]) -> str:
        """更新数据获取函数区域"""
        # 查找函数定义区域
        start_marker = "// 各音频文件的数据获取函数"
        end_marker = "struct AudioFileMetadata {"
        
        start_pos = content.find(start_marker)
        end_pos = content.find(end_marker)
        
        if start_pos == -1 or end_pos == -1:
            print("⚠ 未找到数据获取函数区域标记，跳过更新")
            return content
        
        # 生成新的函数定义
        functions = [start_marker]
        
        for info in audio_files:
            functions.append(f"#ifdef {info.macro_name}")
            functions.append(f"static bool {info.getter_func_name}(const uint8_t*& start, size_t& len) {{")
            functions.append(f"    start = {info.binary_start_symbol};")
            functions.append(f"    len = {info.binary_end_symbol} - {info.binary_start_symbol};")
            functions.append("    return true;")
            functions.append("}")
            functions.append("#endif")
            functions.append("")
        
        new_section = "\n".join(functions)
        return content[:start_pos] + new_section + "\n" + content[end_pos:]
    
    def _update_playback_extern_declarations(self, content: str, audio_files: List[AudioFileInfo]) -> str:
        """Update embedded PCM extern declarations in audio_playback.cpp."""
        anchor = "static uint8_t g_silence_chunk[SILENCE_CHUNK_CAPACITY] = {0};"
        anchor_pos = content.find(anchor)
        start_pos = content.find("// ============================================================================", anchor_pos)
        typedef_pos = content.find("typedef bool (*AudioDataGetter)", start_pos)

        if anchor_pos == -1 or start_pos == -1 or typedef_pos == -1:
            print("⚠ 未找到 audio_playback.cpp 的音频嵌入声明区域，跳过更新")
            return content

        declarations = [
            "// ============================================================================",
            "// 音频文件嵌入声明 - 统一命名规范",
            "// ============================================================================"
        ]

        for info in audio_files:
            declarations.append(f"#ifdef {info.macro_name}")
            declarations.append(f"extern const uint8_t {info.binary_start_symbol}[];")
            declarations.append(f"extern const uint8_t {info.binary_end_symbol}[];")
            declarations.append("#endif")
            declarations.append("")

        declarations.extend([
            "// ============================================================================",
            "// 音频文件元数据表",
            "// ============================================================================"
        ])

        new_section = "\n".join(declarations)
        return content[:start_pos] + new_section + "\n" + content[typedef_pos:]

    def _update_playback_getter_functions(self, content: str, audio_files: List[AudioFileInfo]) -> str:
        """Update PCM data getter functions in audio_playback.cpp."""
        typedef_pos = content.find("typedef bool (*AudioDataGetter)")
        end_pos = content.find("struct AudioFileMetadata {", typedef_pos)

        if typedef_pos == -1 or end_pos == -1:
            print("⚠ 未找到 audio_playback.cpp 的数据获取函数区域，跳过更新")
            return content

        start_pos = content.find("\n", typedef_pos)
        if start_pos == -1 or start_pos >= end_pos:
            print("⚠ 未找到 audio_playback.cpp 的数据获取函数区域，跳过更新")
            return content
        start_pos += 1

        functions = [""]

        for info in audio_files:
            functions.append(f"#ifdef {info.macro_name}")
            functions.append(f"static bool {info.getter_func_name}(const uint8_t*& start, size_t& len) {{")
            functions.append(f"    start = {info.binary_start_symbol};")
            functions.append(f"    len = {info.binary_end_symbol} - {info.binary_start_symbol};")
            functions.append("    return true;")
            functions.append("}")
            functions.append("#endif")
            functions.append("")

        new_section = "\n".join(functions)
        return content[:start_pos] + new_section + content[end_pos:]

    def _update_file_table(self, content: str, audio_files: List[AudioFileInfo]) -> str:
        """更新 AUDIO_FILE_TABLE 定义"""
        # 查找表定义区域
        table_pattern = re.compile(
            r'(static const AudioFileMetadata AUDIO_FILE_TABLE\[\]\s*=\s*\{)(.*?)(\};)',
            re.DOTALL
        )
        
        match = table_pattern.search(content)
        if not match:
            print("⚠ 未找到 AUDIO_FILE_TABLE 定义，跳过更新")
            return content
        
        # 生成新的表项
        table_entries = []
        
        for info in audio_files:
            # 确定声道类型
            channels_macro = "AUDIO_CHANNELS_MONO" if info.channels == 1 else "AUDIO_CHANNELS_STEREO"
            
            # 确定位宽类型
            bits_macro = f"I2S_DATA_BIT_WIDTH_{info.bits}BIT"
            
            entry_lines = [
                f"#ifdef {info.macro_name}",
                f'    {{{info.enum_name}, "{info.filename}", {info.sample_rate}, {channels_macro}, {bits_macro}, {info.getter_func_name}}},',
                "#endif"
            ]
            table_entries.extend(entry_lines)
        
        new_table_body = "\n".join(table_entries)
        new_content = content[:match.start(2)] + "\n" + new_table_body + "\n" + content[match.start(3):]
        
        return new_content
    
    def update_cmake_file(self, audio_files: List[AudioFileInfo]) -> bool:
        """更新 CMakeLists.txt 文件（生成新的配置格式）"""
        if not self.cmake_file.exists():
            print(f"⚠ CMakeLists.txt 不存在: {self.cmake_file}")
            return False
        
        content = self.cmake_file.read_text(encoding='utf-8')
        
        playback_dir_pos = content.find("set(AUDIO_PLAYBACK_DIR")
        configs_pos = content.find("set(AUDIO_FILE_CONFIGS", playback_dir_pos)
        if playback_dir_pos == -1 or configs_pos == -1:
            print("⚠ CMakeLists.txt 中未找到 AUDIO_FILE_CONFIGS 配置区域")
            return False

        section_label = "# 音频文件嵌入配置"
        label_pos = content.find(section_label)
        if label_pos != -1 and label_pos < playback_dir_pos:
            start_pos = content.rfind("# ============================================================================", 0, label_pos)
        else:
            start_pos = content.rfind("# ============================================================================", 0, playback_dir_pos)
        if start_pos == -1:
            start_pos = playback_dir_pos

        end_match = re.search(r'^\)\s*$', content[configs_pos:], re.MULTILINE)
        if not end_match:
            print("⚠ CMakeLists.txt 中未找到 AUDIO_FILE_CONFIGS 结束位置")
            return False
        end_pos = configs_pos + end_match.end()
        
        # 生成新的配置
        cmake_config = []
        cmake_config.append("# ============================================================================")
        cmake_config.append("# 音频文件嵌入配置 - 统一命名规范")
        cmake_config.append("# ============================================================================")
        cmake_config.append("# 配置格式：ID:文件名:默认开关:采样率:声道:位深:描述")
        cmake_config.append("# ID规则：自动从文件名生成，统一使用 AUDIO_ 前缀")
        cmake_config.append("# 添加新文件：只需在下面列表中添加一行即可")
        cmake_config.append("# ============================================================================")
        cmake_config.append('set(AUDIO_PLAYBACK_DIR "otool_audio/audio_playback_material")')
        cmake_config.append("")
        cmake_config.append("set(AUDIO_FILE_CONFIGS")
        cmake_config.append("    # ID:文件名:默认值:采样率:声道:位深:描述")
        
        for info in audio_files:
            audio_id = info.macro_name[4:]  # 去掉 "USE_" 前缀
            if audio_id.startswith("AUDIO_"):
                config_id = audio_id
            else:
                config_id = f"AUDIO_{audio_id}"

            default_val = "ON" if info.enabled else "OFF"
            desc = f"{info.base_name} {info.channels}ch {info.sample_rate}Hz {info.duration_sec}s"

            config_line = (
                f'    "{config_id}:{info.filename}:{default_val}:'
                f'{info.sample_rate}:{info.channels}:{info.bits}:{desc}"'
            )
            cmake_config.append(config_line)
        
        cmake_config.append(")")
        cmake_config.append("")
        
        new_section = "\n".join(cmake_config)
        new_content = content[:start_pos] + new_section + "\n" + content[end_pos:].lstrip("\r\n")
        
        self.cmake_file.write_text(new_content, encoding='utf-8')
        
        enabled_count = sum(1 for f in audio_files if f.enabled)
        print(f"✓ 已更新 CMakeLists.txt: {len(audio_files)} 个文件配置 ({enabled_count} 个已启用)")
        return True
    
    def run(self, dry_run: bool = False, interactive: bool = True, skip_validation: bool = False):
        """
        执行完整更新流程（全量更新模式）
        
        注意：此工具采用全量更新策略，会删除已不存在文件的配置
        
        Args:
            dry_run: 预览模式，不修改文件
            interactive: 是否显示交互式菜单
            skip_validation: 是否跳过文件大小校验
        """
        print("=" * 70)
        print("音频文件自动更新工具（全量更新模式）")
        print("=" * 70)
        print()
        print(f"✓ 工作目录: {self.project_root}")
        print(f"✓ 音频目录: {self.audio_dir}")
        print()
        print("⚠ 注意: 此工具会完全重新生成配置，删除已不存在文件的定义")
        print()
        
        # 1. 扫描音频文件并校验
        print("步骤 1/6: 扫描并校验音频文件...")
        print()
        
        try:
            audio_files = self.scan_audio_files(show_details=True, validate_files=not skip_validation)
        except ValueError as e:
            print(f"\n{e}")
            print("\n✗ 扫描失败：请修正文件后重试，或使用 --skip-validation 跳过校验")
            return False
        
        if not audio_files:
            print("✗ 未找到任何有效的 PCM 文件")
            return False
        
        enabled_count = sum(1 for f in audio_files if f.enabled)
        total_size = sum(f.file_size for f in audio_files if f.enabled)
        print()
        print(f"✓ 共找到 {len(audio_files)} 个音频文件（全部通过校验）")
        print(f"✓ 当前配置: {enabled_count} 个已启用, 总计 {total_size / 1024:.1f} KB ({total_size / 1024 / 1024:.2f} MB)")
        print()
        
        # 2. 交互式选择（如果启用）
        if interactive and not dry_run:
            print("步骤 2/6: 交互式文件选择...")
            selected_files = self.show_selection_menu(audio_files)
            if selected_files is None:
                print("\n操作已取消")
                return False
            audio_files = selected_files
            print()
        else:
            print("步骤 2/6: 跳过交互式选择（使用现有配置）")
            print()
        
        if dry_run:
            print("【预览模式】不会修改任何文件\n")
            print("预览结果:")
            for info in audio_files:
                status = "ON " if info.enabled else "OFF"
                print(f"  [{status}] {info.filename}")
            return True
        
        # 3. 备份现有文件
        print("步骤 3/6: 备份现有文件...")
        self.backup_files()
        print()
        
        # 4. 更新头文件
        print("步骤 4/6: 更新头文件枚举...")
        if not self.update_header_enum(audio_files):
            print("✗ 更新头文件失败")
            return False
        print()
        
        # 5. 更新 CPP 文件
        print("步骤 5/6: 更新 CPP 文件...")
        if not self.update_cpp_file(audio_files):
            print("✗ 更新 CPP 文件失败")
            return False
        print()
        
        # 6. 更新 CMakeLists.txt
        print("步骤 6/6: 更新 CMakeLists.txt...")
        if not self.update_cmake_file(audio_files):
            print("✗ 更新 CMakeLists.txt 失败")
            return False
        print()
        
        print("=" * 70)
        print("✓ 所有更新完成！（全量更新）")
        print(f"✓ 备份位置: {self.backup_dir}")
        print()
        
        # 显示最终统计
        enabled_files = [f for f in audio_files if f.enabled]
        if enabled_files:
            total_size = sum(f.file_size for f in enabled_files)
            print("已启用的音频文件:")
            for f in enabled_files:
                print(f"  [X] {f.filename} ({f.file_size / 1024:.1f} KB)")
            print(f"\n总计: {len(enabled_files)} 个文件, {total_size / 1024:.1f} KB ({total_size / 1024 / 1024:.2f} MB)")
        else:
            print("⚠ 警告: 没有启用任何音频文件")
        
        print("\n下一步:")
        print("  1. 检查生成的代码是否正确")
        print("  2. 重新编译项目: idf.py build")
        print(f"  3. 如有问题，可从备份恢复: {self.backup_dir}")
        print("=" * 70)
        return True


def main():
    parser = argparse.ArgumentParser(
        description="音频文件自动更新工具 - 交互式配置和代码生成",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  %(prog)s                        # 交互式模式：扫描、选择、更新（自动检测目录）
  %(prog)s --no-interactive       # 非交互模式：使用现有配置自动更新
  %(prog)s --dry-run              # 预览模式：仅扫描，不修改文件
  %(prog)s --project-root <DIR>   # 手动指定项目根目录（通常不需要）

注意:
  - 脚本会自动检测为脚本所在目录，无需手动指定
  - 可以从任何目录执行此脚本

工作流程:
  1. 扫描 audio_playback_material 目录下的所有 PCM 文件
  2. 读取 CMakeLists.txt 获取当前启用状态
  3. 显示交互式菜单（类似 menuconfig）供用户选择要启用的文件
     - 使用上/下箭头键移动光标
     - 使用空格键切换选中/不选中
     - 按回车键完成选择
     - 按 ESC 键取消操作
  4. 自动更新头文件、CPP 文件和 CMakeLists.txt（全量更新）
  5. 生成备份并显示更新摘要
        """
    )
    
    parser.add_argument(
        '--project-root',
        type=str,
        default=None,
        help='项目根目录路径（可选，默认自动检测为脚本所在目录）'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='预览模式：仅扫描文件和显示当前配置，不执行更新'
    )
    
    parser.add_argument(
        '--no-interactive',
        action='store_true',
        help='非交互模式：使用现有配置自动更新，不显示选择菜单'
    )
    
    parser.add_argument(
        '--skip-validation',
        action='store_true',
        help='跳过PCM文件大小校验（当文件命名与内容可能不完全匹配时使用）'
    )
    
    args = parser.parse_args()
    
    # 解析项目路径（如果提供了自定义路径）
    if args.project_root:
        project_root = Path(args.project_root).resolve()
        if not project_root.exists():
            print(f"✗ 错误: 项目根目录不存在: {project_root}")
            return 1
    else:
        project_root = None  # 让 CodeUpdater 自动检测
    
    # 执行更新
    updater = CodeUpdater(project_root)
    
    try:
        interactive = not args.no_interactive
        success = updater.run(
            dry_run=args.dry_run, 
            interactive=interactive,
            skip_validation=args.skip_validation
        )
        return 0 if success else 1
    except Exception as e:
        print(f"\n✗ 执行过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())

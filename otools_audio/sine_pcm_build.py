# SPDX-FileCopyrightText: 2025 exia
# SPDX-License-Identifier: MIT

import numpy as np
import struct
import argparse

# 常用频率预设
COMMON_FREQUENCIES = {
    # 测试频率
    'low_20': 20,      'low_50': 50,      'low_100': 100,    'low_200': 200,
    'mid_440': 440,    'mid_500': 500,    'mid_800': 800,    'mid_1k': 1000,
    'high_2k': 2000,   'high_5k': 5000,   'high_8k': 8000,   'high_10k': 10000,
    
    # 音符频率 (第4和第5八度)
    'C4': 261.63,  'D4': 293.66,  'E4': 329.63,  'F4': 349.23,
    'G4': 392.00,  'A4': 440.00,  'B4': 493.88,  'C5': 523.25,
    'D5': 587.33,  'E5': 659.25,  'F5': 698.46,  'G5': 783.99,
    'A5': 880.00,  'B5': 987.77,  'C6': 1046.50,
    
# 特殊测试频率
    'test_60hz': 60,     # 电源频率干扰测试
    'test_120hz': 120,   # 电源谐波
    'test_1khz': 1000,   # 标准1kHz测试音
    'nyquist_22k': 22050, # 奈奎斯特频率(44.1kHz的一半)
}

def show_frequency_presets():
    """显示常用频率预设"""
    print("\n常用频率预设:")
    print("=" * 50)
    
    print("测试频率:")
    test_freqs = ['low_20', 'low_50', 'low_100', 'low_200', 'mid_440', 'mid_500', 'mid_800', 'mid_1k']
    for freq_key in test_freqs:
        print(f"  {freq_key}: {COMMON_FREQUENCIES[freq_key]}Hz")
    
    print("\n高频测试:")
    high_freqs = ['high_2k', 'high_5k', 'high_8k', 'high_10k']
    for freq_key in high_freqs:
        print(f"  {freq_key}: {COMMON_FREQUENCIES[freq_key]}Hz")
    
    print("\n音符频率:")
    note_freqs = ['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']
    for freq_key in note_freqs:
        print(f"  {freq_key}: {COMMON_FREQUENCIES[freq_key]}Hz")
    
    print("\n特殊测试:")
    special_freqs = ['test_60hz', 'test_120hz', 'test_1khz', 'nyquist_22k']
    for freq_key in special_freqs:
        print(f"  {freq_key}: {COMMON_FREQUENCIES[freq_key]}Hz")
    
    print("\n使用方法: 输入预设名称(如 'A4') 或直接输入数字频率")

def parse_frequency_input(freq_input):
    """解析频率输入，支持预设名称或数字"""
    if isinstance(freq_input, (int, float)):
        return float(freq_input)
    
    freq_str = str(freq_input).strip()
    
    # 检查是否是预设名称
    if freq_str in COMMON_FREQUENCIES:
        return COMMON_FREQUENCIES[freq_str]
    
    # 尝试解析为数字
    try:
        return float(freq_str)
    except ValueError:
        raise ValueError(f"无效的频率输入: {freq_str}. 请使用数字或预设名称 (如 'A4', 'mid_1k')")

def generate_sine_wave_pcm(bits_per_sample=16, channel=1, sample_rate=44100, duration=30, frequency=440, output_filename=None):
    """
    生成单通道正弦波PCM文件
    参数:
    - bits_per_sample: 采样位深 (默认: 16)
    - channel: 声道数 (默认: 1, 单通道)
    - sample_rate: 采样率 Hz (默认: 44100)
    - duration: 时长秒数 (默认: 30)
    - frequency: 正弦波频率 Hz (默认: 440)
      常用频率参考:
      * 低频测试: 20Hz, 50Hz, 100Hz, 200Hz
      * 中频测试: 440Hz(A4), 500Hz, 800Hz, 1000Hz(1kHz)
      * 高频测试: 2000Hz(2kHz), 5000Hz(5kHz), 8000Hz(8kHz), 10000Hz(10kHz)
      * 音符频率: 261.63Hz(C4), 293.66Hz(D4), 329.63Hz(E4), 349.23Hz(F4), 
                  392.00Hz(G4), 440.00Hz(A4), 493.88Hz(B4), 523.25Hz(C5)
    - output_filename: 输出文件名 (默认: 自动生成)
    """
    
    # 验证参数
    if bits_per_sample not in [8, 16, 24, 32]:
        raise ValueError("bits_per_sample 必须是 8, 16, 24, 或 32")
    if channel < 1:
        raise ValueError("channel 必须大于 0")
    if sample_rate <= 0:
        raise ValueError("sample_rate 必须大于 0")
    if duration <= 0:
        raise ValueError("duration 必须大于 0")
    if frequency <= 0:
        raise ValueError("frequency 必须大于 0")
    
    # 自动生成文件名
    if output_filename is None:
        # 确保所有数值格式化为合适的字符串
        if frequency == int(frequency):
            freq_str = str(int(frequency))
        else:
            freq_str = f"{frequency:.2f}".rstrip('0').rstrip('.')
        
        if duration == int(duration):
            dur_str = str(int(duration))
        else:
            dur_str = f"{duration:.1f}".rstrip('0').rstrip('.')
            
        output_filename = f"sine_{freq_str}Hz_{dur_str}s_{sample_rate}Hz_{bits_per_sample}bit_{channel}ch.pcm"
    
    # 计算总采样数
    total_samples = int(sample_rate * duration * channel)
    
    # 生成时间轴
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    
    # 生成正弦波 (归一化到 -1 到 1)
    sine_wave = np.sin(2 * np.pi * frequency * t)
    
    # 如果是多声道，复制到所有声道
    if channel > 1:
        sine_wave = np.tile(sine_wave.reshape(-1, 1), (1, channel)).flatten()
    
    # 根据位深转换数据类型
    if bits_per_sample == 8:
        # 8位无符号整数 (0 到 255)
        max_amplitude = 127
        sine_wave_int = ((sine_wave * max_amplitude) + 128).astype(np.uint8)
        pack_format = 'B'
    elif bits_per_sample == 16:
        # 16位有符号整数 (-32768 到 32767)
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int16)
        pack_format = '<h'  # 小端序
    elif bits_per_sample == 24:
        # 24位有符号整数
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int32)
        pack_format = '<i'  # 注意：需要特殊处理24位
    elif bits_per_sample == 32:
        # 32位有符号整数
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int32)
        pack_format = '<i'  # 小端序
    
    # 写入PCM文件
    with open(output_filename, 'wb') as f:
        for sample in sine_wave_int:
            if bits_per_sample == 24:
                # 24位需要特殊处理，只写入3个字节
                sample_bytes = struct.pack('<i', int(sample))[:3]
                f.write(sample_bytes)
            else:
                f.write(struct.pack(pack_format, int(sample)))
    
    # 计算文件大小
    bytes_per_sample = bits_per_sample // 8
    file_size = len(sine_wave_int) * bytes_per_sample
    
    print(f"生成完成！")
    print(f"文件名: {output_filename}")
    print(f"参数:")
    print(f"  - 采样率: {sample_rate} Hz")
    print(f"  - 位深: {bits_per_sample} bits")
    print(f"  - 声道: {channel} ({'单声道' if channel == 1 else f'{channel}声道'})")
    print(f"  - 时长: {duration} 秒")
    print(f"  - 频率: {frequency} Hz")
    print(f"  - 文件大小: {file_size} 字节 ({file_size/1024:.1f} KB)")
    
    return output_filename

def generate_variable_frequency_sine(bits_per_sample=16, channel=1, sample_rate=44100, duration=30, 
                                    freq_start=200, freq_end=800, output_filename=None):
    """
    生成可变频率的正弦波 (可选功能)
    """
    if output_filename is None:
        # 格式化起始和结束频率
        start_str = str(int(freq_start)) if freq_start == int(freq_start) else f"{freq_start:.2f}".rstrip('0').rstrip('.')
        end_str = str(int(freq_end)) if freq_end == int(freq_end) else f"{freq_end:.2f}".rstrip('0').rstrip('.')
        dur_str = str(int(duration)) if duration == int(duration) else f"{duration:.1f}".rstrip('0').rstrip('.')
        output_filename = f"sine_sweep_{start_str}-{end_str}Hz_{dur_str}s_{sample_rate}Hz_{bits_per_sample}bit_{channel}ch.pcm"
    
    total_samples = int(sample_rate * duration)
    t = np.linspace(0, duration, total_samples, endpoint=False)
    
    # 频率线性变化
    frequency = np.linspace(freq_start, freq_end, total_samples)
    
    # 生成变频正弦波
    phase = 2 * np.pi * np.cumsum(frequency) / sample_rate
    sine_wave = np.sin(phase)
    
    # 如果是多声道，复制到所有声道
    if channel > 1:
        sine_wave = np.tile(sine_wave.reshape(-1, 1), (1, channel)).flatten()
    
    # 根据位深转换数据类型
    if bits_per_sample == 8:
        max_amplitude = 127
        sine_wave_int = ((sine_wave * max_amplitude) + 128).astype(np.uint8)
        pack_format = 'B'
    elif bits_per_sample == 16:
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int16)
        pack_format = '<h'
    elif bits_per_sample == 24:
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int32)
        pack_format = '<i'
    elif bits_per_sample == 32:
        max_amplitude = (2 ** (bits_per_sample - 1)) - 1
        sine_wave_int = (sine_wave * max_amplitude).astype(np.int32)
        pack_format = '<i'
    
    # 写入PCM文件
    with open(output_filename, 'wb') as f:
        for sample in sine_wave_int:
            if bits_per_sample == 24:
                sample_bytes = struct.pack('<i', int(sample))[:3]
                f.write(sample_bytes)
            else:
                f.write(struct.pack(pack_format, int(sample)))
    
    print(f"变频正弦波生成完成: {output_filename}")
    print(f"  频率范围: {freq_start}Hz - {freq_end}Hz")
    return output_filename

def main():
    """主函数，支持命令行参数"""
    parser = argparse.ArgumentParser(description='正弦波PCM文件生成器')
    parser.add_argument('-b', '--bits', type=int, default=16, choices=[8, 16, 24, 32],
                        help='采样位深 (默认: 16)')
    parser.add_argument('-c', '--channels', type=int, default=1,
                        help='声道数 (默认: 1)')
    parser.add_argument('-r', '--rate', type=int, default=44100,
                        help='采样率 Hz (默认: 44100)')
    parser.add_argument('-d', '--duration', type=float, default=30,
                        help='时长秒数 (默认: 30)')
    parser.add_argument('-f', '--frequency', type=str, default='440',
                        help='正弦波频率 (默认: 440) - 支持数字或预设名称 (如: A4, mid_1k, C4, high_5k)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出文件名 (默认: 自动生成)')
    parser.add_argument('--sweep', action='store_true',
                        help='生成变频正弦波')
    parser.add_argument('--freq-start', type=float, default=200,
                        help='变频起始频率 Hz (默认: 200)')
    parser.add_argument('--freq-end', type=float, default=800,
                        help='变频结束频率 Hz (默认: 800)')
    parser.add_argument('--list-presets', action='store_true',
                        help='显示所有可用的频率预设')
    
    args = parser.parse_args()
    
    # 如果用户要求显示预设，显示后退出
    if args.list_presets:
        show_frequency_presets()
        return
    
    print("正弦波PCM文件生成器")
    print("=" * 40)
    
    try:
        # 解析频率参数
        frequency = parse_frequency_input(args.frequency)
        
        if args.sweep:
            # 生成变频正弦波
            pcm_file = generate_variable_frequency_sine(
                bits_per_sample=args.bits,
                channel=args.channels,
                sample_rate=args.rate,
                duration=args.duration,
                freq_start=args.freq_start,
                freq_end=args.freq_end,
                output_filename=args.output
            )
        else:
            # 生成标准正弦波
            pcm_file = generate_sine_wave_pcm(
                bits_per_sample=args.bits,
                channel=args.channels,
                sample_rate=args.rate,
                duration=args.duration,
                frequency=frequency,
                output_filename=args.output
            )
        
        print(f"\n生成的文件: {pcm_file}")
        
    except ValueError as e:
        print(f"参数错误: {e}")
    except Exception as e:
        import traceback
        print(f"生成失败: {e}")
        print("详细错误信息:")
        traceback.print_exc()

if __name__ == "__main__":
    import sys
    
    # 如果没有命令行参数，使用交互模式
    if len(sys.argv) == 1:
        print("正弦波PCM文件生成器 - 交互模式")
        print("=" * 40)
        
        try:
            # 显示频率预设
            show_frequency_presets()
            
            print("\n请输入参数:")
            bits = int(input("采样位深 [8/16/24/32] (默认: 16): ") or "16")
            channels = int(input("声道数 (默认: 1): ") or "1")
            rate = int(input("采样率 Hz (默认: 44100): ") or "44100")
            duration = float(input("时长秒数 (默认: 30): ") or "30")
            
            freq_input = input("频率 (默认: 440, 可用预设如 'A4', 'mid_1k'): ") or "440"
            frequency = parse_frequency_input(freq_input)
            
            output = input("输出文件名 (默认: 自动生成): ") or None
            
            # 生成标准正弦波
            pcm_file = generate_sine_wave_pcm(
                bits_per_sample=bits,
                channel=channels,
                sample_rate=rate,
                duration=duration,
                frequency=frequency,
                output_filename=output
            )
            
            # 询问是否生成变频正弦波
            print("\n是否生成变频正弦波? (y/n): ", end="")
            choice = input().lower()
            if choice in ['y', 'yes']:
                freq_start = float(input("起始频率 Hz (默认: 200): ") or "200")
                freq_end = float(input("结束频率 Hz (默认: 800): ") or "800")
                generate_variable_frequency_sine(
                    bits_per_sample=bits,
                    channel=channels,
                    sample_rate=rate,
                    duration=duration,
                    freq_start=freq_start,
                    freq_end=freq_end
                )
            
            print("\n完成！")
            
        except ValueError as e:
            print(f"输入错误: {e}")
        except KeyboardInterrupt:
            print("\n操作取消")
        except Exception as e:
            print(f"生成失败: {e}")
    else:
        # 使用命令行模式
        main()
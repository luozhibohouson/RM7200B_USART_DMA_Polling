#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import struct # Import struct module
from intelhex import IntelHex

TARGET_INFO_ADDR = 0x08007C00
APP_VALID_FLAG = 0xAA55
APP_UPGRADE_INFO_FLAG = 0xCD12 # Using a more descriptive name for 0xCD12

def calculate_app_info(app_ih):
    """计算app.hex的bin大小和16位校验和 (优化版)"""
    all_app_bytes_list = [] # Store lists of bytes from segments
    app_size_bytes = 0
    app_checksum = 0

    if app_ih.segments():
        for start, end in sorted(app_ih.segments()): # Iterate through segments in address order
            # tobinstr(start, end) 'end' is the last address to be read (inclusive)
            # So, we use end - 1 as segment's end is exclusive for tobinstr's end param definition.
            # However, IntelHex's tobinstr actually expects 'end' to be the address of the last byte.
            # A segment is defined by [start_address, end_address)
            # So, the last valid address in the segment is end_address - 1.
            if start < end: # Ensure the segment has a valid range
                segment_bytes = app_ih.tobinstr(start=start, end=end-1)
                all_app_bytes_list.append(segment_bytes) # Add the bytes object itself

    # Concatenate all byte strings and calculate sum and length
    final_app_data = b''.join(all_app_bytes_list)
    app_size_bytes = len(final_app_data)

    if app_size_bytes > 0:
        # Calculate checksum from the final concatenated bytes
        # In Python 3, bytes are sequences of integers, so sum() works directly.
        current_sum = sum(final_app_data)
        app_checksum = current_sum & 0xFFFF # 16-bit checksum

    return app_size_bytes, app_checksum

def merge_hex_files(bootloader_hex_path, app_hex_path, output_hex_path):
    """合并bootloader和app的hex文件,并添加app信息"""
    try:
        # 读取bootloader hex文件
        bootloader_ih = IntelHex(bootloader_hex_path)

        # 读取app hex文件
        app_ih = IntelHex(app_hex_path)

        # 1. 计算app.hex的bin大小和校验和 (从原始app_ih计算)
        print("Calculating app info...")
        app_size, app_checksum = calculate_app_info(app_ih)
        print(f"App Info: Size = {app_size} bytes, Checksum = 0x{app_checksum:04X}")

        # 2. 合并hex文件 (bootloader_ih会被修改)
        print("Merging HEX files...")
        bootloader_ih.merge(app_ih, overlap='replace')

        # 3. 在合并后的hex (bootloader_ih) 中写入app信息
        print(f"Writing app info to address 0x{TARGET_INFO_ADDR:08X}:")

        # Helper function to pack value and call puts
        def write_word_le(address, value):
            packed_value = struct.pack('<H', value) # <H for little-endian unsigned short (16-bit)
            bootloader_ih.puts(address, packed_value)

        print(f"  0x{TARGET_INFO_ADDR:08X} (app_valid)  : 0x{APP_VALID_FLAG:04X}")
        write_word_le(TARGET_INFO_ADDR + 0, APP_VALID_FLAG)

        print(f"  0x{TARGET_INFO_ADDR+2:08X} (app_success): 0x{APP_UPGRADE_INFO_FLAG:04X}")
        write_word_le(TARGET_INFO_ADDR + 2, APP_UPGRADE_INFO_FLAG)

        print(f"  0x{TARGET_INFO_ADDR+4:08X} (app_size)   : {app_size} (0x{app_size:04X})")
        write_word_le(TARGET_INFO_ADDR + 4, app_size & 0xFFFF)

        print(f"  0x{TARGET_INFO_ADDR+6:08X} (app_checksum): 0x{app_checksum:04X}")
        write_word_le(TARGET_INFO_ADDR + 6, app_checksum)

        # 4. 保存合并并修改后的文件
        print(f"Writing output file: {output_hex_path}")
        bootloader_ih.write_hex_file(output_hex_path)
        print(f"成功生成合并并已更新信息的文件: {output_hex_path}")
        return True
    except Exception as e:
        print(f"处理hex文件时出错: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='合并bootloader和app的hex文件，并添加app元数据。所有路径参数都应为绝对路径。')
    parser.add_argument('bootloader_hex_path', help='Bootloader HEX 文件的绝对路径')
    parser.add_argument('app_hex_path', help='Application HEX 文件的绝对路径')
    parser.add_argument('output_hex_path', help='合并后输出的 HEX 文件的绝对路径')
    args = parser.parse_args()

    # 检查bootloader hex文件是否存在
    if not os.path.exists(args.bootloader_hex_path):
        print(f"错误: Bootloader HEX 文件不存在: {args.bootloader_hex_path}")
        return 1

    # 检查app hex文件是否存在
    if not os.path.exists(args.app_hex_path):
        print(f"错误: Application HEX 文件不存在: {args.app_hex_path}")
        return 1

    # 合并hex文件
    if merge_hex_files(args.bootloader_hex_path, args.app_hex_path, args.output_hex_path):
        return 0
    return 1

if __name__ == '__main__':
    sys.exit(main())
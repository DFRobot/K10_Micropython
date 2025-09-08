#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
创建测试用的 bin 文件
"""

import os

def create_test_bin():
    """创建一个测试用的 bin 文件"""
    
    # 确保目录存在
    os.makedirs('frozen_bin', exist_ok=True)
    
    # 创建测试数据
    test_data = b'This is a test binary file for MicroPython filesystem.\n'
    test_data += b'You can replace this with your actual bin file.\n'
    test_data += b'File created for testing freeze functionality.\n'
    
    # 写入文件
    with open('frozen_bin/test.bin', 'wb') as f:
        f.write(test_data)
    
    print("测试 bin 文件已创建: frozen_bin/test.bin")
    print(f"文件大小: {len(test_data)} 字节")

if __name__ == "__main__":
    create_test_bin() 
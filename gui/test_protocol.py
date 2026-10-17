# -*- coding: utf-8 -*-
"""
LHA6958H 极简通信协议单元测试脚本
验证:
1. 16-bit 补码与电压值换算正确性 (正满量程, 负满量程, 零点)
2. 完整 16,392 字节大包构造与 FrameStreamParser 滑窗拼包自愈性
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import numpy as np
from gui.protocol import (
    PACK_GROUPS,
    NUM_CHANNELS,
    PAYLOAD_LEN,
    FRAME_LEN,
    parse_payload_to_matrix,
    build_frame,
    FrameStreamParser,
    DEFAULT_VREF,
)


def test_codec():
    print("=== 测试 1: 16-bit 补码与实际测量电压转换 ===")
    # 构造测试载荷: 1024 组, 每组 8 通道
    # CH1 = 0x7FFF (+32767 -> ~+10.0V)
    # CH2 = 0x8000 (-32768 -> -10.0V)
    # CH3 = 0x0000 (0 -> 0.0V)
    # CH4 = 0x4000 (+16384 -> +5.0V)
    # CH5 = 0xC000 (-16384 -> -5.0V)
    # CH6 = 0x0001 (+1)
    # CH7 = 0xFFFF (-1)
    # CH8 = 0x1000
    test_channels_hex = [
        bytes([0x7F, 0xFF]), # CH1
        bytes([0x80, 0x00]), # CH2
        bytes([0x00, 0x00]), # CH3
        bytes([0x40, 0x00]), # CH4
        bytes([0xC0, 0x00]), # CH5
        bytes([0x00, 0x01]), # CH6
        bytes([0xFF, 0xFF]), # CH7
        bytes([0x10, 0x00]), # CH8
    ]
    single_group = b"".join(test_channels_hex)
    assert len(single_group) == 16, f"每组长度应为16字节, 实际={len(single_group)}"

    full_payload = single_group * PACK_GROUPS
    assert len(full_payload) == PAYLOAD_LEN, f"载荷长度应为{PAYLOAD_LEN}, 实际={len(full_payload)}"

    codes, volts = parse_payload_to_matrix(full_payload, vref=10.0)
    assert codes.shape == (1024, 8)
    assert volts.shape == (1024, 8)

    # 验证第 0 组解析结果
    assert codes[0, 0] == 32767
    assert abs(volts[0, 0] - (32767 / 32768.0 * 10.0)) < 1e-4

    assert codes[0, 1] == -32768
    assert abs(volts[0, 1] - (-10.0)) < 1e-4

    assert codes[0, 2] == 0
    assert abs(volts[0, 2] - 0.0) < 1e-4

    assert codes[0, 3] == 16384
    assert abs(volts[0, 3] - 5.0) < 1e-4

    assert codes[0, 4] == -16384
    assert abs(volts[0, 4] - (-5.0)) < 1e-4

    assert codes[0, 5] == 1
    assert codes[0, 6] == -1
    print("  [PASS] 补码与电压值换算测试通过!")


def test_parser_stream():
    print("=== 测试 2: 流式解析与滑窗防错测试 ===")
    parser = FrameStreamParser()

    # 构造正常帧
    single_group = bytes([0x12, 0x34] * 8)
    payload = single_group * PACK_GROUPS
    frame0 = build_frame(pkt_id=0, payload_bytes=payload, status=0x01)
    frame1 = build_frame(pkt_id=1, payload_bytes=payload, status=0x01)
    assert len(frame0) == FRAME_LEN

    # 1. 模拟帧头前混入杂乱字节
    garbage = b"\x00\xFF\xAA\x55GarbageData"
    res1 = parser.push_data(garbage + frame0[:1000])
    assert len(res1) == 0, "未拼满前不应输出"

    # 2. 灌入剩余部分及第二包前半部分
    res2 = parser.push_data(frame0[1000:] + frame1[:500])
    assert len(res2) == 1, "第 0 包应成功解析"
    assert res2[0]["pkt_id"] == 0
    assert res2[0]["status"] == 0x01

    # 3. 灌入第二包后半部分
    res3 = parser.push_data(frame1[500:])
    assert len(res3) == 1, "第 1 包应成功解析"
    assert res3[0]["pkt_id"] == 1
    print("  [PASS] 流式拼帧与滑窗防错测试通过!")


if __name__ == "__main__":
    test_codec()
    test_parser_stream()
    print("\n所有测试全部通过！LHA6958H 串口通信协议完美运行。")

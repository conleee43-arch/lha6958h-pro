# -*- coding: utf-8 -*-
"""
LHA6958H 采集系统极简通信协议定义与数据解析器
帧结构 (共 16,392 字节):
  - 帧头 (4B): 0x5A 0xA5 0x55 0xAA
  - 序号 (1B): 0x00 ~ 0xFF 循环自增
  - 标志 (1B): status 状态字 (D0: init_done, D1: err_align, D2: err_busy_timeout, D3: err_overrun)
  - 载荷 (16384B): 1024 组 x 8 通道 x 2 字节 (16-bit 补码, MSB first)
  - 帧尾 (2B): 0x0D 0x0A (\r\n)
有效载荷率: 99.951%
"""

import numpy as np
from typing import Tuple, Optional, Dict, Any, List

# 协议常量
FRAME_HEADER = bytes([0x5A, 0xA5, 0x55, 0xAA])
FRAME_TAIL = bytes([0x0D, 0x0A])
PACK_GROUPS = 1024
NUM_CHANNELS = 8
BYTES_PER_SAMPLE = 2  # 16 位 = 2 字节
GROUP_BYTES = NUM_CHANNELS * BYTES_PER_SAMPLE  # 16 字节
PAYLOAD_LEN = PACK_GROUPS * GROUP_BYTES        # 16,384 字节

HEADER_EXTRA_LEN = 2  # 1字节序号 + 1字节标志
HEADER_TOTAL_LEN = len(FRAME_HEADER) + HEADER_EXTRA_LEN  # 6 字节
FRAME_LEN = HEADER_TOTAL_LEN + PAYLOAD_LEN + len(FRAME_TAIL)  # 16,392 字节

DEFAULT_VREF = 10.0  # 默认量程 (+/- 10V 满量程)
DEFAULT_SAMPLE_RATE = 500_000  # 默认采样率 500 kSPS


def parse_payload_to_matrix(
    payload_bytes: bytes, vref: float = DEFAULT_VREF
) -> Tuple[np.ndarray, np.ndarray]:
    """
    高效解析 16,384 字节载荷为 (1024, 8) 维度的整数码值与电压矩阵
    
    参数:
        payload_bytes: 长度必须严格为 16,384 的字节数据
        vref: 参考量程电压，用于将 16-bit 补码换算为实际测量电压 (V)
        
    返回:
        (raw_codes, voltages):
            raw_codes: shape (1024, 8), int16 有符号整数，范围 [-32768, 32767]
            voltages:  shape (1024, 8), float64 测量电压值 (单位: V)
    """
    if len(payload_bytes) != PAYLOAD_LEN:
        raise ValueError(f"载荷长度必须为 {PAYLOAD_LEN} 字节，实际为 {len(payload_bytes)}")

    # 展开为 (1024, 8, 2) 的 uint8 数组: [组号, 通道号, 字节0~1]
    u8_arr = np.frombuffer(payload_bytes, dtype=np.uint8).reshape(
        (PACK_GROUPS, NUM_CHANNELS, BYTES_PER_SAMPLE)
    )

    # 向量化拼接 16-bit (高字节在先: byte0=MSB, byte1=LSB)
    b0 = u8_arr[:, :, 0].astype(np.int32)
    b1 = u8_arr[:, :, 1].astype(np.int32)

    raw_codes = (b0 << 8) | b1

    # 符号位扩展: 若最高位为 1 (b0 >= 128)，则减去 2^16
    sign_mask = b0 >= 128
    raw_codes[sign_mask] -= 65536

    # 换算为实际电压值 (16位二进制补码满量程为 2^15 = 32,768)
    voltages = (raw_codes / 32768.0) * vref

    return raw_codes, voltages


def build_frame(pkt_id: int, payload_bytes: bytes, status: int = 0) -> bytes:
    """
    构造一个符合极简通信协议的完整大包数据 (用于测试与仿真)
    """
    if len(payload_bytes) != PAYLOAD_LEN:
        raise ValueError(f"载荷长度必须为 {PAYLOAD_LEN} 字节")
    
    header = FRAME_HEADER + bytes([pkt_id & 0xFF, status & 0xFF])
    return header + payload_bytes + FRAME_TAIL


class FrameStreamParser:
    """
    流式数据帧解析器 (带滑窗缓冲区防丢包与自愈机制)
    用于在持续到来的无序字节流中，精准定位帧头并提取完整大包
    """

    def __init__(self):
        self._buffer = bytearray()
        self.total_frames_received = 0
        self.lost_frames_count = 0
        self.last_pkt_id: Optional[int] = None

    def push_data(self, chunk: bytes) -> List[Dict[str, Any]]:
        """
        向解析器灌入新接收的原始字节块，返回已完整解析的所有有效帧列表
        """
        self._buffer.extend(chunk)
        frames = []

        while True:
            # 1. 缓冲区数据不足以容纳一帧，等待更多数据
            if len(self._buffer) < FRAME_LEN:
                break

            # 2. 定位帧头
            header_idx = self._buffer.find(FRAME_HEADER)
            if header_idx == -1:
                # 缓冲区内完全没有帧头，保留最后 (len(FRAME_HEADER) - 1) 个字节以防帧头跨包截断
                keep_len = len(FRAME_HEADER) - 1
                if len(self._buffer) > keep_len:
                    self._buffer = self._buffer[-keep_len:]
                break

            # 若帧头前有杂乱垃圾数据，直接丢弃
            if header_idx > 0:
                del self._buffer[:header_idx]

            # 再次确认对齐帧头后是否有足够长度
            if len(self._buffer) < FRAME_LEN:
                break

            # 3. 校验帧尾
            tail_start = HEADER_TOTAL_LEN + PAYLOAD_LEN
            tail = self._buffer[tail_start : tail_start + len(FRAME_TAIL)]
            if tail != FRAME_TAIL:
                # 帧尾不匹配，说明遇到了伪帧头，跳过当前伪帧头的前导字节继续搜寻
                del self._buffer[: len(FRAME_HEADER)]
                continue

            # 4. 提取有效数据
            pkt_id = self._buffer[len(FRAME_HEADER)]
            status = self._buffer[len(FRAME_HEADER) + 1]
            payload = bytes(self._buffer[HEADER_TOTAL_LEN : tail_start])

            # 检测是否有丢包
            if self.last_pkt_id is not None:
                expected_id = (self.last_pkt_id + 1) & 0xFF
                if pkt_id != expected_id:
                    diff = (pkt_id - expected_id) & 0xFF
                    self.lost_frames_count += diff
            self.last_pkt_id = pkt_id
            self.total_frames_received += 1

            frames.append({
                "pkt_id": pkt_id,
                "status": status,
                "payload": payload,
            })

            # 从缓冲区中移除这已经解析完的一整包
            del self._buffer[:FRAME_LEN]

        return frames

    def reset(self):
        """重置解析器状态"""
        self._buffer.clear()
        self.total_frames_received = 0
        self.lost_frames_count = 0
        self.last_pkt_id = None

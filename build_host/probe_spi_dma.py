#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dump dr_rx_buf 原始内容 + SPI1/DMA 寄存器, 判断 DMA 是否实际写入"""
import sys
import time

sys.path.insert(0, r"D:\STM32Project\Flight\build_host")
from check_gins_ocd import Ocd, mdw_words

DR_RX = 0x24000F80          # 2 x 22B, 32B 对齐
DR_TX = 0x24000020
SPI1 = 0x40013000           # CR1 CR2 CFG1 CFG2 IER SR IFCR TXDR.. RXDR@0x30
DMA1_S0 = 0x40026010        # CR NDTR PAR M0AR


def hexdump(ocd, addr, n, label):
    ws = mdw_words(ocd, addr, n)
    if not ws:
        print(f"{label}: READ FAIL")
        return
    print(label + ":")
    for i, w in enumerate(ws):
        print(f"  {addr + i*4:#010x}: {w:08x}")


def main():
    ocd = Ocd()

    print("== dr_tx_buf (应含 0x6800 命令) ==")
    hexdump(ocd, DR_TX, 6, "tx")

    print("\n== dr_rx_buf 第一次 ==")
    hexdump(ocd, DR_RX, 12, "rx[0..]")
    r1 = mdw_words(ocd, DR_RX, 12)
    time.sleep(3.0)
    print("== dr_rx_buf 3s 后 ==")
    hexdump(ocd, DR_RX, 12, "rx[0..]")
    r2 = mdw_words(ocd, DR_RX, 12)
    print(f"\nrx 变化: {'YES -> DMA 在写' if r1 != r2 else 'NO -> DMA 没有写入!'}")

    print("\n== SPI1 寄存器 ==")
    ws = mdw_words(ocd, SPI1, 6)
    if ws:
        names = ["CR1", "CR2", "CFG1", "CFG2", "IER", "SR"]
        for n, v in zip(names, ws):
            print(f"  {n:5s}: {v:#010x}")

    print("\n== DMA1_Stream0 (SPI1_RX) ==")
    ws = mdw_words(ocd, DMA1_S0, 7)
    if ws:
        names = ["?", "CR", "NDTR", "PAR", "M0AR", "M1AR", "FCR"]
        for n, v in zip(names, ws):
            print(f"  {n:5s}: {v:#010x}")

    print("\n== DMA1_Stream1 (SPI1_TX) ==")
    ws = mdw_words(ocd, DMA1_S0 + 0x18, 7)
    if ws:
        names = ["?", "CR", "NDTR", "PAR", "M0AR", "M1AR", "FCR"]
        for n, v in zip(names, ws):
            print(f"  {n:5s}: {v:#010x}")


if __name__ == "__main__":
    sys.exit(main())

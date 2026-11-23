# 验证报告：固定点流式 FIR 滤波器

环境：Ubuntu 22.04.5，Python 3.12.11，Icarus Verilog 11.0，
Yosys 0.70+240（oss-cad-suite 2026-10-27）。
最终命令：`python3 scripts/run_all.py`（YOSYS=/tmp/oss/oss-cad-suite/bin/yosys）。

## 1. 最终结果：passed=19 failed=0 skipped=0

| 项目 | 结果 | 验证内容 |
|---|---|---|
| impulse | PASS | 冲激响应逐样本一致（395/395） |
| constant | PASS | 常数输入 DC 增益（395/395） |
| alternating | PASS | ±max 交替最坏符号翻转（395/395） |
| saturation | PASS | +max 饱和路径（395/395） |
| near_sat | PASS | 饱和边界邻域（395/395） |
| random | PASS | 随机输入（395/395） |
| backpressure | PASS | bursty 反压不丢/不重样本（395/395） |
| reset_stream | PASS | 第 60 周期流中复位行为（30/30） |
| decim3 | PASS | 抽取 /3 模式（131/131） |
| decim3_bp | PASS | 抽取 + 反压（101/101） |
| taps7_random | PASS | TAPS=7 不同流水深度（240/240） |
| taps7_impulse | PASS | TAPS=7 冲激（396/396） |
| lpf_impulse | PASS | Hamming LPF 冲激 + $readmemh 系数 ROM（395/395） |
| lpf_random | PASS | Hamming LPF 随机 + 反压 + 饱和边界（240/240） |
| wrap_saturation | PASS | SAT=0 回绕（395/395） |
| wrap_constant | PASS | SAT=0 回绕常数（395/395） |
| perf:random | PASS | 延迟 5 周期（期望 5），吞吐 1/周期 |
| perf:decim3 | PASS | 延迟 7 周期（期望 7），吞吐 1/3 周期 |
| synth | PASS | Yosys 综合成功，===SYNTH_OK=== 哨兵存在 |

自检覆盖（tb_fir.sv）：逐值比对参考模型；`pushed == accepted`（非抽取场景
395=395）判不丢/不重；样本顺序；复位期间 m_valid==0；流中复位不清已收集
输出；`pushes > accepted` 判重复。数值溢出由模型逐样本断言 |acc|<2^36 校验。

## 2. 已实现功能

参数化 TAPS/DW/CW；S1.15 固定点；显式中间位宽（PW=32/ACCW=36/EXT=21）；
可选舍入与饱和；valid/ready 锁定握手；同步复位；packed 参数或 $readmemh
系数初始化；抽取模式（DECIM_EN/DECIM_PERIOD）；独立整数参考模型；自检 TB；
延迟/吞吐 TB；Yosys 综合流程；Makefile（all/test/gen/perf/synth/clean/zip）。

## 3. 已验证功能

全部 16 个功能场景 + 2 个性能测试 + 1 个综合流程均在本机实际执行通过；
参考模型与 RTL 逐位一致（含舍入、饱和、回绕、抽取、反压、复位、系数量化）。

## 4. 已知问题

- 综合为技术映射统计（门/触发器），未跑 ABC 门级优化与布局布线，故无
  Fmax/面积（mm²）等物理数据；运行时间约 5 秒。
- 锁定握手在输出背压时同步暂停输入（设计特性，非缺陷）。
- 系数为编译期常量，无运行时重配端口。

## 5. 未完成工作

- 无 FPGA 板级验证（本环境无硬件/厂商工具链）；后续可在 Vivado/Quartus
  中综合-布局布线-上板，补充时序收敛与实测 Fmax。
- 未做运行时系数重配（AXI-Lite 寄存器接口）与多通道。
- 未做形式化等价性检查（可选用 SymbiYosys 对 RTL 与参考模型做 BMC）。

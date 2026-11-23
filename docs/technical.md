# 技术文档：固定点流式 FIR 滤波器 FPGA 加速器

## 1. 设计目标与范围

可综合的 SystemVerilog DSP IP：参数化、有符号固定点、流式（valid/ready）
FIR 滤波器。附带独立整数算术参考模型、自检 SystemVerilog 测试平台、可选
抽取模式与 Yosys 综合流程。算术、控制、接口逻辑分三个模块实现。

## 2. 固定点格式（默认配置）

| 量 | 位宽 | 格式 | 说明 |
|---|---|---|---|
| 输入样本 x | DW=16 | S1.15 | 1 符号位 + 15 小数位 |
| 系数 c | CW=16 | S1.15 | DC 增益归一化为 1 |
| 乘积 p | PW=DW+CW=32 | S2.30 | 精确，永不截断 |
| 累加器 acc | ACCW=PW+D=36 | S(2+D).30，D=4 | 4 位保护位，无溢出 |
| 移位后 y | EXT=21 | S(EXT+1-16).15 | 算术右移（>>>） |
| 输出 | DW=16 | S1.15 | 先舍入再饱和/截断 |

### 2.1 无溢出证明
|p| < (2^(DW-1)-1)(2^(CW-1)-1) < 2^(PW-2)。任意 k<=TAPS 个乘积的部分和满足
|sum| < 2^(PW-2)·k <= 2^(PW-2+D)。有符号 ACCW 位可表示 |x| < 2^(ACCW-1) =
2^(PW+D-1)，故树中每一级加法均精确。参考模型对每个被接收样本断言
|acc| < 2^(ACCW-1)（逐样本校验）。

### 2.2 舍入与饱和
- ROUND=1：加 2^(FD-1)（FD=FRAC_IN+FRAC_CO-FRAC_OUT=15），四舍五入（半入）。
- 饱和：SAT=1 时对**移位后**值钳位到 [-2^(DW-1), 2^(DW-1)-1]。注意必须先移位
  再钳位：若只检查移位前保护位，则 35283（可放入累加器但超出 16 位输出）会
  错误回绕成 -30253（lpf_random 场景实测发现并修复）。
- 回绕：SAT=0 时取移位后低 DW 位（显式二补码截断）。

## 3. 架构

```
fir_control.sv（握手/有效跟踪/抽取）
  s_ready = m_ready | ~m_valid         锁定握手，无 skid buffer
  shift_en = s_valid & s_ready
  p[CHAIN-1:0]、allow_p[CHAIN-1:0]     位置/抽取标志流水（CHAIN=D+2）
  arrival = shift_en & p[CHAIN-3]      移位后 p[CHAIN-2] 的寄存器等价
  due                                arrival 锁存
  capture_en = due & shift_en & allow_p[CHAIN-2] & (m_ready | ~m_valid)

fir_arith.sv（延迟线/乘法器/加法树/再量化）
  dl[TAPS] 门控于 shift_en（与位置流水锁步冻结）
  乘积 dl_next[i]*coeff[i]（32 位精确）
  加法树 TREE=D+1 级（36 位）
  y_q = 再量化(tree[D][0])，组合输出

fir_filter.sv（顶层/接口）
  m_data_r <= y_q（capture_en 装载）；m_valid=valid_out；m_data=m_data_r
```

关键时序对齐：模型在样本进入末端位置的同一 step 弹出 pipe[CHAIN-1]；RTL 用
预移位视图（allow_p[CHAIN-2]、tree[D]）在同一周期捕获，capture 以 shift_en
限定保证流末尾不多推样本。全部寄存器仅 shift_en 时更新 → 反压下算术流水与
位置流水锁步冻结，不丢样本、不重复样本。

## 4. 算法

y[n] = sat/round( Σ_{i=0}^{TAPS-1} c_i · x[n-i] )
抽取模式：每 DECIM_PERIOD 个被接收样本输出一个 y（抽取计数 dcount 与 allow
标志随样本走流水，反压不破坏对齐）。

## 5. 性能特征（实测）

- 流水线延迟：**D+1 = 5 周期**（首 accept → 首输出；tb_perf 实测
  first_accept=4, first_push=9）。16 抽头时 D=clog2(16)=4。
- 吞吐：连续 valid/ready 下 **1 样本/周期**；抽取模式 **1/DECIM_PERIOD**。
- 每周期可接收一个样本（locked 握手，输出背压时输入同步暂停）。
- 资源：并行乘加结构，每抽头 1 个 16×16→32 乘法器 + 36 位加法树
  （D+1 级）；Yosys 实测 1365 个触发器、约 4.4 万逻辑门（见验证报告）。

## 6. 设计假设与局限

- 系数为编译期常量（packed 参数或 $readmemh ROM），无运行时重配端口。
- 锁定握手：m_valid 拉高且 m_ready 拉低时输入同步暂停（不丢样本是特性，
  但非完全解耦的 AXI-Stream；解耦需外接 FIFO）。
- 并行乘法阵列面积大；可换串行 MAC 以面积换吞吐。
- 综合流程未跑 ABC 门级优化（16 个 36 位乘法器下过慢），无布局布线。

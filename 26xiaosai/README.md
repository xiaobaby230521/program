# 26校赛 — 单相逆变器项目

## 项目简介
2026 届校赛：单相逆变器并联运行系统
- 主控：STM32G474RET6
- 仿真：MATLAB/Simulink
- PCB：立创EDA

## 目录结构
| 目录/文件 | 说明 |
|-----------|------|
| `SinglePhase_GridTie_OffGrid_Inverter_G474/` | STM32G474 Keil MDK 工程 |
| `SinglePhase_GridTie_OffGrid_Inverter_G474/Core/` | CMSIS与启动文件 |
| `SinglePhase_GridTie_OffGrid_Inverter_G474/Drivers/` | HAL外设库 |
| `SinglePhase_GridTie_OffGrid_Inverter_G474/MDK-ARM/` | Keil工程文件 |
| `slprj/` | Simulink 编译产物 |
| `基础部分仿真/` | 校赛基础部分 Simulink 仿真 |
| `基础部分仿真/AC_DC.slx` | AC-DC整流仿真模型 |
| `基础部分仿真/test.m` | 仿真测试脚本 |
| `发挥部分仿真/` | 校赛发挥部分 Simulink 仿真 |
| `发挥部分仿真/单相逆变并网仿真/` | 并网逆变仿真 |
| `发挥部分仿真/单相逆变离网仿真/` | 离网逆变仿真 |
| `双向DC-DC仿真/` | 双向 DC-DC 环节仿真 |
| `双向DC-DC仿真/DC_DC.slx` | 双向DC-DC仿真模型 |
| `双向DC-DC仿真/DC_DC_boost_currentl_openloop.m` | Boost开环控制脚本 |
| `校赛整流逆变一体板.eprj` | 立创EDA 工程文件 |
| `校赛整流逆变一体板.epro` | 立创EDA 工程配置 |
| `校赛整流逆变一体板_backup/` | 立创EDA 工程备份（v116） |
| `备份信息.txt` | 项目备份说明 |

## 备注
- 本文件夹通过 git 备份至 `xiaobaby230521/program` 仓库

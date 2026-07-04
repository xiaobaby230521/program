# DC-DC — 双向DC-DC变换器项目

## 项目简介
G1 双向 DC-DC 变换器
- 主控：STM32G4 系列
- 驱动：IR2104
- PCB：立创EDA（含 Gerber 输出）

## 目录结构
| 目录/文件 | 说明 |
|-----------|------|
| `code/` | STM32G474 Keil MDK 工程 |
| `code/Core/` | CMSIS与启动文件 |
| `code/Drivers/` | HAL外设库（CMSIS + STM32G4xx HAL） |
| `code/MDK-ARM/` | Keil工程（.uvprojx / .uvoptx / DebugConfig / RTE） |
| `code/mycode/` | 用户代码（Inc / Src） |
| `G1_双向DC.eprj` | 立创EDA 主工程文件 |
| `G1_双向DC.epro` | 立创EDA 工程配置 |
| `G1_双向DC_backup/` | 立创EDA 历史备份（4个版本ZIP） |
| `Gerber_PCB_双向DC-DC_大电感_*.zip` | PCB Gerber 输出文件 |
| `ir2104_mos管驱动芯片.pdf` | IR2104 驱动芯片 datasheet — [下载](https://github.com/xiaobaby230521/program/releases/download/%E8%AE%BA%E6%96%87%E9%99%84%E4%BB%B6_20260703/papers_dcdc_datasheet.zip) |
| `仿真文件/` | MATLAB仿真模型与数据 |
| `仿真文件/boost_contrl.m` | Boost控制MATLAB脚本 |
| `仿真文件/buck_control.m` | Buck控制MATLAB脚本 |
| `仿真文件/*.mat` | 控制器离散化参数数据 |
| `相关论文/` | 双向DC-DC变换器论文（6篇PDF）— [下载](https://github.com/xiaobaby230521/program/releases/download/%E8%AE%BA%E6%96%87%E9%99%84%E4%BB%B6_20260703/papers_dcdc_papers.zip) |

## 备注
- 本文件夹通过 git 备份至 `xiaobaby230521/program` 仓库

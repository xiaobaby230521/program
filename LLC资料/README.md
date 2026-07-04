# LLC资料 — LLC谐振变换器资料

## 项目简介
STM32G474RET6 控制的 LLC 谐振变换器数字控制程序及相关参考资料。

## 目录结构
| 目录/文件 | 说明 |
|-----------|------|
| `程序/` | STM32G474 Keil MDK 工程（G474RET6_Control_TSC_LLC） |
| `程序/.mxproject` | MDK项目配置 |
| `程序/Core/` | CMSIS与启动文件 |
| `程序/Drivers/` | HAL外设库 |
| `程序/G474RET6_Control_TSC_LLC.ioc` | STM32CubeMX 工程配置 |
| `程序/G474RET6_Control_TSC_LLC_TEST2.zip` | 工程压缩包 |
| `程序/MDK-ARM/` | Keil工程（.uvprojx / .uvoptx / DebugConfig / RTE） |
| `程序/User_Code_Include/` | 用户代码头文件（Cordic / Digital_Control） |
| `工程/` | 立创EDA 工程文件 |
| `工程/ProPrj_时移LLC开发板_*.epro` | 时移LLC开发板EDA工程 |
| `PSIM仿真/` | PSIM 仿真模型 |
| `PSIM仿真/TSC_LLC_V2.fra` | PSIM频率分析文件 |
| `PSIM仿真/TSC_LLC_V2.psimsch` | PSIM原理图 |
| `PSIM仿真/TSC_LLC_V2.smv` | PSIM仿真数据 |
| `环路设计/` | 环路设计文件 |
| `环路设计/bodedate.mat` | Bode数据MAT文件 |
| `环路设计/inverter.m` | 逆变器MATLAB脚本 |
| `环路设计/matlab数据/` | 环路设计MATLAB数据 |
| `环路设计/扫频图片/` | 扫频Bode图（多组工况PNG） |
| `参考书籍/` | LLC参考书籍（7本PDF） |
| `设计手册/` | LLC设计手册（PDF） |

## 备注
- 本文件夹通过 git 备份至 `xiaobaby230521/program` 仓库

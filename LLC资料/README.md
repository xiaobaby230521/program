# LLC资料 — LLC谐振变换器资料

## 项目简介
STM32G474RET6 控制的 LLC 谐振变换器数字控制程序及相关参考资料。

## 目录结构
| 目录/文件 | 说明 |
|-----------|------|
| `程序/` | STM32G474 Keil MDK 工程（G474RET6_Control_TSC_LLC） |
| `工程/` | 立创EDA 工程文件（ProPrj_时移LLC开发板） |
| `PSIM仿真/` | PSIM 仿真模型（TSC_LLC_V2） |
| `环路设计/` | 环路设计相关文件（Bode数据、MATLAB脚本、扫频图片） |
| `设计手册/` | 设计手册 PDF |
| `参考书籍/` | 参考书籍（部分大文件见下方说明） |

## 大文件说明

以下两个 PDF 超过 GitHub 100MB 限制，**未纳入 git 版本控制**，已上传至 [GitHub Releases](https://github.com/xiaobaby230521/program/releases/tag/LLC%E8%B5%84%E6%96%99_%E5%8F%82%E8%80%83%E4%B9%A6%E7%B1%8D_20260703) 作为附件：

| 附件名 | 原始文件名 | 大小 | 下载 |
|--------|------|------|------|
| `LLC_.pdf` | 开关电源控制环路设计【ISBN号】9787111637233-14812141.pdf | 168 MB | [下载](https://github.com/xiaobaby230521/program/releases/download/LLC%E8%B5%84%E6%96%99_%E5%8F%82%E8%80%83%E4%B9%A6%E7%B1%8D_20260703/LLC_.pdf) |
| `LLC_magnetic_theory.pdf` | 磁性元件理论（[美]Colonel We.T.Mclyman ...）.pdf | 114 MB | [下载](https://github.com/xiaobaby230521/program/releases/download/LLC%E8%B5%84%E6%96%99_%E5%8F%82%E8%80%83%E4%B9%A6%E7%B1%8D_20260703/LLC_magnetic_theory.pdf) |

> **注意**：因 GitHub 上传附件对中文文件名支持不佳，附件名已转为英文简写，下载后请按原始文件名识别。

## 备注
- 本文件夹通过 git 备份至 `xiaobaby230521/program` 仓库

# 三相DC-AC与AC-DC — 三相逆变仿真与PLL

## 项目简介
三相逆变器仿真模型（两电平/三电平T型）及锁相环（PLL）相关论文。

## 目录结构
| 目录/文件 | 说明 |
|-----------|------|
| `两电平三步法、五步法、七步法、三次谐波注入法仿真/` | 两电平SVPWM仿真 |
| `两电平三步法.../MPPT_3pha_Ttype_Inverter_*.slxc` | 三电平SVPWM Simulink模型 |
| `两电平三步法.../Ph3_2Lvl_SVPWM_Comparison.slx` | 两电平SVPWM对比仿真 |
| `两电平三步法.../Ttype3Level_NPC_Inverter_svpwm_v3.slxc` | NPC逆变器SVPWM模型 |
| `两电平三步法.../svpwm_calc.m` | SVPWM计算MATLAB脚本 |
| `两电平三步法.../test.m` | 仿真测试脚本 |
| `两电平三步法.../相关论文/` | SVPWM调制论文（2篇PDF） |
| `三电平T型离网逆变/` | T型三电平离网逆变仿真 |
| `三电平T型离网逆变/ThreeLevel_Ttype_OffGrid_Inverter_Sim.slx` | T型三电平离网逆变Simulink模型 |
| `三电平T型离网逆变/论文/` | T型三电平逆变论文（4篇PDF） |
| `三电平五步法、七步法、三次谐波注入法仿真/` | 三电平SVPWM仿真 |
| `三电平五步法.../TType_OffGrid_Inverter_5Step_7Step_SVPWM_THIPWM_Sim.slx` | 五步法/七步法/三次谐波注入仿真 |
| `三电平五步法.../论文/` | 三电平DPWM论文（2篇PDF） |
| `锁相环/` | PLL仿真与论文 |
| `锁相环/DSOGI_PLL_ThreePhase.slx` | 三相DSOGI-PLL Simulink仿真 |
| `锁相环/*.pdf` | PLL相关论文（5篇PDF） |

## 备注
- 本文件夹通过 git 备份至 `xiaobaby230521/program` 仓库

# 基于 FPGA 的倒立摆与视觉跟踪系统

## 工程结构

| 文件或目录 | 用途 |
| --- | --- |
| `pendulum_vision.gprj` | Gowin 工程入口 |
| `build.tcl` | 命令行构建脚本，包含引脚复用配置 |
| `src/` | Verilog 源码、引脚约束 CST、时序约束 SDC |
| `impl/pendulum_vision_process_config.json` | Gowin 编译配置，保留 SSPI 引脚作为 GPIO 等选项 |
| `impl/pnr/pendulum_vision.fs` | 清理前工程中现有的最新构建位流 |
| `dashboard/` | 网页波形监测程序及界面 |
| `protocol.py` | 串口数据解析与校验 |
| `monitor.py` | 可选的命令行或原生窗口串口监测程序 |
| `requirements.txt` | Python 串口依赖 |
| `generate_reference_init.py` | 摄像头初始化 ROM 生成工具 |
| `third_party/` | 摄像头初始化参考数据和版权声明 |
| `SUBMISSION_MANIFEST.json` | 本次提交文件的 SHA-256 校验清单 |

## 编译和烧录

- 开发板：Tang Nano 20K；器件：GW2AR-LV18QN88C8/I7。
- 顶层模块：`pendulum_vision_top`；系统时钟：27 MHz。
- 使用 Gowin V1.9.12 打开 `pendulum_vision.gprj` 编译；工程路径使用相对路径。
- 保留 `impl/pendulum_vision_process_config.json`，其中 SSPI 引脚作为 GPIO 的配置是编译所需选项。
- 位流路径为 `impl/pnr/pendulum_vision.fs`；Programmer 选择匹配的器件和 SRAM Program 下载。
- SRAM 下载掉电丢失；上电或加载后待机，按板载复位并松开后开始自动起摆。

也可运行命令行构建；其他电脑请替换 Gowin 安装路径：

```powershell
cd E:\codex_fpga_pid\best-end
& 'E:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' build.tcl
```

## 按键操作

外接按键按下接 GND。

| 按键 | 功能 |
| --- | --- |
| 板载 S1/P88、S2/P87 | 复位；上电待机时按住至少 20 ms 并松开，启动自动起摆 |
| K1/P52 | 短按每次累计顺时针 120°，最后一次按下后等待 1 秒执行；长按约 0.6 秒连续旋转，松开后减速停车并保持停止位置 |
| K2/P53 | 进入或退出视觉 X 坐标跟踪 |
| K3/P71 | 普通倒立摆 → 流星锤 → 大风车 → 普通起摆和平衡 |
| K4/P72 | 进入或确认目标转角设置 |

K4 设置模式中：K1 每次加 100°，K2 每次加 10°，K3 每次加 1°，再次按 K4 执行。设置的是相对保持位置的顺时针转角，范围 0～999°，按模 1000 循环。连续两次 K4 表示 0°，不旋转。设置模式中 K1～K3 不执行普通模式功能。

摄像头固定安装，目标 X 坐标映射总计 270° 转动范围；X 死区 5 像素。Y 仅供监测。

## 网页上位机

需要 Python 3.8 或更新版本：

```powershell
cd E:\codex_fpga_pid\best-end
python -m pip install -r requirements.txt
python -u dashboard\app.py --port COM9
```

浏览器访问 `http://127.0.0.1:8765/`。实际串口不为 COM9 时修改 `--port`；同一串口只由一个接收程序占用，退出后台按 Ctrl+C。

网页显示角度、编码器位置、速度、PWM、目标坐标、按键设置与运行模式。K4 设置模式下显示 K1/K2/K3 累计次数及角度，确认后清零显示。清空波形按钮只重置网页缓存和计时，不复位 FPGA。运行时自动生成的 `logs/` 为采集记录，不属于提交源码。

可选终端采集命令：

```powershell
python -u monitor.py --port COM9 --console --seconds 30
```

上位机只负责监测记录，FPGA 独立完成控制。

## 摄像头初始化参考

初始化 ROM 已包含在 `src/ov5640_reference_rom.v`，正常编译无需重新生成。参考数据及版权声明保留在 `third_party/`；生成脚本为 `generate_reference_init.py`。

本次只整理提交内容，保留原有控制源码、参数、约束、编译配置及最新位流；未重新编译固件。
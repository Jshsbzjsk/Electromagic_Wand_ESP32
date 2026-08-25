# TensorFlow 模型训练参考工具

本目录保存参考项目的手势数据采集、可视化、整理和模型训练脚本。

## 主要脚本

- `serial_data.py`：从串口接收并保存传感器数据。
- `train.py`：使用数据集训练 TensorFlow 模型。
- `view.py`、`show2.py`：查看数据曲线或训练数据。
- `Dimo333_Data_Management_Tool.py`：辅助管理和整理数据。

生成 TensorFlow Lite 模型后，历史流程可使用 `xxd -i model.tflite > model.h` 转换为 C/C++ 头文件。该目录中的脚本包含历史参数和本机路径，部分代码已较旧；使用前需要修改路径，并重新确认采样率、时间窗口、传感器轴和模型输入尺寸。

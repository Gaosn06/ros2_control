std_msgs/Header header
# 2. 核心字段（与你的业务匹配）
string[] joint_names          # 关节名称数组
float64[] displacements       # 关节位置数组
float64[] velocities          # 关节速度数组
float64[] efforts             # 关节力矩数组（新增字段）
float64[] accelerations       # 关节加速度数组（新增字段）
string[] sensor_names         # 传感器名称数组（新增字段）
float64[] sensor_tp           # 传感器数据数组（新增字段）
bool is_active                # 控制器激活状态（可选）
string control_mode           # 控制模式（如 "FAST"/"SLOW"，可选）
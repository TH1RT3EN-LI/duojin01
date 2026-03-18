# 单个 AprilTag 检测结果及推断立方体信息

bool tag_detected           # 是否检测到 Tag
int32 tag_id                # Tag ID（未检测到时为 -1）
string tag_family           # Tag 族，例如 "tag36h11"

# 图像坐标系（像素）
float32 center_u            # 中心点列坐标（x）
float32 center_v            # 中心点行坐标（y）
float32 height_px           # Tag 在图像中的像素高度

# 推断立方体的 8 个顶点投影到图像的像素坐标
# 顺序：顶面 4 点（Tag 所在面，索引 0-3）+ 底面 4 点（索引 4-7）
float32[] cube_vertices_u
float32[] cube_vertices_v

# 摄像机坐标系下的三维位置（仅相机已标定时有效）
bool pose_valid             # 三维姿态估算是否有效
float32 pos_x               # 相机坐标系 X（向右）单位：米
float32 pos_y               # 相机坐标系 Y（向下）单位：米
float32 pos_z               # 相机坐标系 Z（向前/深度）单位：米

# 旋转四元数（相机坐标系下）
float32 rot_x
float32 rot_y
float32 rot_z
float32 rot_w

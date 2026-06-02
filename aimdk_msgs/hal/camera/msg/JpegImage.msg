# 相机元信息
string camera_id
string device
uint32 width
uint32 height
uint32 framerate
uint32 exposure         # 曝光值
uint32 gain             # 模拟增益

# JPEG 图片数据
sensor_msgs/CompressedImage image
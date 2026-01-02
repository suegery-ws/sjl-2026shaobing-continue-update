|字节位置|数据类型|内容|说明|
|-|-|-|-|
|0|uint8|`0xFF`|帧头|
|1|uint8|`fire_advice`|开火建议：`0x01`=开火，`0x00`=不开火|
|2|uint8|`is_spining`|是否小陀螺：`0x01`=是，`0x00`=否|
|3|uint8|`is_navigating`|是否导航中：`0x01`=是，`0x00`=否|
|4-7|float|`pitch`|Pitch角度（弧度）|
|8-11|float|`yaw`|Yaw角度（弧度）|
|12-15|float|`distance`|目标距离（米）|
|16-19|float|`linear.x`|底盘线速度X（m/s）|
|20-23|float|`linear.y`|底盘线速度Y（m/s）|
|24-27|float|`angular.z`|底盘角速度Z（rad/s）|
|28-29|-|保留|未使用|
|30|uint8|check_byte|校验字节|
|31|uint8|`0x0D`|帧尾|


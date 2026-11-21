from pyzbar.pyzbar import decode
from PIL import Image
 
# 读取条形码图像
image = Image.open('/home/abot/abot_ws/src/robot_vision/scripts/code129.png')
 
# 解码条形码
decoded_results = decode(image)
 
# 输出识别结果
for result in decoded_results:
    print(f"识别内容: {result.data.decode('utf-8')}")
    print(f"条码类型: {result.type}")

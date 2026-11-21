#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成可被pyzbar识别的条形码/二维码，并验证识别效果
支持类型：CODE128、EAN13、CODE39、QRCODE
"""
import cv2
import numpy as np
from pyzbar import pyzbar
from PIL import Image, ImageDraw, ImageFont
import barcode
from barcode.writer import ImageWriter

# 解决中文显示问题（可选）
try:
    # 加载系统字体（Linux/macOS），Windows可替换为"simhei.ttf"
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
except:
    font = ImageFont.load_default()

def generate_barcode(barcode_type, data, save_path="generated_barcode.png"):
    """
    生成指定类型的条形码/二维码
    :param barcode_type: 条码类型（CODE128/EAN13/CODE39/QRCODE）
    :param data: 条码内容（EAN13需12位数字，最后1位自动补校验位）
    :param save_path: 保存路径
    :return: 生成的图像路径
    """
    try:
        # 1. 生成一维条码（CODE128/EAN13/CODE39）
        if barcode_type in ["CODE128", "EAN13", "CODE39"]:
            # 校验EAN13数据格式（必须是12位数字）
            if barcode_type == "EAN13" and (not data.isdigit() or len(data) != 12):
                raise ValueError("EAN13条码需要12位数字（自动补第13位校验位）")
            
            # 生成条码图像
            barcode_class = barcode.get_barcode_class(barcode_type.lower())
            barcode_obj = barcode_class(data, writer=ImageWriter())
            # 保存条码（默认白色背景，黑色条码）
            barcode_obj.save(save_path.replace(".png", ""))
            
            # 重命名（barcode库会自动加后缀，这里统一为.png）
            import os
            generated_file = f"{save_path.replace('.png', '')}.png"
            if os.path.exists(generated_file):
                os.rename(generated_file, save_path)
        
        # 2. 生成二维码（QRCODE）
        elif barcode_type == "QRCODE":
            # 使用OpenCV生成二维码
            qr = cv2.QRCodeDetector()
            qr_generator = cv2.QRCodeGenerator()
            qr_generator.setBytes(data.encode('utf-8'))
            qr_img = qr_generator.generate((400, 400))  # 生成400x400的二维码
            # 转换为PIL图像并保存
            qr_img_pil = Image.fromarray(qr_img * 255).convert('RGB')
            qr_img_pil.save(save_path)
        
        else:
            raise ValueError(f"不支持的条码类型：{barcode_type}，可选：CODE128/EAN13/CODE39/QRCODE")
        
        print(f"✅ {barcode_type} 条码生成成功，保存至：{save_path}")
        return save_path
    
    except Exception as e:
        print(f"❌ 条码生成失败：{str(e)}")
        return None

def test_barcode_recognition(image_path):
    """
    用pyzbar识别生成的条码，验证效果
    :param image_path: 条码图片路径
    :return: 识别结果
    """
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        return {"error": "无法读取图像文件"}
    
    # 转换为灰度图（提升识别率）
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 用pyzbar识别
    barcodes = pyzbar.decode(gray)
    
    results = []
    for barcode in barcodes:
        results.append({
            "type": barcode.type,
            "data": barcode.data.decode('utf-8'),
            "position": barcode.rect  # (x, y, width, height)
        })
    
    # 打印识别结果
    if results:
        print("\n�� pyzbar识别结果：")
        for res in results:
            print(f"   类型：{res['type']}, 内容：{res['data']}")
    else:
        print("\n❌ pyzbar未识别到任何条码")
    
    return results

# 示例使用
if __name__ == "__main__":
    # 安装依赖
    # pip install python-barcode opencv-python pyzbar pillow
    
    # ========== 生成并测试不同类型的条码 ==========
    # 1. 生成CODE128条码（最常用的一维条码，支持数字/字母/符号）
    code128_path = generate_barcode("CODE128", "AB123456789", "code128_barcode.png")
    if code128_path:
        test_barcode_recognition(code128_path)
    
    # 2. 生成EAN13条码（商品条码，需12位数字）
    ean13_path = generate_barcode("EAN13", "690123456789", "ean13_barcode.png")
    if ean13_path:
        test_barcode_recognition(ean13_path)
    
    # 3. 生成二维码（QRCODE，支持任意文本）
    qrcode_path = generate_barcode("QRCODE", "https://www.example.com?code=123456", "qrcode.png")
    if qrcode_path:
        test_barcode_recognition(qrcode_path)

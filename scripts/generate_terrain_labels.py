#!/usr/bin/env python3
# 文件：generate_terrain_labels.py
# 作用：从测试场碰撞几何读取参数，生成无碰撞的中文地面标牌。

import argparse
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


PROJECT_DIR = Path(__file__).resolve().parents[1]
MAP_PATH = PROJECT_DIR / 'assets/maps/terrain/map.xml'
LABEL_DIR = MAP_PATH.parent / 'labels'
PREFIX = 'terrain_label_'


def numbers(element, attribute):
    return [float(value) for value in element.get(attribute).split()]


def display(value):
    return f'{value:.2f}'.rstrip('0').rstrip('.')


def generate(font_path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(MAP_PATH, parser=parser)
    root = tree.getroot()
    asset = root.find('asset')
    world = root.find('worldbody')
    # 重跑时替换已有标牌；碰撞几何和场地外观不参与修改。
    for parent in (asset, world):
        for element in list(parent):
            if element.get('name', '').startswith(PREFIX):
                parent.remove(element)
    geoms = {g.get('name'): g for g in world.findall('geom')}
    meshes = {g.get('name'): g for g in asset.findall('mesh')}
    LABEL_DIR.mkdir(exist_ok=True)
    created_files = set()

    def add(key, title, detail, x, y, width=2.4):
        name = PREFIX + key
        filename = key + '.png'
        created_files.add(filename)
        canvas = Image.new('RGB', (1024, 320), '#142331')
        draw = ImageDraw.Draw(canvas)
        draw.rounded_rectangle((8, 8, 1016, 312), radius=20, outline='#ffd166', width=7)
        for text, top, requested_size, color in (
            (title, 45, 82, '#ffffff'), (detail, 180, 49, '#ffd166')
        ):
            font_size = requested_size
            font = ImageFont.truetype(str(font_path), font_size)
            while draw.textbbox((0, 0), text, font=font)[2] > 940:
                font_size -= 1
                font = ImageFont.truetype(str(font_path), font_size)
            bounds = draw.textbbox((0, 0), text, font=font)
            draw.text(((1024 - bounds[2]) / 2, top - bounds[1]), text, font=font, fill=color)
        # 先写临时文件再替换，避免已运行的仿真读到未写完的 PNG。
        image_path = LABEL_DIR / filename
        temporary_path = image_path.with_suffix('.png.tmp')
        canvas.save(temporary_path, format='PNG')
        temporary_path.replace(image_path)
        ET.SubElement(asset, 'texture', name=name, type='2d', file='labels/' + filename)
        ET.SubElement(asset, 'material', name=name, texture=name, texrepeat='1 1',
                      texuniform='false', rgba='1 1 1 1', reflectance='0', emission='.5')
        # 薄牌仅用于视觉；独立 group=1，接触掩码均为零，不改变机器人运动。
        ET.SubElement(world, 'geom', name=name, type='box',
                      pos=f'{x:.6g} {y:.6g} .012', size=f'{width / 2:.6g} .375 .006',
                      material=name, group='1', contype='0', conaffinity='0')

    for name, geom in geoms.items():
        pos = numbers(geom, 'pos') if geom.get('pos') else None
        if re.fullmatch(r'platform_\d+cm', name):
            height = 200 * numbers(geom, 'size')[2]
            add(name, f'高台 · {display(height)} cm', '高度 | 平台 150 × 160 cm', pos[0], pos[1] - 1.25)
        elif re.fullmatch(r'stairs_\d+cm_up_1', name):
            tag = name[:-5]
            rise = 200 * numbers(geom, 'size')[2]
            tread = 200 * numbers(geom, 'size')[0]
            total = 200 * numbers(geoms[tag + '_landing'], 'size')[2]
            add(tag, f'楼梯 · 单级 {display(rise)} cm',
                f'踏面 {display(tread)} cm | 6 级 | 总高 {display(total)} cm', pos[0] + 1.35, pos[1] - 1.2)
        elif re.fullmatch(r'slope_\d+deg_up', name):
            vertices = numbers(meshes[geom.get('mesh')], 'vertex')
            length, height = max(vertices[0::3]), max(vertices[2::3])
            angle = round(math.degrees(math.atan2(height, length)))
            add(name, f'斜坡 · {angle}°', f'上 / 下坡 | 水平长 {display(length)} m', pos[0] + 1.4, pos[1] - 1.2)
        elif re.fullmatch(r'trench_\d+cm_near', name):
            tag = name[:-5]
            far = geoms[tag + '_far']
            gap = numbers(far, 'pos')[0] - numbers(far, 'size')[0] - pos[0] - numbers(geom, 'size')[0]
            depth = 200 * numbers(geom, 'size')[2]
            add(tag, f'沟壑 · 净宽 {display(100 * gap)} cm', f'深 {display(depth)} cm | 通道宽 140 cm', pos[0], pos[1] - 1.2)
        elif name == 'plum_entry' or re.fullmatch(r'plum_t\d+_entry', name):
            tag = name[:-6]
            height = 200 * numbers(geom, 'size')[2]
            title = '赛事梅花桩' if tag == 'plum' else '加高梅花桩'
            add(tag, f'{title} · 高 {display(height)} cm',
                '24 桩 | 顶面 20 cm | 同排净距 40 cm', pos[0] + 1, pos[1] - 1.2, 3.2)
        elif re.fullmatch(r'bridge_\d+cm', name):
            width = 200 * numbers(geom, 'size')[1]
            add(name, f'窄桥 · 宽 {display(width)} cm', '长 3 m | 高 10 cm', pos[0], pos[1] - 1.2)
        elif re.fullmatch(r'cross_bar_\d+', name):
            height = 200 * numbers(geom, 'size')[2]
            add(name, f'高墙 {display(height)} cm', '高度', pos[0], pos[1] - 1.2, 1.05)
        elif re.fullmatch(r'cross_slope_\d+deg', name):
            vertices = numbers(meshes[geom.get('mesh')], 'vertex')
            angle = round(math.degrees(math.atan2(max(vertices[2::3]), max(vertices[0::3]))))
            add(name, f'侧倾坡 · {angle}°', '横向倾斜 | 行进方向长 3 m', pos[0], pos[1] - .5)
        elif re.fullmatch(r'low_friction_\d+', name):
            friction = numbers(geom, 'friction')[0]
            add(name, f'低摩擦 · μ = {display(friction)}', '滑动摩擦系数 | 长 3 m', pos[0], pos[1] - 1.2)

    for tier in sorted({name.split('_')[1] for name in geoms if name.startswith('rough_t')}):
        blocks = [geom for name, geom in geoms.items() if name.startswith('rough_' + tier + '_')]
        highest = max(200 * numbers(g, 'size')[2] for g in blocks)
        x = min(numbers(g, 'pos')[0] for g in blocks) + 1.5
        y = min(numbers(g, 'pos')[1] for g in blocks) - .55
        add('rough_' + tier, f'崎岖地形 · 上限 {math.ceil(highest)} cm',
            f'60 块 | 实际最高 {display(highest)} cm', x, y)

    # 清理本脚本上次生成、当前地图已不再引用的图片。
    for path in LABEL_DIR.glob('*.png'):
        if path.name not in created_files:
            path.unlink()
    ET.indent(tree, space='  ')
    temporary_map = MAP_PATH.with_suffix('.xml.tmp')
    tree.write(temporary_map, encoding='unicode')
    temporary_map.replace(MAP_PATH)
    print(f'已生成 {len(created_files)} 个参数标牌：{MAP_PATH}')


def main():
    parser = argparse.ArgumentParser(description='从 terrain 几何生成中文参数标牌，需要 Pillow。')
    parser.add_argument('--font', type=Path, default=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'),
                        help='支持中文的字体文件')
    args = parser.parse_args()
    if not args.font.is_file():
        parser.error('未找到中文字体，请使用 --font 指定字体文件。')
    generate(args.font)


if __name__ == '__main__':
    main()

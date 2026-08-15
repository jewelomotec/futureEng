#!/usr/bin/env python3
"""
VOC→YOLO converter — prepare.py

Official name: Dataset converter.
Not used at race time. Converts Pascal VOC XML + JPEGs into a YOLO dataset
zip for Colab. Edit BASE to your machine path before running.
Named inventory: docs/CODE_CATALOG.md
"""

import os
import shutil
import xml.etree.ElementTree as ET
from sklearn.model_selection import train_test_split

BASE = r'C:\Users\Dell\Documents\Jewel\FutureEng'

# Handles ALL variations found in your XMLs
NAME_MAP = {
    'red':     1,
    'red_b':   1,
    'green':   0,
    'green_b': 0,
}

SOURCES = [
    ('label_red',   'red'),
    ('label_green', 'green'),
]

# ── Step 1: Clean and create output folders ──
out = os.path.join(BASE, 'dataset')
if os.path.exists(out):
    shutil.rmtree(out)
for split in ['train', 'val']:
    for kind in ['images', 'labels']:
        os.makedirs(os.path.join(out, kind, split), exist_ok=True)

# ── Step 2: Collect all valid pairs ──
all_pairs = []
for xml_folder, img_folder in SOURCES:
    xml_dir = os.path.join(BASE, xml_folder)
    img_dir = os.path.join(BASE, img_folder)
    for f in sorted(os.listdir(xml_dir)):
        if not f.endswith('.xml'):
            continue
        img_path = os.path.join(img_dir, f.replace('.xml', '.jpg'))
        if not os.path.exists(img_path):
            print(f"[WARN] missing image: {img_path}")
            continue
        all_pairs.append((os.path.join(xml_dir, f), img_path))

print(f"Total pairs found: {len(all_pairs)}")

# ── Step 3: Train/val split ──
train_pairs, val_pairs = train_test_split(
    all_pairs, test_size=0.2, random_state=42
)
print(f"Train: {len(train_pairs)}  |  Val: {len(val_pairs)}")

# ── Step 4: Convert XML → YOLO and copy images ──
skipped = 0
for split, pairs in [('train', train_pairs), ('val', val_pairs)]:
    for xml_path, img_path in pairs:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        w = int(root.find('size/width').text)
        h = int(root.find('size/height').text)

        lines = []
        for obj in root.findall('object'):
            cls_name = obj.find('name').text.strip()
            cls_id = NAME_MAP.get(cls_name)
            if cls_id is None:
                print(f"[SKIP] unknown class '{cls_name}' in {xml_path}")
                skipped += 1
                continue
            bb = obj.find('bndbox')
            xmin = int(bb.find('xmin').text)
            ymin = int(bb.find('ymin').text)
            xmax = int(bb.find('xmax').text)
            ymax = int(bb.find('ymax').text)
            xc = (xmin + xmax) / 2 / w
            yc = (ymin + ymax) / 2 / h
            bw = (xmax - xmin) / w
            bh = (ymax - ymin) / h
            lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")

        base = os.path.splitext(os.path.basename(img_path))[0]

        # Write label
        with open(os.path.join(out, 'labels', split, base + '.txt'), 'w') as f:
            f.write('\n'.join(lines))

        # Copy image
        shutil.copy2(img_path, os.path.join(out, 'images', split))

# ── Step 5: Write dataset.yaml ──
yaml_content = """path: /content/dataset
train: images/train
val: images/val

nc: 2
names: ['green', 'red']
"""
with open(os.path.join(out, 'dataset.yaml'), 'w') as f:
    f.write(yaml_content)

# ── Step 6: Zip it ──
shutil.make_archive(os.path.join(BASE, 'dataset'), 'zip', BASE, 'dataset')

print(f"\nSkipped annotations: {skipped}")
print(f"Train images: {len(os.listdir(os.path.join(out, 'images', 'train')))}")
print(f"Val images:   {len(os.listdir(os.path.join(out, 'images', 'val')))}")
print(f"\ndataset.zip ready to upload to Colab!")
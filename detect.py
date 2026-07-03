import cv2
import numpy as np
import onnxruntime as ort

# ── CONFIG ──
MODEL_PATH  = 'best.onnx'
LABELS      = ['green', 'red']
COLORS      = [(0, 255, 0), (0, 0, 255)]  # green, red
CONF_THRESH = 0.5
IMG_SIZE    = 640
# ────────────

# Load model
session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name

def preprocess(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))       # HWC → CHW
    img = np.expand_dims(img, axis=0)         # add batch dim
    return img

def postprocess(outputs, orig_w, orig_h):
    # YOLO26 NMS-free output: (1, 300, 6) → [x1, y1, x2, y2, conf, cls]
    preds = outputs[0][0]  # shape (300, 6)
    boxes = []
    for pred in preds:
        x1, y1, x2, y2, conf, cls_id = pred
        if conf < CONF_THRESH:
            continue
        # Scale back to original frame size
        x1 = int(x1 / IMG_SIZE * orig_w)
        y1 = int(y1 / IMG_SIZE * orig_h)
        x2 = int(x2 / IMG_SIZE * orig_w)
        y2 = int(y2 / IMG_SIZE * orig_h)
        boxes.append((x1, y1, x2, y2, float(conf), int(cls_id)))
    return boxes

def draw(frame, boxes):
    for x1, y1, x2, y2, conf, cls_id in boxes:
        color = COLORS[cls_id] if cls_id < len(COLORS) else (255,255,255)
        label = f"{LABELS[cls_id]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame

# ── Main loop ──
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("Running... press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    orig_h, orig_w = frame.shape[:2]
    inp = preprocess(frame)
    outputs = session.run(None, {input_name: inp})
    boxes = postprocess(outputs, orig_w, orig_h)
    frame = draw(frame, boxes)

    # Show count
    green_count = sum(1 for *_, cls_id in boxes if cls_id == 0)
    red_count   = sum(1 for *_, cls_id in boxes if cls_id == 1)
    cv2.putText(frame, f"Green: {green_count}  Red: {red_count}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    cv2.imshow("WRO Block Detector", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
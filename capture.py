import cv2
import os

# Create folders if they don't exist
os.makedirs("red", exist_ok=True)
os.makedirs("green", exist_ok=True)

# Open webcam
cap = cv2.VideoCapture(0)  # Change to 1 if you have multiple cameras

if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("=== Photo Capture Tool ===")
print("Press 'r' to save a RED cuboid photo")
print("Press 'g' to save a GREEN cuboid photo")
print("Press 'q' to quit")

red_count = len(os.listdir("red"))
green_count = len(os.listdir("green"))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Show live view
    cv2.imshow("Capture", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):
        filename = f"red/red_{red_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        red_count += 1
        print(f"Saved {filename} (total red: {red_count})")

    elif key == ord('g'):
        filename = f"green/green_{green_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        green_count += 1
        print(f"Saved {filename} (total green: {green_count})")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\nDone! Red: {red_count} photos, Green: {green_count} photos")
from ultralytics import YOLO
import cv2

# Загружаем легкую модель YOLOv8n (nano)
print("Загружаем модель YOLOv8...")
model = YOLO('yolov8n.pt')

# Тестируем на изображении из интернета
print("Выполняем детекцию...")
results = model('https://ultralytics.com/images/bus.jpg')

# Выводим информацию о найденных объектах
for r in results:
    print(f"Найдено объектов: {len(r.boxes)}")
    for box in r.boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        class_name = model.names[class_id]
        print(f"Объект: {class_name}, уверенность: {confidence:.2f}")

print("Тест завершен успешно!")

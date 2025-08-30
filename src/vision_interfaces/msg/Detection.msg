# Описывает один распознанный объект

string class_name      # Имя класса объекта (например, "traffic cone", "person")
int32 class_id         # ID класса объекта в модели YOLO
float32 confidence     # Уверенность в распознавании (от 0.0 до 1.0)

# Нормализованные координаты ограничивающей рамки (от 0.0 до 1.0)
float32 x_center_norm
float32 y_center_norm
float32 width_norm
float32 height_norm
float32 distatce_m # дистанция до объекта в метрах
# Proveedor YOLO de detección de personas

`YoloDetectPeople` adapta el flujo `yolo_msgs/msg/DetectionArray` al contrato
portable `hri_capability_interfaces/srv/DetectPeople`.

El runner C++ se limita al ciclo de vida exigido por `pluginlib` y a traducir
mensajes. La inferencia permanece en el nodo Python de `yolo_ros`; este
proveedor no inicia, configura ni detiene ese proceso.

## Flujo

```text
/yolo/detections
        ↓
YoloDetectPeopleRunner
        ↓
/hri/detect_people
```

El proveedor:

- acepta solamente detecciones cuya clase sea exactamente `person`;
- aplica el umbral solicitado, o `0.5` cuando se solicita un valor no positivo;
- rechaza umbrales superiores a `1.0` y solicitudes con valores `NaN` o infinitos;
- usa una edad máxima predeterminada de un segundo;
- descarta detecciones con confianza o geometría no finitas y tamaños negativos;
- diferencia un frame reciente vacío de datos ausentes o vencidos;
- elimina su servicio y su suscripción cuando Capabilities2 lo libera.

Los cuadros se expresan en píxeles del frame procesado. El identificador de
seguimiento queda vacío cuando `yolo_ros` se ejecuta sin tracking.

## Compilación

Con `capabilities2`, `portable-hri-interfaces` y `yolo_ros` presentes en el
workspace:

```bash
colcon build --packages-up-to hri_yolo_providers --symlink-install
source install/setup.bash
```

La prueba simulada se encuentra en `portable-hri-reference-system`:

```bash
python3 tests/capabilities_yolo_detect_people_probe.py
```

La prueba no requiere cámara ni robot. Publica frames YOLO ficticios y valida
catálogo, selección, filtrado, traducción, ausencia, vencimiento y liberación.

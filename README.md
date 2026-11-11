# Portable HRI Providers

Implementaciones de las capacidades comunes definidas en
`portable-hri-interfaces`. Los paquetes de este repositorio adaptan servicios
específicos de robots o backends sin introducir esos detalles en la aplicación.

## Proveedores

`hri_naoqi_providers` contiene el proveedor `NaoSpeak`, administrado por
Capabilities2. Expone `/hri/speak` y delega de forma no animada y síncrona al
servicio SinfonIA `/nao/say`.

El mismo paquete contiene `NaoqiMoveRelative`, que expone
`/hri/move_relative`, serializa solicitudes, aplica límites y verifica el
resultado de `/nao/move_to` mediante odometría. El proveedor no modifica la
vida autónoma. Los nombres internos pueden remapearse a Pepper porque el
contrato de la aplicación permanece igual.

Consulta [hri_naoqi_providers/README.md](hri_naoqi_providers/README.md) para
dependencias, compilación y limitaciones.

`hri_yolo_providers` contiene `YoloDetectPeople`. El proveedor consume
`/yolo/detections`, conserva únicamente el último frame reciente, filtra la
clase `person` y expone el resultado portable en `/hri/detect_people`.
`yolo_ros` continúa siendo el motor externo de inferencia y debe ejecutarse por
separado.

Consulta [hri_yolo_providers/README.md](hri_yolo_providers/README.md) para la
semántica de los resultados y sus dependencias.

`hri_whisper_providers` implementa `TranscribeSpeech` con un proxy de
Capabilities2 y un worker local separado. El modo continuo usa WebRTC VAD,
endpoint de 0.8 s de silencio y segmentos de hasta 10 s. La captura permanece
activa durante la inferencia, con un máximo de dos segmentos pendientes;
descarta el más antiguo si la cola se llena y comunica el contador de pérdidas.
Liberar la capacidad vacía los buffers y descarta resultados tardíos. El modo
de una sola ventana sigue disponible y no requiere VAD.

El worker usa el modelo Whisper local ya instalado. Para la dependencia VAD,
desde la raíz del workspace y después de compilar:

```bash
python3 -m pip install --no-deps --target install/hri_whisper_providers/lib/hri_whisper_providers \
  -r src/portable-hri-providers/hri_whisper_providers/requirements.txt
```

La dependencia se instala dentro del workspace; repetir si se elimina `install/`.
Los tests `hri_whisper_providers/test/test_*.py` verifican segmentación, límites,
captura durante inferencia lenta, cancelación y resultados sin hardware.

## Licencia

El código propio se distribuye bajo MIT. Las dependencias externas conservan
sus licencias; `naoqi_utilities_msgs` se referencia externamente y no se copia.

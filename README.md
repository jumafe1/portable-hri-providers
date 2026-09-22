# Portable HRI Providers

Implementaciones de las capacidades comunes definidas en
`portable-hri-interfaces`. Los paquetes de este repositorio adaptan servicios
específicos de robots o backends sin introducir esos detalles en la aplicación.

## Proveedores

`hri_naoqi_providers` contiene el proveedor `NaoSpeak`, administrado por
Capabilities2. Expone `/hri/speak` y delega de forma no animada y síncrona al
servicio SinfonIA `/nao/say`.

El mismo paquete contiene `NaoqiMoveRelative`, que expone
`/hri/move_relative`, desactiva la vida autónoma, serializa solicitudes,
aplica límites y verifica el resultado de `/nao/move_to` mediante odometría.
Los nombres internos pueden remapearse a Pepper porque el contrato de la
aplicación permanece igual.

Consulta [hri_naoqi_providers/README.md](hri_naoqi_providers/README.md) para
dependencias, compilación y limitaciones.

`hri_yolo_providers` contiene `YoloDetectPeople`. El proveedor consume
`/yolo/detections`, conserva únicamente el último frame reciente, filtra la
clase `person` y expone el resultado portable en `/hri/detect_people`.
`yolo_ros` continúa siendo el motor externo de inferencia y debe ejecutarse por
separado.

Consulta [hri_yolo_providers/README.md](hri_yolo_providers/README.md) para la
semántica de los resultados y sus dependencias.

## Licencia

El código propio se distribuye bajo MIT. Las dependencias externas conservan
sus licencias; `naoqi_utilities_msgs` se referencia externamente y no se copia.

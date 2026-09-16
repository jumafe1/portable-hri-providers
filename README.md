# Portable HRI Providers

Implementaciones de las capacidades comunes definidas en
`portable-hri-interfaces`. Los paquetes de este repositorio adaptan servicios
específicos de robots o backends sin introducir esos detalles en la aplicación.

## Estado inicial

`hri_naoqi_providers` contiene el proveedor `NaoSpeak`, administrado por
Capabilities2. Expone `/hri/speak` y delega de forma no animada y síncrona al
servicio SinfonIA `/nao/say`.

Consulta [hri_naoqi_providers/README.md](hri_naoqi_providers/README.md) para
dependencias, compilación y limitaciones.

## Licencia

El código propio se distribuye bajo MIT. Las dependencias externas conservan
sus licencias; `naoqi_utilities_msgs` se referencia externamente y no se copia.

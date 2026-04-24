PROYECTO: Identificación de RPM mediante Arduino R4 WiFi

1. Descripción General
Este proyecto contiene un script de Python diseñado para procesar archivos de texto con mensajes de la red CAN (Controller Area Network) capturados desde un Dodge Durango 2017 utilizando un Arduino R4 WiFi. El objetivo principal es identificar automáticamente qué identificador (ID) y qué bytes corresponden a las revoluciones por minuto (RPM) del motor.

2. Requisitos del Sistema

    Lenguaje: Python 3.x

    Librerías Necesarias:

        matplotlib (para la generación de gráficos)

        re y collections (incluidas en la instalación estándar de Python)

    Instalación de dependencias: Ejecutar la terminal luego de haber instalado Python (puede ser en la terminal de Windows, Powershell o su IDE de preferencia) y ejecutar el siguiente comando:
    pip install matplotlib

3. Estructura de Archivos

    RPM_CAN.py: Script principal que realiza el filtrado, análisis y graficación.

    Can Datos 1.txt: Log de datos crudos exportado desde el Arduino. Formato requerido: [ID] (DLC) : HEX.

    CAN Campo de Datos.xlsx: Archivo de validación utilizado para corroborar los resultados del script mediante análisis estático.

4. Funcionamiento del Algoritmo
El script utiliza una metodología de "Scoring" (puntuación) para encontrar el mejor candidato de RPM basándose en:

    Filtro de Mensajes: Descarta IDs con menos de 30 apariciones para asegurar relevancia estadística.

    Análisis Multi-escala: Evalúa señales de 8 y 16 bits aplicando diversos factores de conversión automotriz (÷4, ÷8, etc.).

    Criterio de Suavidad: Es el filtro más importante. Las RPM reales cambian de forma fluida; el script penaliza señales con cambios bruscos o aleatorios (ruido/checksums) calculando el promedio de cambio absoluto entre muestras.

    Rango Dinámico: Busca señales que se mantengan dentro de un rango lógico (ej. 0 a 3000 RPM) y que presenten variaciones significativas.

5. Instrucciones de Uso

    Asegúrese de que el archivo de datos (Can Datos 1.txt) esté en la misma carpeta que el script y que el nombre del archivo coincida con el que Python tiene en la última línea del Script.

    Ejecute el script dentro de la terminal:
    python RPM_CAN.py

    El programa imprimirá en consola los 3 mejores candidatos encontrados y abrirá una ventana con las gráficas comparativas.

6. Configuración de Parámetros
Dentro del código, puede ajustar la variable rpm_max_esperado (por defecto 3000) al final del código y el nombre del archivo para adaptar la búsqueda al comportamiento de conducción registrado en la captura.
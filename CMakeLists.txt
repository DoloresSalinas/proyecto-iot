cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mqtt_ssl)

# Incluye el PEM como binario (asegúrate de que la ruta es correcta)
target_add_binary_data(${PROJECT_NAME}.elf "main/emqxsl_ca.pem" TEXT)
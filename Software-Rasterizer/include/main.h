#ifndef MAIN_H
#define MAIN_H
#include "geometry.h"
#include <vector>
#include "../include/rasterizer.h"
#include "../include/raster.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

// typedef unsigned char Rgb[3];

// Atributos dos vértices (multiplicar por 255 ao imprimir na imagem):
Vec3f colorV0 = {1, 0, 0}; // Atributo do vértice 0: Vermelho
Vec3f colorV1 = {0, 1, 0}; // Atributo do vértice 1: Verde
Vec3f colorV2 = {0, 0, 1}; // Atributo do vértice 2: Azul
Triangle color(colorV0, colorV1, colorV2);

Object object;
std::string configFile = "./config/rasterizer.config";
std::string objFile;
std::string texFile;
std::string depthOut;
std::string rasterOut;
// A camera é definida apenas por dois valores, o olho e o ângulo de visão
// Esse ângulo de visão, por sua vez, é definido pelo tamanho do filme e o
// tamanho focal

uint16_t imageWidth, imageHeight;
float top, bottom, left, right; // Limites da tela, usadas para saber se um
                                // ponto está dentro da tela
float focalLength; // Em milímetros, é usado para computar o ângulo de
                        // visão e a distância para o plano de imagem, que
                        // vai ser no near clipping plane
float apertureWidth,
      apertureHeight; // Em "inches", usado para definir o ângulo de
                              // visão e o aspect ratio do filme
float farClipping,
      nearClipping;  // Planos de recorte, definem o campo de visão
Matrix44f cameraToWorld; // Define a posição e orientação da câmera
Matrix44f worldToCamera; // Usado para projetar objetos para o espaço da câmera
bool winding; // Define se a geometria dos vértices é no sentido horário
                  // ou anti-horário 0 - Horário; 1 - Anti-horário

Vec3f up(0, 1, 0);
Vec3f from(5, 5, 5);
Vec3f to(0, 0, 0);

Rgb *framebuffer;
float *depthbuffer; // lembrar de inicializar o depthbuffer com o
                    // far clipping plane

#endif
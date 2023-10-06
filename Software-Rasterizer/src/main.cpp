/**
 *  Pinada style rasterizer
 */

// #define OPTIMIZACAO

// #include "../include/geometry.h"
#include "../include/rasterizer.hpp"
#include "../include/cow.h"
#include "../include/raster.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

typedef char Rgb[3];

// Atributos dos vértices (multiplicar por 255 ao imprimir na imagem):
Vec3f colorV0 = {1, 0, 0}; // Atributo do vértice 0: Vermelho
Vec3f colorV1 = {0, 1, 0}; // Atributo do vértice 1: Verde
Vec3f colorV2 = {0, 0, 1}; // Atributo do vértice 2: Azul

std::vector<uint32_t> triangules;
std::vector<Vec3f> veriticis;
uint32_t n_triangles = 3156;

// A camera é definida apenas por dois valores, o olho e o ângulo de visão
// Esse ângulo de visão, por sua vez, é definido pelo tamanho do filme e o
// tamanho focal

uint16_t imageWidth = 640, imageHeight = 480; // Quadrado por enquanto
float top, bottom, left, right; // Limites da tela, usadas para saber se um
                                // ponto está dentro da tela
float focalLength = 20; // Em milímetros, é usado para computar o ângulo de
                        // visão e a distância para o plano de imagem, que
                        // vai ser no near clipping plane
float apertureWidth = 0.980,
      apertureHeight = 0.735; // Em "inches", usado para definir o ângulo de
                              // visão e o aspect ratio do filme
float farClipping = 100,
      nearClipping = 1;  // Planos de recorte, definem o campo de visão
Matrix44f cameraToWorld; // Define a posição e orientação da câmera
Matrix44f worldToCamera; // Usado para projetar objetos para o espaço da câmera
bool winding = 1; // Define se a geometria dos vértices é no sentido horário
                  // ou anti-horário 0 - Horário; 1 - Anti-horário

Rgb *framebuffer;
float *depthbuffer; // lembrar de inicializar o depthbuffer com o
                    // far clipping plane

int main() {
  std::cout << "Starting\n";

  auto t_start = std::chrono::high_resolution_clock::now();
  uint32_t rendered = 0;
  uint32_t pixels = 0;

  framebuffer = new Rgb[imageWidth * imageHeight];
  depthbuffer = new float[imageWidth * imageHeight];

  /**
   *  Define a cor de background para um roxo meio claro
   */
  for (int i = 0; i < imageWidth * imageHeight; ++i) {
    framebuffer[i][0] = 65;
    framebuffer[i][1] = 50;
    framebuffer[i][2] = 100;
  }

  for (int i = 0; i < imageWidth * imageHeight; ++i) {
    depthbuffer[i] = farClipping;
  }

  Vec3f up(0, 1, 0);
  Vec3f from(5, 5, 5);
  Vec3f to(0, 0, 0);

  // lookAt(cameraToWorld, from, to, up);
  worldToCamera = {
      0.707107,  -0.331295, 0.624695, 0, 0,        0.883452,  0.468521,   0,
      -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1};
  // cameraToWorld.inverse();

  initializeCamera(apertureHeight, apertureWidth, top, bottom, left, right,
                   focalLength, nearClipping);

  /** Passo de rasterização
   *  Para cada pixel na tela, calcular a edge function entre ele e os lados
   *  do triângulo (em raster) Lembre-se que para melhor precisão, mesmo
   *  espaço raster representando os pixels da tela, eles ainda estão em
   *  valores Float, e como queremos o meio do pixel para o teste, a passada
   *  será de 0.5 em 0.5 unidades Se o pixel está dentro do triângulo,
   *  verificar se é visível através de uma comparação à coordenada no
   *  depthbuffer Sendo o pixel visível, atribuir uma cor (inverso das
   *  coordenadas baricêntricas)
   */
  for (int i = 0; i < n_triangles; ++i) {
    Vec3f v0raster;
    Vec3f v1raster;
    Vec3f v2raster;

/**
 * Projeção dos vértices ao espaço câmera -> espaço NDC -> espaço Raster
 * Winding = sentido de definição dos triângulos, horário / anti-horário
 */
#if 1
    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right, vertices[nvertices[3 * i]], v0raster);
    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right,
                   vertices[nvertices[(1 + winding) + 3 * i]], v1raster);
    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right,
                   vertices[nvertices[(2 - winding) + 3 * i]], v2raster);
    readOBJ("./objects/Lowpoly_tree_sample.obj", triangules, veriticis);
    n_triangles = triangules.size() / 3;
#else
    readOBJ("./objects/Lowpoly_tree_sample.obj", triangules, veriticis);
    n_triangles = triangules.size() / 3;

    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right, veriticis[triangules[3 * i]], v0raster);
    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right,
                   veriticis[triangules[(1 + winding) + 3 * i]], v1raster);
    rasterizePoint(worldToCamera, nearClipping, imageWidth, imageHeight, top,
                   bottom, left, right,
                   veriticis[triangules[(2 - winding) + 3 * i]], v2raster);
#endif
    /**
     *  Back-face culling
     *  Se produto escalar entre a direção à que a câmera olha e a normal do
     *  triângulo for menor que 0, isto é, se apontam em direções opostas, não
     *  renderiza o triângulo
     */
    Vec3f normal = (v2raster - v0raster).crossProduct(v1raster - v0raster);
    Vec3f foward = {-worldToCamera[2][0], -worldToCamera[2][1],
                    -worldToCamera[2][2]};
    if (normal.dotProduct(foward) < 0)
      continue;

    /**
     *  Calculamos a área do triângulo para acharmos as coordenadas
     *  baricêntricas
     */
    float triangleArea = edgeFunction(v0raster, v1raster, v2raster);

    uint xMax = INFINITY, xMin = 0, yMax = 0, yMin = INFINITY;
    computeTriangleBounaries(imageWidth, imageHeight, v0raster, v1raster,
                             v2raster, xMax, xMin, yMax, yMin);
#ifndef OPTIMIZACAO
    for (int j = xMin; j <= xMax; ++j) {
      for (int k = yMin; k <= yMax; ++k) {

        Vec3f current(j + 0.5, k + 0.5, 0);
        float weight0, weight1, weight2;

        /**
         *  Calculamos a área formada pelo sub-triângulo entre os lados e o
         *  Ponto
         */
        weight0 = edgeFunction(v1raster, v2raster, current);
        weight1 = edgeFunction(v2raster, v0raster, current);
        weight2 = edgeFunction(v0raster, v1raster, current);

        /**
         *  Se o valor dessa área for negativa, então o ponto está à esquerda de
         *  um dos lados, ou seja, fora do triângulo
         */
        if (!isTopLeft(v0raster, v1raster, v2raster, weight0, weight1, weight2))
          continue;

        weight0 /= triangleArea;
        weight1 /= triangleArea;
        weight2 /= triangleArea;

        /**
         *  Dividimos a área dos sub-triângulos pela área do triângulo para
         *  termos as coordenadas baricêntricas
         */

        Vec3f color;

        computeCorrectZ(v0raster.z, v1raster.z, v2raster.z, weight0, weight1,
                        weight2, current.z);
        computeCorrectAttribute(colorV0, colorV1, colorV2, v0raster.z,
                                v1raster.z, v2raster.z, weight0, weight1,
                                weight2, current.z, color);

        /**
         *  Apenas renderiza o pixel atual se for o mais a próximo da tela
         */
        if (current.z > depthbuffer[imageWidth * k + j])
          continue;

        depthbuffer[imageWidth * k + j] = current.z;

        framebuffer[imageWidth * k + j][0] = (unsigned char)(color[0] * 255);
        framebuffer[imageWidth * k + j][1] = (unsigned char)(color[1] * 255);
        framebuffer[imageWidth * k + j][2] = (unsigned char)(color[2] * 255);
        pixels++;
      }
    }
    rendered++;
#else
    ///////////////////////////////////////////////////OPTIMIZACAO////////////////////////////////////
    Vec3f current(xMin + 0.5, yMin + 0.5, 0);
    float weight0, weight1, weight2;

    float x0_step = (v2raster.y - v1raster.y),
          x1_step = (v0raster.y - v2raster.y),
          x2_step = (v1raster.y - v0raster.y);

    float y0_step = (v2raster.x - v1raster.x),
          y1_step = (v0raster.x - v2raster.x),
          y2_step = (v1raster.x - v0raster.x);

    weight0 = edgeFunction(v1raster, v2raster, current);
    weight1 = edgeFunction(v2raster, v0raster, current);
    weight2 = edgeFunction(v0raster, v1raster, current);

    weight0 /= triangleArea;
    weight1 /= triangleArea;
    weight2 /= triangleArea;

    for (int j = xMin; j <= xMax; ++j) {
      for (int k = yMin; k <= yMax; ++k) {

        float w0 = weight0, w1 = weight1, w2 = weight2;

        /**
         *  Se o valor dessa área for negativa, então o ponto está à esquerda de
         *  um dos lados, ou seja, fora do triângulo
         */
        if (!isTopLeft(v0raster, v1raster, v2raster, w0, w1, w2))
          continue;
        /**
         *  Dividimos a área dos sub-triângulos pela área do triângulo para
         *  termos as coordenadas baricêntricas
         */

        Vec3f color;

        computeCorrectZ(v0raster.z, v1raster.z, v2raster.z, w0, w1, w2,
                        current.z);
        computeCorrectAttribute(colorV0, colorV1, colorV2, v0raster.z,
                                v1raster.z, v2raster.z, w0, w1, w2, current.z,
                                color);

        /**
         *  Apenas renderiza o pixel atual se for o mais a próximo da tela
         */
        if (current.z > depthbuffer[imageWidth * k + j])
          continue;

        depthbuffer[imageWidth * k + j] = current.z;

        framebuffer[imageWidth * k + j][0] = (unsigned char)(color[0] * 255);
        framebuffer[imageWidth * k + j][1] = (unsigned char)(color[1] * 255);
        framebuffer[imageWidth * k + j][2] = (unsigned char)(color[2] * 255);

        current.y++;
        w0 += y0_step;
        w1 += y1_step;
        w2 += y2_step;
        pixels++;
      }

      weight0 += x0_step;
      weight1 += x1_step;
      weight2 += x2_step;
      current.x++;
    }
    rendered++;
#endif
  }
  std::cout << "\nRendered: " << rendered;
  std::cout << "\nPixels: " << pixels;

  writePPM("./images/raster2d.ppm", imageWidth, imageHeight, framebuffer, 255);

  /**
   *  Faz o mapeamento da profundidade para [0:1] para obter uma cor e
   *  cria uma imagem
   */
  float depthMax = -INFINITY;
  for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) {
    if (depthbuffer[i] > depthMax && depthbuffer[i] < INFINITY)
      depthMax = depthbuffer[i];
  }

  float normalZ = 1 / depthMax;
  for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) {
    depthbuffer[i] *= normalZ * 255;
    framebuffer[i][0] = (unsigned char)depthbuffer[i];
    framebuffer[i][1] = (unsigned char)depthbuffer[i];
    framebuffer[i][2] = (unsigned char)depthbuffer[i];
  }

  writePPM("./images/depth2d.ppm", imageWidth, imageHeight, framebuffer, 255);

  delete[] framebuffer;
  delete[] depthbuffer;
  auto t_end = std::chrono::high_resolution_clock::now();
  auto passedTime =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
  std::cerr << "\nWall passed time:  " << passedTime << " ms" << std::endl;
  std::cout << "\nFinished!\n";
}
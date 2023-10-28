#ifndef RASTERIZER_H
#define RASTERIZER_H

#include "geometry.h"

void initializeCamera(const float &apertureH, const float &apertureW, float &t,
                      float &b, float &l, float &r, const float &focalLenght,
                      const float &near) {
  static const float inchToMm = 25.4;
  float filmAspectRatio = apertureW / apertureH;

  /** computar as coordenadas limite da tela   */
  /** t = Top, b = Bottom, r = Right, l = Left */
  t = ((apertureH * inchToMm / 2) / focalLenght) * near;
  b = -t;
  r = t * filmAspectRatio;
  l = -r;
}

/**
 *  Define a orientação da câmera a partir de um ponto de origem (From) e
 *  um ponto de destino (To) ao qual a câmera olhará.
 *  A orientação e posição da câmera é definida na matriz CameraToWorld
 *  Lembrando que uma matriz pode ser interpretada como um sistema de
 *  coordenadas próprio.
 *  Nesta função, calculamos individualmente cada eixo desse sistema
 *  Limitação: Olhar diretamente para cima/baixo gerará erros
 */
void lookAt(Matrix44f &camera, const Vec3f &from, const Vec3f &to,
            const Vec3f &up) {
  // Eixo Z, ao qual a camera fica alinhado. É o vetor no sentido to -> from
  // normalizado (lembrando que a câmera olha no sentido oposto do eixo Z)
  Vec3f z = (from - to).normalize();

  // Eixo X, utilizamos o vetor "up" passado para definir o vetor "direita"
  Vec3f x = up.crossProduct(z).normalize();

  // Eixo Y, calculamos o verdadeiro vetor "cima", agora que temos dois vetores
  // perpendiculares. Como x e z e já são normalizados, não precisamos
  // normalizar este
  Vec3f y = z.crossProduct(x);

  camera[0][0] = x.x, camera[0][1] = x.y, camera[0][2] = x.z; // Eixo X
  camera[1][0] = y.x, camera[1][1] = y.y, camera[1][2] = y.z; // Eixo Y
  camera[2][0] = z.x, camera[2][1] = z.y, camera[2][2] = z.z; // Eixo Z
  camera[3][0] = from.x, camera[3][1] = from.y,
  camera[3][2] = from.z; // Translação
}

/** Passo de projeção
 *  Transformar para espaço câmera (matriz mundo-para-câmera)
 *  Fazer o z-divide (estão na tela)
 *  Converter para NDC (valores estão entre -1 e 1 se estiverem dentro da tela)
 *  Converter para raster (vai retornar as coordenadas em pixels)
 *  @return Vértice em coordenadas raster
 */
inline void rasterizePoint(const Matrix44f &worldToCamera, const float &near,
                           const float &screenW, const float &screenH,
                           const float &t, const float &b, const float &l,
                           const float &r, const Vec3f &point,
                           Vec3f &pointRaster) {
  Vec3f tempPoint;

  // Transforma o ponto para o espaço da câmera
  worldToCamera.multVecMatrix(point, tempPoint);

  // Converte o ponto para a tela
  tempPoint.x = near * tempPoint.x / -tempPoint.z;
  tempPoint.y = near * tempPoint.y / -tempPoint.z;
  tempPoint.z = -tempPoint.z;

  // Converte para o espaço NDC
  tempPoint.x = (2 * tempPoint.x) / (r - l) - (r + l) / (r - l);
  tempPoint.y = (2 * tempPoint.y) / (t - b) - (t + b) / (t - b);

  // Converte para espaço Raster
  pointRaster.x = (tempPoint.x + 1) / 2 * screenW;
  pointRaster.y = (1 - tempPoint.y) / 2 * screenH;
  pointRaster.z = tempPoint.z;
}

/**
 *  Realiza a correção de interpolação perspectiva
 *  Basicamente, quando projetamos pontos do espaço da câmera (ou mundo) para
 *  o espaço da tela, o valor da coordenada z deste ponto não varia linearmente,
 *  então precisamos realizar a interpolação de outra forma
 *  Atributo = Z_do_ponto_corretamente_interpolado * (inverso_dos_atributos *
 *  seus_pesos)
 */
inline void computeCorrectAttribute(Vec3f vAttribute0, Vec3f vAttribute1,
                                    Vec3f vAttribute2, const float &vertex0_Z,
                                    const float &vertex1_Z,
                                    const float &vertex2_Z,
                                    const float &weight0, const float &weight1,
                                    const float &weight2, float &pointZ,
                                    Vec3f &resultAttribute) {
  /**
   *  Pré-divide cada elemento dos atributos por Z de seu respectivo vértice
   *  (Como a função recebe uma cópia, os atributos originais não são alterados)
   */
  vAttribute0 /= vertex0_Z;
  vAttribute1 /= vertex1_Z;
  vAttribute2 /= vertex2_Z;

  /** Interpola os valores de cada atributo */
  resultAttribute[0] = vAttribute0[0] * weight0 + vAttribute1[0] * weight1 +
                       vAttribute2[0] * weight2;
  resultAttribute[1] = vAttribute0[1] * weight0 + vAttribute1[1] * weight1 +
                       vAttribute2[1] * weight2;
  resultAttribute[2] = vAttribute0[2] * weight0 + vAttribute1[2] * weight1 +
                       vAttribute2[2] * weight2;

  // A multiplicação pelo Z (também corretamente interpolado) de cada valor dos
  // atributos interpolados é necessária
  resultAttribute *= pointZ;
}
/* 
 *  Quando um triângulo é projetado para a tela, ao tentarmos interpolar dois de
 *  seus vértices, temos o problema de que distâncias quando projetadas não são
 *  preservadas, então o valor de Z não vária linearmente entre os vértices.
 *  Calculamos então o inverso de Z como sendo igual ao inverso dos Z's dos
 *  vértices multiplicados pelos seus respectivos pesos.
 *  Isto é chamado de "Perspective Correct Interpolation" ou interpolação com
 *  perspectiva corrigida.
 */
inline void computeCorrectZ(const float &vertex0_Z, const float &vertex1_Z,
                            const float &vertex2_Z, const float &weight0,
                            const float &weight1, const float &weight2,
                            float &pointZ) {
  /** Computa o valor de Z corretamente */
  pointZ = 1 / (weight0 / vertex0_Z + weight1 / vertex1_Z + weight2 / vertex2_Z);
}

/**  
 *  Futuro: fazer a otimização
 *  Tecnicamente calcula o determinante de dois vetores 2D, nesse caso os
 *  vetores são Vetor A = b - a, e Vetor B = point - a
 *  | Ax Ay |
 *  | Bx By |
 *  D = Ax * By - Ay * Bx
 *
 *  O resultado dessa função é um valor real que representa a 2 vezes a área do
 *  triângulo formada pelo lado e o ponto O valor dessa área está diretamente
 *  associoado à coordenada baricentrica do vértice oposto ao lado passado As
 *  coordenadas baricêntricas portanto, são a razão entre a área total do
 *  triângulo e a área formada pelo lado(oposto) e o ponto
 */
inline double edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &point) {
  // return (     Ax     *          By     -      Ay     *         Bx      )
  return ((b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x));
}

inline float nextEdge(const float &weight, const float &weightStep) {
  return weight + weightStep;
}

/**
 *  Calcula os limites do triângulo na imagem
 *  @return Verdade se o triângulo estiver dentro da imagem, falso senão
 */
bool computeTriangleBounaries(const uint &imageWidth, const uint &imageHeight,
                              const Vec3f &v0, const Vec3f &v1, const Vec3f &v2,
                              uint &xMax, uint &xMin, uint &yMax, uint &yMin) {

  // Pega os limites
  xMax = std::floor(std::fmax(std::fmax(v0.x, v1.x), v2.x));
  xMin = std::floor(std::fmin(std::fmin(v0.x, v1.x), v2.x));
  yMax = std::floor(std::fmax(std::fmax(v0.y, v1.y), v2.y));
  yMin = std::floor(std::fmin(std::fmin(v0.y, v1.y), v2.y));

  if (xMin < 0 || xMax > imageWidth - 1 || yMin < 0 || yMax > imageHeight - 1)
    return false;

  // Garante que os limites estão dentro da imagem
  xMax = std::min((int32_t)xMax, (int32_t)imageWidth - 1);
  yMax = std::min((int32_t)yMax, (int32_t)imageHeight - 1);
  xMin = std::max((int32_t)0, (int32_t)xMin);
  yMin = std::max((int32_t)0, (int32_t)yMin);

  return true;
}

/**
 *  Essa regra garante que pontos que dividem a mesmo lado de dois triângulos
 *  diferentes não sejam renderizados mais de uma vez
 *  Os pesos passados são tomados como já sendo não-negativos
 */
bool isTopLeft(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2,
               const float &w0, const float &w1, const float &w2) {
  Vec3f side0 = v2 - v1; // Lado que representa o peso 0
  Vec3f side1 = v0 - v2; // Lado que representa o peso 1
  Vec3f side2 = v1 - v0; // Lado que representa o peso 2

  /**
   * Quando o peso0 for 0, então o ponto está na linha oposta ao ponto que
   * correspondende à este peso. Para sabermos se a linha está top-left: linha é
   * horizontal e está acima de todas as outras ou a linha está à esquerda das
   * outras
   */
  if (w0 == 0 || w1 == 0 || w2 == 0) {
    bool isTopleft = true;
    isTopleft &=
        (w0 == 0) ? (side0.y == 0 && side0.x > 0 || side0.y > 0) : true;
    isTopleft &=
        (w1 == 0) ? (side1.y == 0 && side1.x > 0 || side1.y > 0) : true;
    isTopleft &=
        (w2 == 0) ? (side2.y == 0 && side2.x > 0 || side2.y > 0) : true;
    return isTopleft;
  }

  // O ideal seria de alguma forma fazer (w0 | w1 | w2), pois o bit de sinal
  // seria 1 se o de qualquer um dos 3 pesos for 1, mas não é possível com
  // floats
  return (w0 > 0 && w1 > 0 && w2 > 0);
}

#endif
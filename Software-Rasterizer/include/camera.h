#ifndef CAMERA_H
#define CAMERA_H

#include "./geometry.h"
#include "./raster.h"

static const float inchToMm = 25.4;

class Camera {
public:
  Camera();
  Camera(const Vec2<uint32_t> imageSize, const Vec2f aperture, const Vec2f clippingPlane,
         const float focalLenght);

  void setPosition(Matrix44f &position) {
    worldToCamera = position;
    cameraToWorld = worldToCamera.inverse();
  }

  void load();

  /*
   *  Define a orientação da câmera a partir de um ponto de origem (From) e
   *  um ponto de destino (To) ao qual a câmera olhará.
   *  A orientação e posição da câmera é definida na matriz worldToCamera
   *  Lembrando que uma matriz pode ser interpretada como um sistema de
   *  coordenadas próprio.
   *  Nesta função, calculamos individualmente cada eixo desse sistema
   *  Limitação: Olhar diretamente para cima/baixo gerará erros
   */
  void lookAt(const Vec3f &from, const Vec3f &to, const Vec3f &up);

  void rasterizePoint(const Vec3f &point, Vec3f &pointRaster);

  void loadConfigs(std::string file);

  /** Tamanho em pixeis da imagem | x = width, y = height */
  Vec2<uint32_t> imageSize;
  /** Limites da tela, usadas para saber se um ponto está dentro da tela */
  float top, bottom, left, right;
  /**
   *  Em milímetros, é usado para computar o ângulo de
   *  visão e a distância para o plano de imagem, que
   *  vai ser no near clipping plane
   */
  float focalLength;
  /** Em "inches", usado para definir o ângulo de visão e o aspect ratio do
   * filme | x = width, y = height */
  Vec2f aperture;
  /** Planos de recorte, definem o campo de visão | [0] = Near, [1] = Far */
  Vec2f clippingPlane;
  /** Define a posição e orientação da câmera */
  Matrix44f worldToCamera;
  /** Usado para projetar objetos para o espaço da câmera */
  Matrix44f cameraToWorld;
};

#endif
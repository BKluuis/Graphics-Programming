#include "../include/camera.h"

Camera::Camera() {}

Camera::Camera(const Vec2<uint32_t> imageSize, const Vec2f aperture,
               const Vec2f clippingPlane, const float focalLenght) {
  this->imageSize = imageSize;
  this->aperture = aperture;
  this->clippingPlane = clippingPlane;
  this->focalLength = focalLenght;
  this->load();
}

void Camera::load(){
  float filmAspectRatio = this->aperture.x / this->aperture.y;
  // computar as coordenadas limite da tela
  this->top    = ((this->aperture.y * inchToMm / 2) / this->focalLength) * this->clippingPlane[0];
  this->bottom = -this->top;
  this->right  = this->top * filmAspectRatio;
  this->left   = -this->right;
  
  float fov = 360.0 / M_PI * atan((this->aperture.x * inchToMm / 2) / this->focalLength);

  std::cout << "Fov: " << fov << std::endl;
}

void Camera::rasterizePoint(const Vec3f &point, Vec3f &pointRaster) {
  Vec3f tempPoint;

  // Transforma o ponto para o espaço da câmera
  worldToCamera.multVecMatrix(point, tempPoint);

  // Converte o ponto para a tela
  tempPoint.x = clippingPlane.x * tempPoint.x / -tempPoint.z;
  tempPoint.y = clippingPlane.x * tempPoint.y / -tempPoint.z;
  tempPoint.z = -tempPoint.z;

  // Converte para o espaço NDC
  tempPoint.x =
      (2 * tempPoint.x) / (right - left) - (right + left) / (right - left);
  tempPoint.y =
      (2 * tempPoint.y) / (top - bottom) - (top + bottom) / (top - bottom);

  // Converte para espaço Raster
  pointRaster.x = (tempPoint.x + 1) / 2 * imageSize.x;
  pointRaster.y = (1 - tempPoint.y) / 2 * imageSize.y;
  pointRaster.z = tempPoint.z;
}

void Camera::lookAt(const Vec3f &from, const Vec3f &to, const Vec3f &up) {
  /** Eixo Z, ao qual a camera fica alinhado. É o vetor no sentido to -> from
   * normalizado */
  Vec3f z = (from - to).normalize();

  /** Eixo X, utilizamos o vetor "up" passado para definir o vetor "direita"
   */
  Vec3f x = up.crossProduct(z).normalize();

  /**
   *  Eixo Y, calculamos o verdadeiro vetor "cima", agora que temos dois
   * vetores perpendiculares. Como x e z e já são normalizados, não precisamos
   * normalizar este
   */
  Vec3f y = z.crossProduct(x);

  cameraToWorld[0][0] = x.x, cameraToWorld[0][1] = x.y, cameraToWorld[0][2] = x.z; // Eixo X
  cameraToWorld[1][0] = y.x, cameraToWorld[1][1] = y.y, cameraToWorld[1][2] = y.z; // Eixo Y
  cameraToWorld[2][0] = z.x, cameraToWorld[2][1] = z.y, cameraToWorld[2][2] = z.z; // Eixo Z
  cameraToWorld[3][0] = from.x, cameraToWorld[3][1] = from.y, cameraToWorld[3][2] = from.z; // Translação

  this->worldToCamera = cameraToWorld.inverse();    
}

void Camera::loadConfigs(std::string file){
  loadConfig("./config/rasterizer.config", this->imageSize.x, "imageWidth");
  loadConfig("./config/rasterizer.config", this->imageSize.y, "imageHeight");
  loadConfig("./config/rasterizer.config", this->focalLength, "focalLength");
  loadConfig("./config/rasterizer.config", this->aperture.x, "apertureWidth");
  loadConfig("./config/rasterizer.config", this->aperture.y, "apertureHeight");
  loadConfig("./config/rasterizer.config", this->clippingPlane[0], "nearClipping");
  loadConfig("./config/rasterizer.config", this->clippingPlane[1], "farClipping");
  this->load();
}
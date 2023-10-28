#ifndef RASTER_H
#define RASTER_H

#define GET_VARIABLE_NAME(Variable) (#Variable)

#include "./geometry.h"
// #include "./rasterizer.h"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>

typedef uint8_t Rgb[3];

class Triangle{
public:
  Triangle();
  Triangle(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2);

  float isPointInside(const Vec3f &point);
  /**
   *  Essa regra garante que pontos que dividem a mesmo lado de dois triângulos
   *  diferentes não sejam renderizados mais de uma vez
   *  Os pesos passados são tomados como já sendo não-negativos
   */
  bool isTopLeft(const Vec3f &weights);
  /**
   *  Calculamos as áreas dos paralelogramos formadas pelo lado do triângulo e o ponto atual
   */
  void getWeights(Vec3f &weights, const Vec3f &point);
  void correctZ(float &pointZ, Vec3f &weights);
  void getCorrectAttribute(Triangle attribute, const float &pointZ, const Vec3f &weights, Vec3f &resultAttribute);
  bool getBoundaries(const Vec2<uint32_t> &imageSize, uint &xMax, uint &xMin, uint &yMax, uint &yMin);
  Vec3f getNormal() const;

  const Vec3f& operator [] (uint8_t i) const { return vertex[i]; }
  Vec3f& operator [] (uint8_t i) { return vertex[i]; }
  Triangle& operator /= (const float &d) { vertex[0] /= d, vertex[1] /= d, vertex[2] /= d; return *this; }
  Vec3f vertex[3];
};

class Mesh{
public:
  std::vector<uint32_t> triangles;
  std::vector<Vec3f> vertices;
  uint32_t n_triangles;
};

class Texture{
public:
  std::vector<uint32_t> triangles;
  std::vector<Vec2f> vertices;
  Vec2<uint32_t> size;
  Rgb *image;
  uint8_t channels;

  void samplePixel(const Vec2f &coordinates, Vec3f &result);
};

class Object{
public:
  Matrix44f position;
  Mesh mesh;
  Texture texture;
  void readOBJ(std::string file);
  void loadTexture(std::string file);
};

// void readOBJ(std::string file, std::vector<uint32_t> &triangles, std::vector<Vec3f> &vertices);

void writePPM(std::string file, uint32_t width, uint32_t height, Rgb *buffer, uint16_t colorSize);

void readPPM(std::string file, uint32_t &width, uint32_t &height, Rgb *(*buffer), uint16_t &colorSize);

inline double edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &point);

template <typename T>
void loadConfig(std::string file, T &var, std::string varName){
  std::ifstream f(file);
  std::stringstream buffer;
  char line[256];
  std::string s;
  
  while(f.getline(line, 256)){
    buffer.str(line);
    buffer >> s;
    if(s.compare(varName) == 0){
     buffer >> var;
     break;
    }
    buffer.str("");
    buffer.clear();
  }
  f.close();
}
#endif
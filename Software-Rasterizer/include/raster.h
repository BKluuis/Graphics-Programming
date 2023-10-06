#ifndef RASTER_H
#define RASTER_H

#include "geometry.h"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>

typedef char Rgb[3];

class Triangle{
public:
  Triangle &rasterize(const Matrix44f &worldToCamera, const float &near,
                           const float &screenW, const float &screenH,
                           const float &t, const float &b, const float &l,
                           const float &r);
  Triangle rasterized(const Matrix44f &worldToCamera, const float &near,
                           const float &screenW, const float &screenH,
                           const float &t, const float &b, const float &l,
                           const float &r) const;
  float isPointInside(const Vec3f &point);
  void computeWeights();
  void computeCorrectZ(float &pointZ);
  void computeCorrectAttribute(Vec3f vAttribute0, Vec3f vAttribute1,
                                Vec3f vAttribute2, float &pointZ,
                                Vec3f &resultAttribute);
  bool computeTriangleBounaries(const uint &imageWidth, const uint &imageHeight,
                                uint &xMax, uint &xMin, uint &yMax, uint &yMin);

  Vec3f v0, v1, v2;
  float w0, w1, w2;
};

void readOBJ(std::string file, std::vector<uint32_t> &triangles, std::vector<Vec3f> &vertices);

void writePPM(std::string file, uint32_t width, uint32_t height, Rgb *buffer, uint16_t colorSize);
#endif
#include "../include/raster.h"
#include <cstring>

// typedef char Rgb[3];

// Triangle &Triangle::rasterize(const Matrix44f &worldToCamera, const float
// &near,
//                               const float &screenW, const float &screenH,
//                               const float &t, const float &b, const float &l,
//                               const float &r) {}
// Triangle Triangle::rasterized(const Matrix44f &worldToCamera, const float
// &near,
//                               const float &screenW, const float &screenH,
//                               const float &t, const float &b, const float &l,
//                               const float &r) const {}
// float Triangle::isPointInside(const Vec3f &point) {}
// void Triangle::computeWeights() {}
// void Triangle::computeCorrectZ(float &pointZ) {
//   pointZ = 1 / (w0 / v0.z + w1 / v1.z + w2 / v2.z);
// }
// void Triangle::computeCorrectAttribute(Vec3f vAttribute0, Vec3f vAttribute1,
//                                        Vec3f vAttribute2, float &pointZ,
//                                        Vec3f &resultAttribute) {}
// bool Triangle::computeTriangleBounaries(const uint &imageWidth,
//                                         const uint &imageHeight, uint &xMax,
//                                         uint &xMin, uint &yMax, uint &yMin) {
//   // Pega os limites
//   xMax = std::floor(std::fmax(std::fmax(v0.x, v1.x), v2.x));
//   xMin = std::floor(std::fmin(std::fmin(v0.x, v1.x), v2.x));
//   yMax = std::floor(std::fmax(std::fmax(v0.y, v1.y), v2.y));
//   yMin = std::floor(std::fmin(std::fmin(v0.y, v1.y), v2.y));

//   if (xMin < 0 || xMax > imageWidth - 1 || yMin < 0 || yMax > imageHeight -
//   1)
//     return false;

//   // Garante que os limites estão dentro da imagem
//   xMax = std::min((int32_t)xMax, (int32_t)imageWidth - 1);
//   yMax = std::min((int32_t)yMax, (int32_t)imageHeight - 1);
//   xMin = std::max((int32_t)0, (int32_t)xMin);
//   yMin = std::max((int32_t)0, (int32_t)yMin);

//   return true;
// }

// inline float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &point)
// {
//   return ((b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x));
// }

/**
 *  Ainda não funciona muito bem
 */
void readOBJ(std::string file, std::vector<uint32_t> &triangles,
             std::vector<Vec3f> &vertices) {
  std::ifstream f(file);
  std::stringstream buffer;
  std::string s;
  char ss[300];

  int count = 0;
  while (f.getline(ss, 300)) {
    count++;
    buffer.str(ss);
    buffer >> s;

    std::cout << buffer.str() << "\n";
    std::cout << s << "\n";

    if (s == "v") {
      Vec3f aux;
      buffer >> aux[0];
      buffer >> aux[1];
      buffer >> aux[2];

      vertices.push_back(aux);

      std::cout << "Line " << count << ": " << aux << "\n";

      buffer.str("");
      buffer.clear();
    }

    if (s == "f") {
      std::vector<float> list;
      float aux = 0;

      while (buffer >> s) {
        aux = std::stof(s.substr(0, s.find('/')));
        list.push_back(aux);
      }

      uint8_t size = list.size();

      for (uint8_t i = 1; i + 1 < size; ++i) {
        triangles.push_back(list[0]);
        triangles.push_back(list[i]);
        triangles.push_back(list[i + 1]);
      }

      buffer.str("");
      buffer.clear();
    }
  }
  std::cout << "Lines read: " << count;
  std::cout << "\nTriangulos: " << triangles.size();
  std::cout << "\nVertices: " << vertices.size();
}

void writePPM(std::string file, uint32_t width, uint32_t height, Rgb *buffer,
              uint16_t colorSize) {
  std::ofstream ofs;
  ofs.open(file, std::ios::out | std::ios::binary);
  ofs << "P6\n" << width << " " << height << "\n" << colorSize << "\n";
  ofs.write((char *)buffer, width * height * 3);
  ofs.close();
}
#include "../include/raster.h"

Triangle::Triangle() { vertex[0] = 0, vertex[1] = 0, vertex[2] = 0; }

Triangle::Triangle(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2) {
  vertex[0] = v0, vertex[1] = v1, vertex[2] = v2;
}

bool Triangle::isTopLeft(const Vec3f &weights) {
  Triangle sides(vertex[2] - vertex[1], vertex[0] - vertex[2],
                 vertex[1] - vertex[0]);

  /**
   * Quando o peso[0] for 0, então o ponto está na linha oposta ao ponto que
   * correspondende à este peso. Para sabermos se a linha está top-left: linha é
   * horizontal e está acima de todas as outras ou a linha está à esquerda das
   * outras
   */
  if (weights[0] == 0 || weights[1] == 0 || weights[2] == 0) {
    bool isTopleft = true;
    isTopleft &= (weights[0] == 0)
                     ? (sides[0].y == 0 && sides[0].x > 0 || sides[0].y > 0)
                     : true;
    isTopleft &= (weights[1] == 0)
                     ? (sides[1].y == 0 && sides[1].x > 0 || sides[1].y > 0)
                     : true;
    isTopleft &= (weights[2] == 0)
                     ? (sides[2].y == 0 && sides[2].x > 0 || sides[2].y > 0)
                     : true;
    return isTopleft;
  }

  // O ideal seria de alguma forma fazer (w0 | w1 | w2), pois o bit de sinal
  // seria 1 se o de qualquer um dos 3 pesos for 1, mas não é possível com
  // floats
  return (weights[0] > 0 && weights[1] > 0 && weights[2] > 0);
}

void Triangle::getWeights(Vec3f &weights, const Vec3f &point) {
  weights[0] = edgeFunction(vertex[1], vertex[2], point);
  weights[1] = edgeFunction(vertex[2], vertex[0], point);
  weights[2] = edgeFunction(vertex[0], vertex[1], point);
}

/** Interpola o valor de Z de cada vértice de acordo com seu peso, retorna o inverso do resultado*/
void Triangle::correctZ(float &pointZ, Vec3f &weights) {
  /** Computa o valor de Z corretamente */
  pointZ = 1 / (weights[0] / vertex[0].z + weights[1] / vertex[1].z +
                weights[2] / vertex[2].z);
}


void Triangle::getCorrectAttribute(Triangle attribute, const float &pointZ,
                                   const Vec3f &weights,
                                   Vec3f &resultAttribute) {
  /**
   *  Pré-divide cada elemento dos atributos por Z de seu respectivo vértice
   *  (Como a função recebe uma cópia, os atributos originais não são alterados)
   */
  attribute /= pointZ;

  /** Interpola os valores de cada atributo */
  resultAttribute[0] = attribute[0].x * weights[0] +
                       attribute[1].x * weights[1] +
                       attribute[2].x * weights[2];
  resultAttribute[1] = attribute[0].y * weights[0] +
                       attribute[1].y * weights[1] +
                       attribute[2].y * weights[2];
  resultAttribute[2] = attribute[0].z * weights[0] +
                       attribute[1].z * weights[1] +
                       attribute[2].z * weights[2];

  // A multiplicação pelo Z (também corretamente interpolado) de cada valor dos
  // atributos interpolados é necessária
  resultAttribute *= pointZ;
}

bool Triangle::getBoundaries(const Vec2<uint32_t> &imageSize, uint &xMax,
                             uint &xMin, uint &yMax, uint &yMin) {
  // Pega os limites
  xMax =
      std::floor(std::fmax(std::fmax(vertex[0].x, vertex[1].x), vertex[2].x));
  xMin =
      std::floor(std::fmin(std::fmin(vertex[0].x, vertex[1].x), vertex[2].x));
  yMax =
      std::floor(std::fmax(std::fmax(vertex[0].y, vertex[1].y), vertex[2].y));
  yMin =
      std::floor(std::fmin(std::fmin(vertex[0].y, vertex[1].y), vertex[2].y));

  if (xMin < 0 || xMax > imageSize.x - 1 || yMin < 0 || yMax > imageSize.y - 1)
    return false;

  // Garante que os limites estão dentro da imagem
  xMax = std::min((int32_t)xMax, (int32_t)imageSize.x - 1);
  yMax = std::min((int32_t)yMax, (int32_t)imageSize.y - 1);
  xMin = std::max((int32_t)0, (int32_t)xMin);
  yMin = std::max((int32_t)0, (int32_t)yMin);

  return true;
}

Vec3f Triangle::getNormal() const {
  return (vertex[2] - vertex[0]).crossProduct(vertex[1] - vertex[0]);
}

void Texture::samplePixel(const Vec2f &coordinates, Vec3f &result){
  
}

void Object::readOBJ(std::string file) {
  std::ifstream f(file);
  std::stringstream buffer;
  std::string s;
  char ss[300];

  while (f.getline(ss, 300)) {
    buffer.str(ss);
    buffer >> s;

    if (s == "v") {
      Vec3f aux;
      buffer >> aux[0];
      buffer >> aux[1];
      buffer >> aux[2];

      this->mesh.vertices.push_back(aux);
    }

    else if (s == "f") {
      std::vector<uint32_t> vertices, textures;
      uint32_t vertex = 0, texture = 0, found1 = 0, found2 = 0;

      while (buffer >> s) {
        found1 = s.find('/');
        vertex = std::stof(s.substr(0, found1));
        vertex--;
        vertices.push_back(vertex);

        found2 = s.find('/', found1 + 1);
        texture = std::stof(s.substr(found1 + 1, found2));
        texture--;
        textures.push_back(texture);
      }

      uint8_t size = vertices.size();

      for (uint8_t i = 1; i + 1 < size; ++i) {
        this->mesh.triangles.push_back(vertices[0]);
        this->mesh.triangles.push_back(vertices[i]);
        this->mesh.triangles.push_back(vertices[i + 1]);

        this->texture.triangles.push_back(textures[0]);
        this->texture.triangles.push_back(textures[i]);
        this->texture.triangles.push_back(textures[i + 1]);
      }
    } else if (s == "vt") {
      Vec2f aux;
      buffer >> aux[0];
      buffer >> aux[1];

      this->texture.vertices.push_back(aux);
    }

    buffer.str("");
    buffer.clear();
  }

  this->mesh.n_triangles = this->mesh.triangles.size() / 3;

  f.close();
}

void Object::loadTexture(std::string file){
  uint16_t colorsize = 0;
  readPPM(file, this->texture.size.x, this->texture.size.y, &this->texture.image, colorsize);
}

void writePPM(std::string file, uint32_t width, uint32_t height, Rgb *buffer,
              uint16_t colorSize) {
  std::ofstream ofs;
  ofs.open(file, std::ios::out | std::ios::binary);
  ofs << "P6\n" << width << " " << height << "\n" << colorSize << "\n";
  ofs.write((char *)buffer, width * height * 3);
  ofs.close();
}


void readPPM(std::string file, uint32_t &width, uint32_t &height,
             Rgb *(*buffer), uint16_t &colorSize) {
  std::ifstream f(file, std::ios::binary);
  std::string s;
  if (!f.is_open())
    std::cerr << "Erro ao abrir o arquivo\n";
  std::getline(f, s);
  if (s.substr(0, 2) != "P6")
    std::cerr << "Erro ao ler o arquivo: Formato errado\n";

  f >> width >> height >> colorSize, f.ignore(1);
  
  (*buffer) = new Rgb[width * height];
  f.read((char *)(*buffer), width * height * 3);
  f.close();
}

inline double edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &point) {
  // return (     Ax     *          By     -      Ay     *         Bx      )
  return ((b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x));
}
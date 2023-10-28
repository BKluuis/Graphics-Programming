/**
 *  Pinada style rasterizer
 */

/**
*  Futuro: implementar conversão ppm para jpeg: 
*  https://sistenix.com/rgb2ycbcr.html
*  https://www.youtube.com/watch?v=Kv1Hiv3ox8I&t=4s
*  vish maria vai ser dificil esse é seu próprio projeto
*/
#define STB_IMAGE_IMPLEMENTATION
#include "../include/stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb/stb_image_write.h"
#include "../include/main.h"
#include "../include/camera.h"

int main() {
  std::cout << "Starting\n";
  
  
  loadConfig(configFile, winding, "winding");
  loadConfig(configFile, objFile, "objFile");
  loadConfig(configFile, depthOut, "depthOut");
  loadConfig(configFile, rasterOut, "rasterOut");
  loadConfig(configFile, up, "up");
  loadConfig(configFile, from, "from");
  loadConfig(configFile, to, "to");
  loadConfig(configFile, texFile, "texFile");
  
  uint32_t rendered = 0;
  uint32_t pixels = 0;

  Camera camera;
  camera.loadConfigs(configFile);
  camera.lookAt(from, to, up);
  object.readOBJ(objFile);
  object.loadTexture(texFile);

  // int channels, w, h;
  // unsigned char* image = stbi_load(texFile.c_str(), &w, &h, &channels, 3);
  // if (image == NULL) { std::cout << "Erro ao carregar imagem\n"; return -1; }
  // object.texture.size.x = w, object.texture.size.y = h;
  // object.texture.image = new Rgb[w * h];
  // std::memcpy(object.texture.image, image, object.texture.size.x * object.texture.size.y * 3);
  // stbi_image_free(image);

  framebuffer = new Rgb[camera.imageSize.x * camera.imageSize.y];
  depthbuffer = new float[camera.imageSize.x * camera.imageSize.y];
  
  /**
   *  Define a cor de background para um roxo meio claro
   */
  for (int i = 0; i < camera.imageSize.x * camera.imageSize.y; ++i) {
    framebuffer[i][0] = 35;
    framebuffer[i][1] = 35;
    framebuffer[i][2] = 35;
  }

  for (int i = 0; i < camera.imageSize.x * camera.imageSize.y; ++i) {
    depthbuffer[i] = camera.clippingPlane[1];
  }
  
  auto t_start = std::chrono::high_resolution_clock::now();
  
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
  for (int i = 0; i < object.mesh.n_triangles; ++i) {
    Triangle raster;
    Triangle texture;

    /**
     * Projeção dos vértices ao espaço câmera -> espaço NDC -> espaço Raster
     * Winding = sentido de definição dos triângulos, horário / anti-horário
     */
    camera.rasterizePoint(object.mesh.vertices[object.mesh.triangles[3 * i]], raster[0]);
    camera.rasterizePoint(object.mesh.vertices[object.mesh.triangles[(1 + winding) + 3 * i]], raster[1]);
    camera.rasterizePoint(object.mesh.vertices[object.mesh.triangles[(2 - winding) + 3 * i]], raster[2]); 

    /**
     *  Back-face culling
     *  Se produto escalar entre a direção à que a câmera olha e a normal do
     *  triângulo for menor que 0, isto é, se apontam em direções opostas, não
     *  renderiza o triângulo
     */
    Vec3f normal = raster.getNormal();
    Vec3f foward = {-camera.worldToCamera[2][0], -camera.worldToCamera[2][1],
                    -camera.worldToCamera[2][2]};
    if (normal.dotProduct(foward) < 0)
      continue;

    /**
     *  Calculamos a área do paralelogramo formado pelos lados do triângulo para acharmos as coordenadas
     *  baricêntricas
     */
    float triangleArea = edgeFunction(raster[0], raster[1], raster[2]);

    uint xMax = INFINITY, xMin = 0, yMax = 0, yMin = INFINITY;
    if(!raster.getBoundaries(camera.imageSize, xMax, xMin, yMax, yMin)) continue;

    /** TODO: para aplicar texturas, para cada pixel, interpola as coordenadas de textura do triangulo (u, v)
    *  correspondente na textura para obtermos o texel (s, t) 
    */
    
    for (int j = xMin; j <= xMax; ++j) {
      for (int k = yMin; k <= yMax; ++k) {
        Vec3f current(j + 0.5, k + 0.5, 0);
        Vec3f weight;
        Vec3f colorResult, texResult;

        
        /**
         *  Calculamos as áreas dos paralelogramos formadas pelo lado do triângulo e o ponto atual
         */
        raster.getWeights(weight, current);
        
        /**
         *  Se o valor dessa área for negativa, então o ponto está à esquerda de
         *  um dos lados, ou seja, fora do triângulo
         */
        if(!raster.isTopLeft(weight)) continue;
        
        /**
         *  Dividimos a área dos sub-triângulos pela área do triângulo para
         *  termos as coordenadas baricêntricas
         */
        weight /= triangleArea;
        
        raster.correctZ(current.z, weight);
        /**
         *  Apenas renderiza o pixel atual se for o mais a próximo da tela
         */
        if (current.z > depthbuffer[camera.imageSize.x * k + j]) continue;
        
        // raster.getCorrectAttribute(color, current.z, weight, colorResult);

        // TESTE DE TEXTURA (Foram feitos teste, claramente há algo errado no cálculo(no entiendo))
        Vec2f uv0 = object.texture.vertices[object.texture.triangles[3 * i]];
        Vec2f uv1 = object.texture.vertices[object.texture.triangles[(1 + winding) + 3 * i]];
        Vec2f uv2 = object.texture.vertices[object.texture.triangles[(2 - winding) + 3 * i]];

        // Vec2f uv0 = Vec2f(raster[0].x, raster[0].y); 
        // Vec2f uv1 = Vec2f(raster[1].x, raster[1].y);
        // Vec2f uv2 = Vec2f(raster[2].x, raster[2].y);
        texture[0] = Vec3f(uv0, 1), texture[1] = Vec3f(uv1, 1), texture[2] = Vec3f(uv2, 1);

        raster.getCorrectAttribute(texture, current.z, weight, texResult);
        
        if(texResult.x < 0) texResult.x -= floor(texResult.x);
        if(texResult.y < 0) texResult.y -= floor(texResult.y);
        if(texResult.x > 1) texResult.x = fmod(texResult.x, 1);
        if(texResult.y > 1) texResult.y = fmod(texResult.y, 1);
        
        texResult.x *= object.texture.size.x, texResult.y *= object.texture.size.y;
        
        // TESTE DE TEXTURA       

        colorResult[0] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][0];
        colorResult[1] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][1];
        colorResult[2] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][2];
        
        depthbuffer[camera.imageSize.x * k + j] = current.z;
        
        framebuffer[camera.imageSize.x * k + j][0] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][0];
        framebuffer[camera.imageSize.x * k + j][1] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][1];
        framebuffer[camera.imageSize.x * k + j][2] = object.texture.image[(int)texResult.x + object.texture.size.x * (int)texResult.y][2];
        
        // framebuffer[camera.imageSize.x * k + j][0] = (unsigned char)(colorResult[0] * 255);
        // framebuffer[camera.imageSize.x * k + j][1] = (unsigned char)(colorResult[1] * 255);
        // framebuffer[camera.imageSize.x * k + j][2] = (unsigned char)(colorResult[2] * 255);
        pixels++;
      }
    }
    rendered++;
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout << "\nRendered: " << rendered;
  std::cout << "\nPixels: " << pixels;

  // image = new unsigned char[camera.imageSize.x * camera.imageSize.y * 3];
  // std::memcpy(image, framebuffer, camera.imageSize.x * camera.imageSize.y * 3);
  // stbi_write_png("output.png", camera.imageSize.x, camera.imageSize.y, 3, image, camera.imageSize.x * 3);
  writePPM(rasterOut, camera.imageSize.x, camera.imageSize.y, framebuffer, 255);
  
  /**
   *  Faz o mapeamento da profundidade para [0:1] para obter uma cor e
   *  cria uma imagem
   */
  float depthMax = camera.clippingPlane[0];
  float depthMin = camera.clippingPlane[1];
  for (uint32_t i = 0; i < camera.imageSize.x * camera.imageSize.y; ++i) {
    if (depthbuffer[i] > depthMax && depthbuffer[i] < camera.clippingPlane[1])
      depthMax = depthbuffer[i];
    else if (depthbuffer[i] < depthMin && depthbuffer[i] > camera.clippingPlane[0])
      depthMin = depthbuffer[i];
  }

  float normalZ = 1 / depthMax;

  for (uint32_t i = 0; i < camera.imageSize.x * camera.imageSize.y; ++i) {
    if(depthbuffer[i] != camera.clippingPlane[1]) depthbuffer[i] -= depthMin;
    depthbuffer[i] *= normalZ * 255;
    framebuffer[i][0] = (unsigned char)depthbuffer[i];
    framebuffer[i][1] = (unsigned char)depthbuffer[i];
    framebuffer[i][2] = (unsigned char)depthbuffer[i];
  }

  
  writePPM(depthOut, camera.imageSize.x, camera.imageSize.y, framebuffer, 255);
  delete[] framebuffer;
  delete[] depthbuffer;
  
  auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  std::cerr << "\nWall passed time:  " << passedTime << " ms" << std::endl;
  std::cout << "\nFinished!\n";
}
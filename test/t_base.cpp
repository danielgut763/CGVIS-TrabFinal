// Testes de regressao do M7 (sem janela). Ver tools/check.sh.
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "scene.h"
#include "physics.h"
#include "level.h"
#include "player.h"
#include "grab.h"
// GetMeshLocalBBox vem de test/stub.cpp
std::vector<AABB> Solid(){std::vector<AABB> v;for(size_t i=0;i<g_Instances.size();++i){auto&in=g_Instances[i];if(!in.solid)continue;glm::vec3 a,b;GetMeshLocalBBox(in.mesh,&a,&b);v.push_back(InstanceCollisionBox((int)i,a,b));}return v;}
int firstMesh(const char* n){for(size_t i=0;i<g_Instances.size();++i) if(g_Instances[i].mesh==n) return (int)i; return -1;}
LevelState L; Player P; const float dt=1.0f/120;
void step(int n, PlayerInput in, float yaw){ for(int k=0;k<n;++k){ UpdatePlayer(P,in,yaw,Solid(),dt); UpdateFallingObjects(-1,PlayerAABB(P),dt); LevelUpdate(L,-1,PlayerAABB(P),dt);} }
// M8: recolocar um objeto tambem endireita (tilt) e zera o estado de tombo/rolamento, como grab.cpp faz ao pegar.
void put(int i, glm::vec3 p, float s){ g_Instances[i].pos=p; g_Instances[i].scale=glm::vec3(s); g_Instances[i].yaw=0; g_Instances[i].vel=glm::vec3(0); g_Instances[i].tilt=glm::mat4(1.0f); g_Instances[i].body=BodyState(); }
int main(){
  BuildScene(g_Instances); LevelInit(L); PlayerInit(P, glm::vec3(-4.6f,0,0));
  PlayerInput none={0,0,0,0,0}, fw={1,0,0,0,0}, fj={1,0,0,0,1};
  // 1) repouso
  std::vector<Instance> before=g_Instances; step(240,none,0);
  float drift=0; for(size_t i=0;i<g_Instances.size();++i) if(g_Instances[i].grabbable) drift=std::max(drift,std::fabs(g_Instances[i].pos.y-before[i].pos.y)+std::fabs(g_Instances[i].pos.x-before[i].pos.x));
  printf("1 repouso: deriva %.5f\n",drift);
  int d0=firstMesh("the_die"), d1=d0+1, r=firstMesh("the_rubik"), pw=firstMesh("the_pawn");
  // 2) o caso da imagem: dado grande 1.5 no botao; cubo 0.36 na borda com 60%% para fora
  put(d0, glm::vec3(4.5f,0.75f,0), 1.5f); step(60,none,0);
  float edge=4.5f+0.75f; // borda +x do dado em x=5.25
  put(r, glm::vec3(edge+0.36f*0.10f, 1.5f+0.18f+0.3f, 0), 0.36f); // centro 3.6 cm fora da borda
  float t=0; for(int k=0;k<600;++k){ step(1,none,0); t+=dt; if(g_Instances[r].pos.y<0.19f && std::fabs(g_Instances[r].vel.y)<1e-6) break; }
  printf("2 cubo com centro 3.6cm fora da borda: caiu ao chao em %.2fs, pos=(%.2f,%.3f,%.2f)\n",t,g_Instances[r].pos.x,g_Instances[r].pos.y,g_Instances[r].pos.z);
  // 3) cubo com centro 3.6 cm DENTRO da borda: deve ficar
  put(r, glm::vec3(edge-0.036f, 1.5f+0.18f+0.3f, 0), 0.36f); step(600,none,0);
  printf("3 cubo com centro 3.6cm dentro: ficou em cima? y=%.3f x=%.3f (esperado y=1.680)\n",g_Instances[r].pos.y,g_Instances[r].pos.x);
  // 4) ponte: dado pequeno apoiado sobre dois cubos com vao no meio
  put(d1, glm::vec3(-2,0.17f,2), 0.34f); put(r, glm::vec3(-2+0.6f,0.17f,2), 0.34f); step(30,none,0);
  put(pw, glm::vec3(-2+0.3f, 0.34f+0.28f+0.2f, 2), 0.56f); // peao (0.67 largo) ponte sobre vao de 0.26 m
  step(300,none,0); printf("4 ponte sobre dois apoios: peao y=%.3f (esperado 0.620) x=%.3f\n",g_Instances[pw].pos.y,g_Instances[pw].pos.x);
  // 5) pilha caindo junta
  put(d0, glm::vec3(2,1.5f,-3), 0.4f); put(d1, glm::vec3(2,1.9f,-3), 0.4f); float gap=0; for(int k=0;k<120;++k){ step(1,none,0); gap=std::max(gap,std::fabs((g_Instances[d1].pos.y-0.2f)-(g_Instances[d0].pos.y+0.2f))); }
  printf("5 pilha: folga max %.4f m (antes 0.045)\n",gap);
  // 6) objeto deslizando para fora encontra parede: nao atravessa
  put(d0, glm::vec3(5.0f,0.5f,-3.78f), 1.0f); step(30,none,0); // dado 1 m: z de -4.28 a -3.28
  put(r, glm::vec3(5.0f, 1.0f+0.18f+0.1f, -4.32f), 0.36f); // centro 4 cm fora da borda -z, encostado na parede
  step(300,none,0); printf("6 desliza contra parede: z_min=%.3f (parede em -4.500) y=%.3f\n",g_Instances[r].pos.z-0.18f,g_Instances[r].pos.y);
  // 7) puzzle completo ainda resolve
  LevelRestart(L); PlayerInit(P,glm::vec3(2.3f,0,0)); d0=firstMesh("the_die"); r=firstMesh("the_rubik");
  put(firstMesh("the_cube")>=0?[&]{for(size_t i=0;i<g_Instances.size();++i) if(g_Instances[i].mesh=="the_cube"&&g_Instances[i].grabbable) return (int)i; return -1;}():-1, glm::vec3(-5,0.18f,-3), 0.36f); put(d0, glm::vec3(4.5f,3,0),1.5f); put(r, glm::vec3(3.25f,2.5f,0),0.85f); step(400,none,0);
  PlayerInput ju={0,0,0,0,1}; int phase=0; float levels[2]={0.85f,1.5f};
  for(int k=0;k<3000 && !L.won;++k){ PlayerInput in=none; if(phase<2){ if(P.on_ground&&P.pos.y>levels[phase]-0.01f) phase++; if(phase<2){ if(P.on_ground) in=ju; else if(P.pos.y>levels[phase]+0.02f) in=fw; } } if(phase==2){ in=(P.pos.x>4.95f&&P.on_ground)?fj:fw; } step(1,in,0); }
  printf("7 puzzle: porta=%.2f venceu=%d fase=%d P=(%.2f,%.2f,%.2f) chao=%d | rubik=(%.2f,%.2f,%.2f) dado=(%.2f,%.2f,%.2f)\n",L.door_offset,L.won,phase,P.pos.x,P.pos.y,P.pos.z,P.on_ground,g_Instances[r].pos.x,g_Instances[r].pos.y,g_Instances[r].pos.z,g_Instances[d0].pos.x,g_Instances[d0].pos.y,g_Instances[d0].pos.z);
  // 8) raio x esfera
  float tt; bool h=RaySphere(glm::vec3(0,0,0),glm::vec3(1,0,0),glm::vec3(5,0.5f,0),1.0f,&tt); printf("8 raio-esfera: hit=%d t=%.3f (esperado %.3f); ", h,tt,5-std::sqrt(0.75f));
  h=RaySphere(glm::vec3(0,0,0),glm::vec3(1,0,0),glm::vec3(5,1.5f,0),1.0f,&tt); printf("fora: hit=%d; ",h);
  h=RaySphere(glm::vec3(0,0,0),glm::vec3(-1,0,0),glm::vec3(5,0,0),1.0f,&tt); printf("atras: hit=%d\n",h);
}

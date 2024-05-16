// Ryan Millett
// MAT201B-2024

#include "al/app/al_App.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Functions.hpp"

#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"

#include "../utils/octree.cpp"
#include <iostream>
#include <string>
// #include "classes/boid_4.cpp"

const int CUBE_SIZE = 20;

const int MAX_BOIDS = 2000;
const int NEIGHBOR_LIMIT = 100;
// const float MAX_BOID_RADIUS = CUBE_SIZE * 0.1;

const int N_PARTICLES = 1500;

using namespace al;

double r() { return rnd::uniformS(); }
Vec3f randomVec3f(float scale = 1.0) {
  return Vec3f(r(), r(), r()) * scale;
}

struct Axes {
  void draw(Graphics &g) {
    Mesh mesh(Mesh::LINES);
    // x axis
    mesh.vertex(-CUBE_SIZE, 0, 0);
    mesh.color(1, 0, 0);
    mesh.vertex(CUBE_SIZE, 0, 0);
    mesh.color(1, 0, 0);

    // y axis
    mesh.vertex(0, -CUBE_SIZE, 0);
    mesh.color(0, 1, 0);
    mesh.vertex(0, CUBE_SIZE, 0);
    mesh.color(0, 1, 0);

    // z axis
    mesh.vertex(0, 0, -CUBE_SIZE);
    mesh.color(0, 0, 1);
    mesh.vertex(0, 0, CUBE_SIZE);
    mesh.color(0, 0, 1);

    g.draw(mesh);
  }
};

string slurp(string fileName);  // forward declaration

struct CommonState {
  // particles
  float pointSize;
  // boids
  Pose boid[MAX_BOIDS];

  int i_boids[MAX_BOIDS][NEIGHBOR_LIMIT];
  // XXX - boid vertex colors???
  Pose pose;
  Vec3f boidCenterMass;
};

struct MyApp : DistributedAppWithState<CommonState> {
  
  Parameter timeStep{"Time Step", "", 3.0, "", 0.0333, 3.0};
  Parameter pointSize{"pointSize", "", 0.5, 0.05, 6.0};
  Parameter DBbRadius{"Dark Blue Boid Vision Radius", "", 0.7, 0.05, 1.5};
  Parameter DBcohesionForce{"Dark Blue Cohesion Force", "", 0.625, 0.0, 1.0};
  Parameter DBseparationForce{"Dark Blue Separation Force", "", 0.65, 0.0, 1.0};
  Parameter DBalignmentForce{"Dark Blue Alignment Force", "", 0.8, 0.0, 1.0};
  Parameter DBturnRate{"Dark Blue Turn Rate", "", 0.3, 0.0001, 1.0};

  Parameter YbRadius{"Yellow Boid Vision Radius", "", 0.7, 0.05, 1.5};
  Parameter YcohesionForce{"Yellow Cohesion Force", "", 0.625, 0.0, 1.0};
  Parameter YseparationForce{"Yellow Separation Force", "", 0.75, 0.0, 1.0};
  Parameter YalignmentForce{"Yellow Alignment Force", "", 1.0, 0.0, 1.0};
  Parameter YturnRate{"Yellow Turn Rate", "", 0.6, 0.0001, 1.0};

  Parameter WbRadius{"White Boid Vision Radius", "", 0.7, 0.05, 1.5};
  Parameter WcohesionForce{"White Cohesion Force", "", 0.625, 0.0, 1.0};
  Parameter WseparationForce{"White Separation Force", "", 0.65, 0.0, 1.0};
  Parameter WalignmentForce{"White Alignment Force", "", 0.75, 0.0, 1.0};
  Parameter WturnRate{"White Turn Rate", "", 0.3, 0.0001, 1.0};

  
  std::vector<Boid> boids;    
  std::vector<Vec3f> food;
  vector<Vec3f> velocity;
  vector<Vec3f> force;
  vector<float> mass;
  Octree* boidTree{nullptr};

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  Axes axes;
  // Mesh predMesh;
  Mesh DBMesh;
  Mesh YMesh;
  Mesh WMesh;
  // Mesh boidMesh;
  Mesh foodMesh;
  // Mesh lineMesh{Mesh::LINES};

  // Nav point;

  ShaderProgram pointShader;

  void onCreate() override {
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));


    // place the camera so that we can see the axes
    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    // nav().pos(CUBE_SIZE, CUBE_SIZE * 0.5, CUBE_SIZE * 1.5);
    // nav().pos(Vec3f(0.0));
    nav().pos(Vec3f(5.0, 0.0, 0.0));
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    // Don't do this:
    // nav().faceToward(0, 0, 0);
    // because it will be interpreted as this:
    // nav().faceToward(Vec3d(0), Vec3d(0), 0);
    // which has no effect because of the final 0!

    // Dark Blue Boid
    DBMesh.primitive(Mesh::TRIANGLE_FAN);
		DBMesh.vertex(0, 0, -3);        // Nose
		DBMesh.color(0, 0.5, 1.0);
		DBMesh.vertex(0, 1, 0);         // Top center edge ("back")
		DBMesh.color(0.45, 0.17, 0.28);
		DBMesh.vertex(-3, -1, 0);       // Left edge
		DBMesh.color(0, 0.15, 0.7);
		DBMesh.vertex(3, -1, 0);        // Right edge
		DBMesh.color(0.08, 0.08, 0.60);
		DBMesh.vertex(0, 1, 0);         // Top center edge, closing the fan
		DBMesh.color(0.45, 0.17, 0.28);

    // Yellow Boid
    YMesh.primitive(Mesh::TRIANGLE_FAN);
		YMesh.vertex(0, 0, -5);      // Nose
		YMesh.color(0.96, 0.71, 0.02);//(0.6, 1.0, 0.2);
		YMesh.vertex(0, 0.5, 0);     // Top center edge ("back")
		YMesh.color(255.0/255.0, 203.0/255.0, 61.0/255.0);
		YMesh.vertex(-1, 0, 0);      // Left edge
		YMesh.color(181.0/255.0, 144.0/255.0, 43.0/255.0);
		YMesh.vertex(1, 0, 0);       // Right edge
		YMesh.color(181.0/255.0, 144.0/255.0, 43.0/255.0);
		YMesh.vertex(0, 0.5, 0);     // Top center edge, closing the fan
		YMesh.color(255.0/255.0, 203.0/255.0, 61.0/255.0);

    // White Boid
    WMesh.primitive(Mesh::TRIANGLE_FAN);
		WMesh.vertex(0, 0, -3);      // Nose
		WMesh.color(1.0, 1.0, 1.0);//(0.6, 1.0, 0.2);
		WMesh.vertex(0, 1, 0);     // Top center edge ("back")
		WMesh.color(1.0, 1.0, 1.0);
		WMesh.vertex(-3, -1, 0);      // Left edge
		WMesh.color(1.0, 1.0, 1.0);
		WMesh.vertex(3, -1, 0);       // Right edge
		WMesh.color(1.0, 1.0, 1.0);
		WMesh.vertex(0, 1, 0);     // Top center edge, closing the fan
		WMesh.color(1.0, 1.0, 1.0);
    
    // setUp(); // init boids

    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };
    foodMesh.primitive(Mesh::POINTS);

    setUp();


      for (int i = 0; i < N_PARTICLES; ++i) {
      }

    if (isPrimary()) {
    }

    for (int i = 0; i < N_PARTICLES; ++i) {
      foodMesh.vertex(randomVec3f(CUBE_SIZE));
      foodMesh.color(randomColor());
      float m = rnd::uniform(8.0, 0.5);
      // float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);
      // using a simplified volume/size relationship
      foodMesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t
    }
  } 
  
  void randomize(Nav& boidNav) {
    boidNav.pos(randomVec3f(CUBE_SIZE*0.95));
    boidNav.quat().set(r(), r(), r(), r()).normalize();
    // boidNav.faceToward(randomVec3f(CUBE_SIZE), 0.5);
  }
  
  bool freeze = false;
  void onAnimate(double dt) override {
    // std::cout << nav().pos() << std::endl;
    if (isPrimary()) {
      if (freeze) return;
      dt *= timeStep.get();
      time += dt;
      boidTree->build(boids);

      Vec3d boidCenterOfMass(0, 0, 0);
      int i = 0;
      for (auto& b : boids) {
        double bRadius;
        double alignmentForce;
        double cohesionForce;
        double separationForce;
        double turnRate;
        if (b.getType() == "DB"){
          bRadius = DBbRadius.get();
          alignmentForce = DBalignmentForce.get();
          cohesionForce = DBcohesionForce.get();
          separationForce = DBseparationForce.get();
          turnRate = DBturnRate.get();
        }else if (b.getType() == "Y"){
          bRadius = YbRadius.get();
          alignmentForce = YalignmentForce.get();
          cohesionForce = YcohesionForce.get();
          separationForce = YseparationForce.get();
          turnRate = YturnRate.get();
        }else if (b.getType() == "W"){
          bRadius = WbRadius.get();
          alignmentForce = WalignmentForce.get();
          cohesionForce = WcohesionForce.get();
          separationForce = WseparationForce.get();
          turnRate = WturnRate.get();
        }else{
          bRadius = DBbRadius.get();
          alignmentForce = DBalignmentForce.get();
          cohesionForce = DBcohesionForce.get();
          separationForce = DBseparationForce.get();
          turnRate = DBturnRate.get();
        }
        boidCenterOfMass += b.bNav.pos();
        b.originAvoidance(2.0, 2.0);
        b.handleBoundary(CUBE_SIZE);

        boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius), b.i_boids);

        b.boidForces(boids, alignmentForce, cohesionForce, separationForce, turnRate);
        b.updatePosition(dt);

        state().boid[i].set(b.bNav);
        i++;
      }
      boidCenterOfMass /= boids.size();
      nav().faceToward(boidCenterOfMass, Vec3d(0, 1, 0), 0.2);
      state().pose = nav();
      
      for (int i = 0; i < boids.size(); i++) {
        for (int j = 0; j < NEIGHBOR_LIMIT; j++) {
          state().i_boids[i][j] = -1;
        }
        for (int j = 0; j < boids[i].i_boids.size() && j < NEIGHBOR_LIMIT; j++) {
          state().i_boids[i][j] = boids[i].i_boids[j];
        }
      }

    } else {
      nav().set(state().pose);

      int i = 0;
      for (auto& b : boids) {
        b.bNav.set(state().boid[i]);
        i++;
      }

      for (int i = 0; i < boids.size(); i++) {
        boids[i].i_boids.clear();
        for (int j = 0; j < NEIGHBOR_LIMIT; j++) {
          int n = state().i_boids[i][j];
          if (n == -1) break;
          boids[i].i_boids.push_back(n);
        }
      }
    
      // cout << boids[0].i_boids.size() << endl;
    }
  }

  bool onKeyDown(Keyboard const& k) override {
    switch (k.key()) {
      case ' ':
        // reset the simulation
        setUp();
        break;
    }
    return true;
  }

  void setUp() {         
      boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);

      boids.clear();
      for (int i = 0; i < MAX_BOIDS; ++i) {
        Boid b;
        if (i < MAX_BOIDS / 4){
          b.setType("Y");
          b.speed_percentage = 1.25f;
        }else if (i < MAX_BOIDS / 2){
          b.setType("W");
          b.speed_percentage = 0.5f;
        }else{
          b.setType("DB");
          b.speed_percentage = 0.75f;
        }
        randomize(b.bNav);
        state().boid[i] = b.bNav.pos();
        boids.push_back(b);
      }
  }

  void onDraw(Graphics& g) override {
    // graphics / drawing settings
    g.clear(0);
    g.meshColor();
    g.pointSize(10);
    // axes.draw(g);

    int i = 0;
    for (auto& b : boids) {
      {
        Nav& a(b.bNav);
        g.pushMatrix();
        g.translate(a.pos());
        g.rotate(a.quat());
        g.scale(
          // (i % 11 != 0) ? 0.01 : 0.005
          (i % 11 != 0) ? 0.03 : 0.02
        );
        if (b.getType() == "DB"){
          g.draw(DBMesh);
        }else if (b.getType() == "Y"){
          g.draw(YMesh);
        }else if (b.getType() == "W"){
          g.draw(WMesh);
        }
        g.popMatrix();
      }
      Mesh m{Mesh::LINES};
      for (int j : b.i_boids) {
        if (i < j) {
          m.vertex(b.bNav.pos());
          m.vertex(boids[j].bNav.pos());
          m.color(1.0, 1.0, 1.0);
        }
      }
      if (m.vertices().size() > 0) {
        //g.draw(m);
      }
      ++i;
    }

    g.shader(pointShader);
    g.shader().uniform("pointSize", state().pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    //g.draw(foodMesh);
    // g.draw(lineMesh);
  }

  void onInit() override {
    auto cuttleboneDomain =
        CuttleboneStateSimulationDomain<CommonState>::enableCuttlebone(this);
    
    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }

    if (isPrimary()) {
      auto guiDomain = GUIDomain::enableGUI(defaultWindowDomain());
      auto &gui = guiDomain->newGUI();
      gui.add(timeStep);
      gui.add(pointSize);
      gui.add(DBcohesionForce);
      gui.add(DBseparationForce);
      gui.add(DBalignmentForce);
      gui.add(DBturnRate);
      gui.add(DBbRadius);
      gui.add(YcohesionForce);
      gui.add(YseparationForce);
      gui.add(YalignmentForce);
      gui.add(YturnRate);
      gui.add(YbRadius);
      gui.add(WcohesionForce);
      gui.add(WseparationForce);
      gui.add(WalignmentForce);
      gui.add(WturnRate);
      gui.add(WbRadius);
    }
  }
};

int main() {
  MyApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}

string slurp(string fileName) {
  fstream file(fileName);
  string returnValue = "";
  while (file.good()) {
    string line;
    getline(file, line);
    returnValue += line + "\n";
  }
  return returnValue;
}

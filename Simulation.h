#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <random>
#include <algorithm>

using namespace std;

extern mt19937 generator;

float randomFloat();
enum class Direction { NORTH, EAST, SOUTH, WEST, UNKNOWN };
Direction randomDirection();

enum class TrafficLightColor { RED, YELLOW, GREEN, UNKNOWN };
enum class SpeedState { STOPPED = 0, HALF_SPEED = 1, FULL_SPEED = 2 }; 
enum class ObjectCategory { CAR, BIKE, PARKED_CAR, STOP_SIGN, TRAFFIC_LIGHT, UNKNOWN };
enum class ViewType { CENTERED, FRONT };

struct Position {
    int x, y;
    bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    int manhattanDistance(const Position& other) const { return abs(x - other.x) + abs(y - other.y); } 
};

struct SensorReading { 
    string objectId = "N/A";
    Position position = {-1, -1};
    ObjectCategory type = ObjectCategory::UNKNOWN;
    float confidence = 0.0f;
    int distance = -1; 
    SpeedState speed = SpeedState::STOPPED;
    Direction direction = Direction::UNKNOWN;
    string signText = "N/A";
    TrafficLightColor color = TrafficLightColor::UNKNOWN;
};

// --- ΙΕΡΑΡΧΙΑ ΑΝΤΙΚΕΙΜΕΝΩΝ ΚΟΣΜΟΥ ---
class WorldObject {
protected:
    string id; 
    Position position; 
    char glyph; 
public:
    WorldObject(const string& cat, int count, Position pos, char g) : position(pos), glyph(g) {
        id = cat + ":" + to_string(count); 
    }
    virtual ~WorldObject() {}
    virtual void update(int currentTick) = 0; 
    const Position& getPosition() const { return position; }
    virtual char getDisplayGlyph() const { return glyph; }
    char getGlyph() const { return getDisplayGlyph(); }
    const string& getId() const { return id; }
    virtual SensorReading getObjectState() const;
};

class StaticObject : public WorldObject {
public:
    StaticObject(const string& cat, int count, Position pos, char g) : WorldObject(cat, count, pos, g) {}
    void update(int currentTick) override {} 
};

class TrafficLight : public StaticObject { 
private:
    TrafficLightColor color; 
    const int RED_TICKS = 4; 
    const int GREEN_TICKS = 8; 
    const int YELLOW_TICKS = 2; 
public:
    TrafficLight(int count, Position pos);
    void update(int currentTick) override;
    TrafficLightColor getColor() const { return color; }
    char getDisplayGlyph() const override { return glyph; }
    SensorReading getObjectState() const override;
};

class StopSign : public StaticObject { 
public:
    StopSign(int count, Position pos) : StaticObject("SIGN", count, pos, 'S') {} 
    SensorReading getObjectState() const override;
};

class StationaryVehicle : public StaticObject { 
public:
    StationaryVehicle(int count, Position pos);
    SensorReading getObjectState() const override;
};

class MovingObject : public WorldObject { 
protected:
    SpeedState speed; 
    Direction direction; 
public:
    MovingObject(const string& cat, int count, Position pos, char g, Direction dir)
        : WorldObject(cat, count, pos, g), speed(SpeedState::HALF_SPEED), direction(dir) {}
    void update(int currentTick) override;
    Direction getDirection() const { return direction; }
    SpeedState getSpeed() const { return speed; }
    SensorReading getObjectState() const override;
};

class Car : public MovingObject {
public:
    Car(int count, Position pos, Direction dir);
    SensorReading getObjectState() const override;
};

class Bike : public MovingObject {
public:
    Bike(int count, Position pos, Direction dir) : MovingObject("BIKE", count, pos, 'B', dir) {} 
    SensorReading getObjectState() const override;
};

// --- ΣΥΣΤΗΜΑΤΑ ΠΛΟΗΓΗΣΗΣ & ΑΙΣΘΗΤΗΡΩΝ ---
class SensorFusionEngine { 
private:
    float minConfidenceThreshold; 
public:
    SensorFusionEngine(float threshold) : minConfidenceThreshold(threshold) {}
    vector<SensorReading> fuseSensorData(const vector<vector<SensorReading>>& allSensorReadings);
};

class NavigationSystem { 
private:
    string id;
    vector<Position> gpsTargets; 
    Position currentTarget;
    SensorFusionEngine fusionEngine; 
public:
    NavigationSystem(int count, const vector<Position>& targets, float threshold);
    vector<SensorReading> syncNavigationSystem(const vector<vector<SensorReading>>& allSensorReadings) {
        return fusionEngine.fuseSensorData(allSensorReadings); 
    }
    SpeedState makeDecision(Position& carPos, Direction& carDir, SpeedState carSpeed, const vector<SensorReading>& fusedReadings);
    const Position& getTarget() const { return currentTarget; }
    bool hasTargets() const { return !gpsTargets.empty(); }
};

class Sensor {
protected:
    string id;
    float baseAccuracy;
    int range; 
    float getBaseConfidence(float baseAcc, ObjectCategory type) const {
        if (baseAcc == 0.99f) return 0.99f; 
        if (baseAcc == 0.87f) return 0.87f; 
        return baseAcc;
    }
    float calculateConfidence(int dist, float baseAcc, ObjectCategory type) const;
public:
    Sensor(const string& type, int count, float acc, int rng) : baseAccuracy(acc), range(rng) {
        id = type + ":" + to_string(count);
    }
    virtual ~Sensor() { cout << "[-SENSOR: " << id << "] Sensor destroyed - No further data from me!" << endl; } 
    virtual vector<SensorReading> collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) = 0;
    int getRange() const { return range; }
};

class LidarSensor : public Sensor { 
public:
    LidarSensor(int count);
    vector<SensorReading> collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) override;
};

class RadarSensor : public Sensor { 
public:
    RadarSensor(int count);
    vector<SensorReading> collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) override;
};

class CameraSensor : public Sensor { 
public:
    CameraSensor(int count);
    vector<SensorReading> collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) override;
};

class SelfDrivingCar : public MovingObject { 
private:
    NavigationSystem navSystem;
    vector<Sensor*> sensors;
public:
    SelfDrivingCar(int count, Position pos, Direction dir, const vector<Position>& targets, float threshold);
    ~SelfDrivingCar() {
        for (Sensor* s : sensors) delete s;
    }
    void performUpdate(int currentTick, const vector<WorldObject*>& worldObjects);
    void update(int currentTick) override {}
    const NavigationSystem& getNavSystem() const { return navSystem; }
};

class GridWorld { 
private:
    int dimX, dimY;
    vector<WorldObject*> objects; 
    SelfDrivingCar* car = nullptr; 
public:
    GridWorld(int x, int y) : dimX(x), dimY(y) { 
        cout << "[+WORLD:GRID] Reticulating splines - Hello, world!" << endl; 
    }
    ~GridWorld() {
        for (WorldObject* obj : objects) delete obj;
        if (car) delete car;
        cout << "[-WORLD:GRID] Goodbye, cruel world!" << endl; 
    }
    void addObject(WorldObject* obj);
    bool updateAllObjects(int currentTick);
    char getGlyphAt(int x, int y) const;
    void visualization_full() const;
    void visualization_pov(int radius, ViewType view) const;
    const SelfDrivingCar* getCar() const { return car; }
};

#endif
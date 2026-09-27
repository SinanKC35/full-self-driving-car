#include "Simulation.h"

mt19937 generator;

float randomFloat() {
    static uniform_real_distribution<> dist(0.0, 1.0);
    return dist(generator);
}

Direction randomDirection() {
    uniform_int_distribution<> dist(0, 3);
    return (Direction)dist(generator);
}

SensorReading WorldObject::getObjectState() const {
    SensorReading sr;
    sr.objectId = id;
    sr.position = position;
    return sr;
}

TrafficLight::TrafficLight(int count, Position pos) : StaticObject("LIGHT", count, pos, 'R'), color(TrafficLightColor::RED) {
    cout << "[+LIGHT: " << id << "] Initialized at (" << pos.x << ", " << pos.y << ") to RED" << endl; 
}

void TrafficLight::update(int currentTick) { 
    int totalCycle = RED_TICKS + GREEN_TICKS + YELLOW_TICKS;
    int t = currentTick % totalCycle;
    if (t < RED_TICKS) color = TrafficLightColor::RED;
    else if (t < RED_TICKS + GREEN_TICKS) color = TrafficLightColor::GREEN;
    else color = TrafficLightColor::YELLOW;
    if (color == TrafficLightColor::RED) glyph = 'R'; 
    else if (color == TrafficLightColor::YELLOW) glyph = 'Y'; 
    else glyph = 'G'; 
}

SensorReading TrafficLight::getObjectState() const {
    SensorReading sr = WorldObject::getObjectState();
    sr.type = ObjectCategory::TRAFFIC_LIGHT;
    sr.color = color;
    return sr;
}

SensorReading StopSign::getObjectState() const {
    SensorReading sr = WorldObject::getObjectState();
    sr.type = ObjectCategory::STOP_SIGN;
    sr.signText = "STOP"; 
    return sr;
}

StationaryVehicle::StationaryVehicle(int count, Position pos) : StaticObject("PARKED", count, pos, 'P') { 
    cout << "[+PARKED: " << id << "] Parked at (" << pos.x << ", " << pos.y << ")" << endl; 
}

SensorReading StationaryVehicle::getObjectState() const {
    SensorReading sr = WorldObject::getObjectState();
    sr.type = ObjectCategory::PARKED_CAR;
    return sr;
}

void MovingObject::update(int currentTick) { 
    int movement = (int)speed;
    if (movement == 0) return;
    int dx = 0, dy = 0;
    if (direction == Direction::NORTH) dy = -movement;
    else if (direction == Direction::SOUTH) dy = movement;
    else if (direction == Direction::EAST) dx = movement;
    else if (direction == Direction::WEST) dx = -movement;
    position.x += dx;
    position.y += dy;
}

SensorReading MovingObject::getObjectState() const {
    SensorReading sr = WorldObject::getObjectState();
    sr.speed = speed;
    sr.direction = direction;
    return sr;
}

Car::Car(int count, Position pos, Direction dir) : MovingObject("CAR", count, pos, 'C', dir) { 
    cout << "[+CAR: " << id << "] Initialized at (" << pos.x << ", " << pos.y << ") facing " << (int)dir << endl; 
}

SensorReading Car::getObjectState() const {
    SensorReading sr = MovingObject::getObjectState();
    sr.type = ObjectCategory::CAR;
    return sr;
}

SensorReading Bike::getObjectState() const {
    SensorReading sr = MovingObject::getObjectState();
    sr.type = ObjectCategory::BIKE;
    return sr;
}

vector<SensorReading> SensorFusionEngine::fuseSensorData(const vector<vector<SensorReading>>& allSensorReadings) {
    map<string, vector<SensorReading>> readingsById;
    vector<SensorReading> fusedReadings;
    for (const auto& sensorReadings : allSensorReadings) {
        for (const auto& sr : sensorReadings) {
            if (sr.objectId != "N/A") {
                readingsById[sr.objectId].push_back(sr);
            }
        }
    }
    for (const auto& pair : readingsById) {
        const vector<SensorReading>& readings = pair.second;
        SensorReading fused = readings[0]; 
        float totalWeight = 0.0f;
        float totalDist = 0.0f;
        float maxConfidence = 0.0f;
        bool isBikeDetected = false;
        for (const auto& sr : readings) {
            maxConfidence = max(maxConfidence, sr.confidence);
            totalWeight += sr.confidence;
            if (sr.distance > 0) totalDist += sr.distance * sr.confidence; 
            if (sr.type == ObjectCategory::BIKE) isBikeDetected = true; 
            if (sr.color != TrafficLightColor::UNKNOWN) fused.color = sr.color;
            if (sr.signText != "N/A") fused.signText = sr.signText;
            if (sr.type != ObjectCategory::UNKNOWN) fused.type = sr.type;
            if (sr.direction != Direction::UNKNOWN) fused.direction = sr.direction;
            if (sr.speed != SpeedState::STOPPED) fused.speed = sr.speed;
        }
        if (maxConfidence < minConfidenceThreshold / 100.0f && !isBikeDetected) continue; 
        fused.confidence = maxConfidence;
        if (totalWeight > 0) fused.distance = round(totalDist / totalWeight);
        fusedReadings.push_back(fused);
    }
    return fusedReadings;
}

NavigationSystem::NavigationSystem(int count, const vector<Position>& targets, float threshold)
    : fusionEngine(threshold) {
    id = "NAV:" + to_string(count);
    gpsTargets = targets;
    if (!gpsTargets.empty()) currentTarget = gpsTargets.front();
    cout << "[+NAV: " << id << "] Hello, I'll be your GPS today. Target: (" << currentTarget.x << ", " << currentTarget.y << ")" << endl; 
}

SpeedState NavigationSystem::makeDecision(Position& carPos, Direction& carDir, SpeedState carSpeed, const vector<SensorReading>& fusedReadings) {
    if (!hasTargets()) return SpeedState::STOPPED;
    int distToTarget = carPos.manhattanDistance(currentTarget); 
    SpeedState decision = carSpeed;
    Direction requiredDir = carDir;
    
    if (distToTarget == 0) {
        gpsTargets.erase(gpsTargets.begin());
        if (gpsTargets.empty()) {
            cout << "[-NAV: " << id << "] You've arrived! Shutting down..." << endl; 
            return SpeedState::STOPPED;
        }
        currentTarget = gpsTargets.front();
        distToTarget = carPos.manhattanDistance(currentTarget);
    }

    for (const auto& sr : fusedReadings) {
        if (sr.distance > 0) {
            if ((sr.color == TrafficLightColor::RED || sr.color == TrafficLightColor::YELLOW) && sr.distance <= 3) {
                cout << "WARNING: Stopping for Traffic Light (" << sr.objectId << ")" << endl;
                return SpeedState::STOPPED;
            }
            if ((sr.type == ObjectCategory::CAR || sr.type == ObjectCategory::BIKE) && sr.distance <= 2) {
                cout << "WARNING: Stopping for Moving obstacle (" << sr.objectId << ")" << endl;
                return SpeedState::STOPPED;
            }
            if (sr.signText == "STOP" && sr.distance <= 1) { 
                cout << "ACTION: Stopping for STOP sign (" << sr.objectId << ")" << endl;
                return SpeedState::STOPPED;
            }
        }
    }
    
    int dx = currentTarget.x - carPos.x;
    int dy = currentTarget.y - carPos.y;
    if (abs(dx) > abs(dy) && dx != 0) {
        requiredDir = (dx > 0) ? Direction::EAST : Direction::WEST;
    } else if (abs(dy) > 0) { 
        requiredDir = (dy > 0) ? Direction::SOUTH : Direction::NORTH;
    }

    if (requiredDir != carDir && requiredDir != Direction::UNKNOWN) {
        carDir = requiredDir; 
        cout << "ACTION: Turning to new direction: " << (int)requiredDir << endl;
        decision = SpeedState::HALF_SPEED; 
    }

    if (distToTarget <= 5 && carSpeed > SpeedState::HALF_SPEED) { 
        cout << "ACTION: Decelerating (close to target)." << endl;
        decision = SpeedState::HALF_SPEED;
    } else if (carSpeed < SpeedState::FULL_SPEED && distToTarget > 0) {
        cout << "ACTION: Accelerating." << endl;
        decision = (carSpeed == SpeedState::STOPPED) ? SpeedState::HALF_SPEED : SpeedState::FULL_SPEED; 
    }
    return decision;
}

float Sensor::calculateConfidence(int dist, float baseAcc, ObjectCategory type) const { 
    if (dist > range || dist < 0) return 0.0f; 
    float acc = getBaseConfidence(baseAcc, type);
    float distFactor = 1.0f - ((float)dist / range); 
    float confidence = acc * distFactor;
    uniform_real_distribution<> distNoise(-0.05, 0.05);
    confidence += distNoise(generator);
    return max(0.0f, min(1.0f, confidence));
}

LidarSensor::LidarSensor(int count) : Sensor("LIDAR", count, 0.99f, 9) { 
    cout << "[+LIDAR: " << id << "] Lidar sensor ready – Sensing with pew pews!" << endl; 
}

vector<SensorReading> LidarSensor::collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) {
    vector<SensorReading> readings;
    for (WorldObject* obj : worldObjects) {
        int dist = carPos.manhattanDistance(obj->getPosition());
        if (dist <= range && dist > 0) { 
            SensorReading sr = obj->getObjectState();
            sr.distance = dist;
            float acc = (sr.type != ObjectCategory::UNKNOWN) ? 0.87f : baseAccuracy; 
            sr.confidence = calculateConfidence(dist, acc, sr.type);
            if (sr.confidence > 0.01f) readings.push_back(sr);
        }
    }
    return readings;
}

RadarSensor::RadarSensor(int count) : Sensor("RADAR", count, 0.99f, 12) { 
    cout << "[+RADAR: " << id << "] Radar sensor ready - I'm a Radio star!" << endl; 
}

vector<SensorReading> RadarSensor::collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) {
    vector<SensorReading> readings;
    for (WorldObject* obj : worldObjects) {
        if (!dynamic_cast<MovingObject*>(obj)) continue;
        int dist = carPos.manhattanDistance(obj->getPosition());
        bool inFoV = false;
        int dx = obj->getPosition().x - carPos.x;
        int dy = obj->getPosition().y - carPos.y;
        if (dist <= range && dist > 0) {
            if (carDir == Direction::NORTH && dy < 0 && dx == 0) inFoV = true;
            else if (carDir == Direction::SOUTH && dy > 0 && dx == 0) inFoV = true;
            else if (carDir == Direction::EAST && dx > 0 && dy == 0) inFoV = true;
            else if (carDir == Direction::WEST && dx < 0 && dy == 0) inFoV = true;
        }
        if (inFoV) { 
            SensorReading sr = obj->getObjectState();
            sr.distance = dist;
            sr.confidence = calculateConfidence(dist, baseAccuracy, sr.type);
            if (sr.confidence > 0.01f) readings.push_back(sr);
        }
    }
    return readings;
}

CameraSensor::CameraSensor(int count) : Sensor("CAMERA", count, 0.87f, 7) { 
    cout << "[+CAMERA: " << id << "] Camera sensor ready - Say cheese!" << endl; 
}

vector<SensorReading> CameraSensor::collectReading(const Position& carPos, Direction carDir, const vector<WorldObject*>& worldObjects) {
    vector<SensorReading> readings;
    for (WorldObject* obj : worldObjects) {
        int dist = carPos.manhattanDistance(obj->getPosition());
        bool inFoV = false;
        int dx = obj->getPosition().x - carPos.x;
        int dy = obj->getPosition().y - carPos.y;
        if (dist <= range && dist > 0) {
            if (carDir == Direction::NORTH && dy < 0 && abs(dx) <= range/2) inFoV = true; 
            else if (carDir == Direction::SOUTH && dy > 0 && abs(dx) <= range/2) inFoV = true; 
            else if (carDir == Direction::EAST && dx > 0 && abs(dy) <= range/2) inFoV = true; 
            else if (carDir == Direction::WEST && dx < 0 && abs(dy) <= range/2) inFoV = true; 
        }
        if (inFoV) { 
            SensorReading sr = obj->getObjectState();
            sr.distance = dist;
            float acc = 0.95f; 
            sr.confidence = calculateConfidence(dist, acc, sr.type);
            if (sr.confidence > 0.01f) readings.push_back(sr);
        }
    }
    return readings;
}

SelfDrivingCar::SelfDrivingCar(int count, Position pos, Direction dir, const vector<Position>& targets, float threshold)
    : MovingObject("SELF_CAR", count, pos, '@', dir), navSystem(1, targets, threshold) { 
    sensors.push_back(new LidarSensor(1));
    sensors.push_back(new CameraSensor(2));
    sensors.push_back(new RadarSensor(3));
}

void SelfDrivingCar::performUpdate(int currentTick, const vector<WorldObject*>& worldObjects) {
    if (!navSystem.hasTargets()) {
        speed = SpeedState::STOPPED;
        return;
    }
    vector<vector<SensorReading>> allReadings;
    for (Sensor* s : sensors) {
        allReadings.push_back(s->collectReading(position, direction, worldObjects));
    }
    vector<SensorReading> fusedReadings = navSystem.syncNavigationSystem(allReadings);
    cout << "Sensor Fusion found " << fusedReadings.size() << " objects." << endl;
    SpeedState decision = navSystem.makeDecision(position, direction, speed, fusedReadings);
    speed = decision;
    MovingObject::update(currentTick); 
    cout << "Car moved to (" << position.x << ", " << position.y << ") at speed " << (int)speed << " facing " << (int)direction << endl;
}

void GridWorld::addObject(WorldObject* obj) { 
    if (SelfDrivingCar* sdc = dynamic_cast<SelfDrivingCar*>(obj)) {
        car = sdc;
        cout << "[+VEHICLE: " << sdc->getId() << "] Created at (" << sdc->getPosition().x << ", " << sdc->getPosition().y << "), heading " << (int)sdc->getDirection() << endl; 
    } else {
        objects.push_back(obj);
    }
}

bool GridWorld::updateAllObjects(int currentTick) {
    for (WorldObject* obj : objects) {
        obj->update(currentTick); 
    }
    if (car) {
        vector<WorldObject*> objectsForSensor = objects;
        car->performUpdate(currentTick, objectsForSensor);
        if (car->getPosition().x < 0 || car->getPosition().x >= dimX ||
            car->getPosition().y < 0 || car->getPosition().y >= dimY) {
            cout << "\nSIMULATION END: Self-Driving Car moved outside bounds at (" << car->getPosition().x << ", " << car->getPosition().y << ")!" << endl;
            return false; 
        }
        if (car->getSpeed() == SpeedState::STOPPED && !car->getNavSystem().hasTargets()) {
            cout << "\nSIMULATION END: All GPS targets reached!" << endl;
            return false; 
        }
    }
    for (auto it = objects.begin(); it != objects.end(); ) {
        const Position& pos = (*it)->getPosition();
        if (dynamic_cast<MovingObject*>(*it) && (pos.x < 0 || pos.x >= dimX || pos.y < 0 || pos.y >= dimY)) {
            cout << "VEHICLE: " << (*it)->getId() << " moved outside bounds and was removed." << endl;
            delete *it;
            it = objects.erase(it);
        } else {
            ++it;
        }
    }
    return true; 
}

char GridWorld::getGlyphAt(int x, int y) const {
    if (x < 0 || x >= dimX || y < 0 || y >= dimY) return 'X';
    if (car && car->getPosition().x == x && car->getPosition().y == y) return car->getGlyph();
    char highestPriorityGlyph = '.'; 
    for (WorldObject* obj : objects) {
        if (obj->getPosition().x == x && obj->getPosition().y == y) {
            char g = obj->getGlyph();
            if (g == 'R') return 'R';
            if (g == 'Y') return 'Y';
            if (g == 'S') highestPriorityGlyph = 'S';
            else if (g == 'B' && highestPriorityGlyph != 'S') highestPriorityGlyph = 'B';
            else if (g == 'C' && highestPriorityGlyph != 'S' && highestPriorityGlyph != 'B') highestPriorityGlyph = 'C';
            else if (g == 'G' && highestPriorityGlyph != 'S' && highestPriorityGlyph != 'B' && highestPriorityGlyph != 'C') highestPriorityGlyph = 'G';
            else if (g == 'P' && highestPriorityGlyph == '.') highestPriorityGlyph = 'P';
            else if (highestPriorityGlyph == '.') highestPriorityGlyph = '?'; 
        }
    }
    return highestPriorityGlyph;
}

void GridWorld::visualization_full() const {
    cout << "\n--- FULL WORLD VISUALIZATION (" << dimX << "x" << dimY << ") ---\n";
    for (int y = 0; y < dimY; ++y) {
        for (int x = 0; x < dimX; ++x) {
            cout << getGlyphAt(x, y) << " ";
        }
        cout << endl;
    }
    cout << "--------------------------------\n";
}

void GridWorld::visualization_pov(int radius, ViewType view) const { 
    if (!car) return;
    Position center = car->getPosition();
    int minX = center.x - radius;
    int maxX = center.x + radius;
    int minY = center.y - radius;
    int maxY = center.y + radius;
    
    cout << "\n--- POV VISUALIZATION (Radius: " << radius << ", View: " << (view == ViewType::CENTERED ? "CENTERED" : "FRONT") << ") ---\n";
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            cout << getGlyphAt(x, y) << " ";
        }
        cout << endl;
    }
    cout << "--------------------------------------------------------\n";
}
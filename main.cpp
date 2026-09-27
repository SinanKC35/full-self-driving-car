#include "Simulation.h"
#include <iostream>
#include <sstream>
#include <ctime>

using namespace std;

void printHelp() { 
    cout << "\nSelf-Driving Car Simulation" << endl;
    cout << "Usage: ./simulation [OPTIONS] --gps <x1> <y1> <x2> <y2>..." << endl;
    cout << "----------------------------------------------------------------------" << endl;
    cout << "--seed <N>               Random seed (default: current time)" << endl;
    cout << "--dimX <N>               World width (default: 40)" << endl;
    cout << "--dimY <N>               World height (default: 40)" << endl;
    cout << "--numMovingCars <N>      Number of moving cars (default: 3)" << endl;
    cout << "--numMovingBikes <N>     Number of moving bikes (default: 4)" << endl;
    cout << "--numParkedCars <N>      Number of parked cars (default: 5)" << endl;
    cout << "--numStopSigns <N>       Number of stop signs (default: 2)" << endl;
    cout << "--numTrafficLights <N>   Number of traffic lights (default: 2)" << endl;
    cout << "--simulationTicks <N>    Maximum simulation ticks (default: 100)" << endl;
    cout << "--minConfidenceThreshold <N> Minimum confidence cutoff (default: 40)" << endl;
    cout << "--gps <x y ...>          GPS target coordinates (required)" << endl;
    cout << "--help                   Show this help message" << endl << endl;
    cout << "Example usage: ./simulation --seed 12 --dimY 50 --gps 10 20 32 15" << endl;
}

map<string, string> parseArgs(int argc, char* argv[]) { 
    map<string, string> args;
    vector<string> gps_targets;
    bool gps_mode = false;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "--help") {
            printHelp();
            exit(0);
        } else if (arg == "--gps") { 
            gps_mode = true;
        } else if (arg.rfind("--", 0) == 0 && !gps_mode) {
            if (i + 1 < argc && argv[i+1][0] != '-') {
                args[arg.substr(2)] = argv[i+1];
                i++;
            }
        } else if (gps_mode) { 
            gps_targets.push_back(arg);
        }
    }
    
    if (!gps_targets.empty()) {
        stringstream ss;
        for (const auto& target : gps_targets) ss << target << " ";
        args["gps"] = ss.str();
    }
    return args;
}

int main(int argc, char* argv[]) {
    map<string, string> args = parseArgs(argc, argv);
    
    int seed = time(0); 
    int dimX = 40, dimY = 40;
    int simulationTicks = 100; 
    float confidenceThreshold = 40.0f; 
    int numMovingCars = 3, numMovingBikes = 4;
    int numParkedCars = 5, numStopSigns = 2, numTrafficLights = 2;

    if (args.count("seed")) seed = stoi(args["seed"]);
    if (args.count("dimX")) dimX = stoi(args["dimX"]);
    if (args.count("dimY")) dimY = stoi(args["dimY"]);
    if (args.count("simulationTicks")) simulationTicks = stoi(args["simulationTicks"]);
    if (args.count("minConfidenceThreshold")) confidenceThreshold = stof(args["minConfidenceThreshold"]);
    if (args.count("numMovingCars")) numMovingCars = stoi(args["numMovingCars"]);
    if (args.count("numMovingBikes")) numMovingBikes = stoi(args["numMovingBikes"]);
    if (args.count("numParkedCars")) numParkedCars = stoi(args["numParkedCars"]);
    if (args.count("numStopSigns")) numStopSigns = stoi(args["numStopSigns"]);
    if (args.count("numTrafficLights")) numTrafficLights = stoi(args["numTrafficLights"]);

    generator.seed(seed);

    if (!args.count("gps") || args["gps"].empty()) {
        cerr << "ERROR: GPS targets (--gps x1 y1 ...) are required." << endl; 
        printHelp();
        return 1;
    }

    vector<Position> gpsTargets;
    stringstream ss(args["gps"]);
    int x, y;
    while (ss >> x >> y) {
        gpsTargets.push_back({x, y});
    }

    cout << "\n\n*** SIMULATION STARTUP (SEED: " << seed << ") ***" << endl;

    GridWorld world(dimX, dimY);
    uniform_int_distribution<> distX(0, dimX - 1);
    uniform_int_distribution<> distY(0, dimY - 1);

    world.addObject(new SelfDrivingCar(1, {distX(generator), distY(generator)}, randomDirection(), gpsTargets, confidenceThreshold));
    for (int i = 0; i < numTrafficLights; ++i) world.addObject(new TrafficLight(i + 1, {distX(generator), distY(generator)}));
    for (int i = 0; i < numStopSigns; ++i) world.addObject(new StopSign(i + 1, {distX(generator), distY(generator)}));
    for (int i = 0; i < numParkedCars; ++i) world.addObject(new StationaryVehicle(i + 1, {distX(generator), distY(generator)}));
    for (int i = 0; i < numMovingCars; ++i) world.addObject(new Car(i + 1, {distX(generator), distY(generator)}, randomDirection()));
    for (int i = 0; i < numMovingBikes; ++i) world.addObject(new Bike(i + 1, {distX(generator), distY(generator)}, randomDirection()));

    world.visualization_full(); 
    const SelfDrivingCar* car = world.getCar();

    for (int tick = 0; tick < simulationTicks; ++tick) { 
        cout << "\n\n--- TICK " << tick + 1 << " / " << simulationTicks << " --- (Current Target: " << car->getNavSystem().getTarget().x << ", " << car->getNavSystem().getTarget().y << ")\n";
        if (!world.updateAllObjects(tick)) break; 
        world.visualization_pov(5, ViewType::CENTERED);
    }

    world.visualization_full(); 
    cout << "\n*** SIMULATION FINISHED ***" << endl;

    return 0;
}
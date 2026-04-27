/*
============================================================
  EMERGENCY VEHICLE SHORTEST PATH FINDER
  Object-Oriented C++ Project
  Uses Dijkstra's Algorithm on a weighted graph
============================================================

  CLASSES:
    - Location       : Represents a node/intersection
    - Road           : Represents a weighted edge
    - CityGraph      : Graph built from locations & roads
    - PathFinder     : Dijkstra's algorithm engine
    - Vehicle        : Emergency vehicle (Ambulance/FireTruck/Police/Private)
    - EmergencySystem: Top-level controller (interactive UI)

  FEATURES:
    - Add/remove locations and roads dynamically
    - Block roads (simulate traffic/closure)
    - Find shortest path for any vehicle
    - Display full city map
    - Multiple vehicle types with priorities
    - G63 AMG private vehicle at University Road
    - Full interactive console menu
============================================================
*/

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std;

// ─────────────────────────────────────────────
//  ANSI Color Codes
// ─────────────────────────────────────────────
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"
#define WHITE   "\033[37m"

// ─────────────────────────────────────────────
//  CLASS: Location
// ─────────────────────────────────────────────
class Location {
private:
    int id;
    string name;
    string type;

public:
    Location() : id(-1), name(""), type("intersection") {}
    Location(int id, const string& name, const string& type = "intersection")
        : id(id), name(name), type(type) {}

    int getId() const { return id; }
    string getName() const { return name; }
    string getType() const { return type; }

    void setName(const string& n) { name = n; }
    void setType(const string& t) { type = t; }

    string getTypeIcon() const {
        if (type == "hospital") return "[H]";
        if (type == "station")  return "[S]";
        if (type == "landmark") return "[L]";
        return "[X]";
    }

    void display() const {
        cout << CYAN << getTypeIcon() << RESET << " "
             << BOLD << name << RESET
             << " (ID:" << id << ", Type:" << type << ")";
    }
};

// ─────────────────────────────────────────────
//  CLASS: Road
// ─────────────────────────────────────────────
class Road {
private:
    int fromId, toId;
    double distance;
    bool blocked;
    string roadName;

public:
    Road(int from, int to, double dist, const string& name = "")
        : fromId(from), toId(to), distance(dist), blocked(false), roadName(name) {}

    int getFrom() const { return fromId; }
    int getTo()   const { return toId; }
    double getDistance() const { return distance; }
    bool isBlocked() const { return blocked; }
    string getRoadName() const { return roadName; }

    void block()   { blocked = true; }
    void unblock() { blocked = false; }
    void setDistance(double d) { distance = d; }

    void display() const {
        cout << "  Road: " << (roadName.empty() ? "Unnamed" : roadName)
             << " | From:" << fromId << " -> To:" << toId
             << " | Dist:" << distance << "km"
             << " | Status:" << (blocked ? RED " BLOCKED" RESET : GREEN " OPEN" RESET);
    }
};

// ─────────────────────────────────────────────
//  CLASS: CityGraph
// ─────────────────────────────────────────────
class CityGraph {
private:
    map<int, Location> locations;
    vector<Road> roads;
    int nextId;
    map<int, vector<pair<int, double> > > adjList;

    void rebuildAdjList() {
        adjList.clear();
        for (map<int,Location>::iterator it = locations.begin(); it != locations.end(); ++it)
            adjList[it->first] = vector<pair<int,double> >();
        for (int i = 0; i < (int)roads.size(); i++) {
            if (!roads[i].isBlocked()) {
                adjList[roads[i].getFrom()].push_back(make_pair(roads[i].getTo(), roads[i].getDistance()));
                adjList[roads[i].getTo()].push_back(make_pair(roads[i].getFrom(), roads[i].getDistance()));
            }
        }
    }

public:
    CityGraph() : nextId(1) {}

    int addLocation(const string& name, const string& type = "intersection") {
        int id = nextId++;
        locations[id] = Location(id, name, type);
        adjList[id] = vector<pair<int,double> >();
        cout << GREEN << "  [+] Location added: " << RESET << name << " (ID:" << id << ")\n";
        return id;
    }

    bool removeLocation(int id) {
        if (locations.find(id) == locations.end()) {
            cout << RED << "  [!] Location not found.\n" << RESET;
            return false;
        }
        locations.erase(id);
        vector<Road> remaining;
        for (int i = 0; i < (int)roads.size(); i++) {
            if (roads[i].getFrom() != id && roads[i].getTo() != id)
                remaining.push_back(roads[i]);
        }
        roads = remaining;
        rebuildAdjList();
        cout << GREEN << "  [-] Location removed: ID " << id << RESET << "\n";
        return true;
    }

    bool addRoad(int from, int to, double dist, const string& name = "") {
        if (locations.find(from) == locations.end() || locations.find(to) == locations.end()) {
            cout << RED << "  [!] One or both locations not found.\n" << RESET;
            return false;
        }
        roads.push_back(Road(from, to, dist, name));
        rebuildAdjList();
        cout << GREEN << "  [+] Road added: " << RESET
             << locations[from].getName() << " <-> " << locations[to].getName()
             << " (" << dist << " km)\n";
        return true;
    }

    bool setRoadBlocked(int from, int to, bool block) {
        for (int i = 0; i < (int)roads.size(); i++) {
            if ((roads[i].getFrom()==from && roads[i].getTo()==to) ||
                (roads[i].getFrom()==to   && roads[i].getTo()==from)) {
                block ? roads[i].block() : roads[i].unblock();
                rebuildAdjList();
                cout << YELLOW << "  [~] Road " << from << " <-> " << to
                     << (block ? " BLOCKED" : " UNBLOCKED") << RESET << "\n";
                return true;
            }
        }
        cout << RED << "  [!] Road not found.\n" << RESET;
        return false;
    }

    const map<int, Location>& getLocations() const { return locations; }
    const map<int, vector<pair<int,double> > >& getAdjList() const { return adjList; }
    const vector<Road>& getRoads() const { return roads; }

    bool locationExists(int id) const { return locations.find(id) != locations.end(); }

    string getLocationName(int id) const {
        map<int,Location>::const_iterator it = locations.find(id);
        return (it != locations.end()) ? it->second.getName() : "Unknown";
    }

    void displayMap() const {
        cout << "\n" << BOLD << BLUE << "=======================================\n";
        cout << "         CITY MAP\n";
        cout << "=======================================\n" << RESET;
        cout << BOLD << "LOCATIONS (" << locations.size() << "):\n" << RESET;
        for (map<int,Location>::const_iterator it = locations.begin(); it != locations.end(); ++it) {
            cout << "  ";
            it->second.display();
            cout << "\n";
        }
        cout << BOLD << "\nROADS (" << roads.size() << "):\n" << RESET;
        for (int i = 0; i < (int)roads.size(); i++) {
            roads[i].display();
            cout << "\n";
        }
        cout << BLUE << "=======================================\n" << RESET;
    }
};

// ─────────────────────────────────────────────
//  CLASS: PathFinder (Dijkstra's Algorithm)
// ─────────────────────────────────────────────
class PathFinder {
public:
    struct Result {
        bool found;
        double totalDistance;
        vector<int> path;
        vector<string> pathNames;
    };

    static Result dijkstra(const CityGraph& city, int startId, int endId) {
        Result res;
        res.found = false;
        res.totalDistance = 0;

        const map<int, vector<pair<int,double> > >& adj = city.getAdjList();
        const map<int, Location>& locs = city.getLocations();

        if (locs.find(startId) == locs.end() || locs.find(endId) == locs.end()) {
            cout << RED << "  [!] Invalid start or end location.\n" << RESET;
            return res;
        }

        map<int, double> dist;
        map<int, int> prev;
        priority_queue<pair<double,int>, vector<pair<double,int> >, greater<pair<double,int> > > pq;

        for (map<int,Location>::const_iterator it = locs.begin(); it != locs.end(); ++it) {
            dist[it->first] = numeric_limits<double>::infinity();
            prev[it->first] = -1;
        }

        dist[startId] = 0.0;
        pq.push(make_pair(0.0, startId));

        while (!pq.empty()) {
            double d = pq.top().first;
            int u    = pq.top().second;
            pq.pop();

            if (d > dist[u]) continue;
            if (u == endId) break;

            map<int, vector<pair<int,double> > >::const_iterator adjIt = adj.find(u);
            if (adjIt == adj.end()) continue;

            const vector<pair<int,double> >& neighbors = adjIt->second;
            for (int i = 0; i < (int)neighbors.size(); i++) {
                int v    = neighbors[i].first;
                double w = neighbors[i].second;
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    pq.push(make_pair(dist[v], v));
                }
            }
        }

        if (dist[endId] == numeric_limits<double>::infinity()) {
            return res;
        }

        vector<int> path;
        for (int at = endId; at != -1; at = prev[at])
            path.push_back(at);
        reverse(path.begin(), path.end());

        res.found = true;
        res.totalDistance = dist[endId];
        res.path = path;
        for (int i = 0; i < (int)path.size(); i++)
            res.pathNames.push_back(city.getLocationName(path[i]));

        return res;
    }
};

// ─────────────────────────────────────────────
//  CLASS: Vehicle
// ─────────────────────────────────────────────
class Vehicle {
public:
    enum Type { AMBULANCE, FIRE_TRUCK, POLICE, PRIVATE };

private:
    string name;
    Type type;
    int currentLocationId;
    double speedKmph;

public:
    Vehicle(const string& name, Type type, int startId, double speed)
        : name(name), type(type), currentLocationId(startId), speedKmph(speed) {}

    string getName() const { return name; }
    Type getType() const { return type; }
    int getCurrentLocation() const { return currentLocationId; }
    double getSpeed() const { return speedKmph; }
    bool isPrivate() const { return type == PRIVATE; }

    string getTypeStr() const {
        switch(type) {
            case AMBULANCE:  return "Ambulance";
            case FIRE_TRUCK: return "Fire Truck";
            case POLICE:     return "Police";
            case PRIVATE:    return "Private Vehicle";
        }
        return "Unknown";
    }

    string getIcon() const {
        switch(type) {
            case AMBULANCE:  return "[AMB]";
            case FIRE_TRUCK: return "[FIR]";
            case POLICE:     return "[POL]";
            case PRIVATE:    return "[PRV]";
        }
        return "[VEH]";
    }

    double estimateTime(double distKm) const {
        return (distKm / speedKmph) * 60.0;
    }

    void setCurrentLocation(int id) { currentLocationId = id; }

    void display() const {
        cout << BOLD << getIcon() << " " << name << RESET
             << " [" << getTypeStr() << "]"
             << " | Speed: " << speedKmph << " km/h"
             << " | At Location ID: " << currentLocationId;
        if (isPrivate()) cout << YELLOW << " [Private]" << RESET;
    }
};

// ─────────────────────────────────────────────
//  CLASS: EmergencySystem
// ─────────────────────────────────────────────
class EmergencySystem {
private:
    CityGraph city;
    vector<Vehicle> vehicles;

    void printBanner() {
        cout << BOLD << RED;
        cout << "\n+======================================================+\n";
        cout << "|   *** EMERGENCY VEHICLE SHORTEST PATH FINDER ***     |\n";
        cout << "|        C++ OOP Project | Dijkstra's Algorithm        |\n";
        cout << "+======================================================+\n";
        cout << RESET;
    }

    void printMenu() {
        cout << "\n" << BOLD << YELLOW << "============== MAIN MENU ==============\n" << RESET;
        cout << CYAN << " [1]" << RESET << " Find Shortest Path\n";
        cout << CYAN << " [2]" << RESET << " Display City Map\n";
        cout << CYAN << " [3]" << RESET << " Add Location\n";
        cout << CYAN << " [4]" << RESET << " Add Road\n";
        cout << CYAN << " [5]" << RESET << " Block / Unblock Road\n";
        cout << CYAN << " [6]" << RESET << " Add Emergency Vehicle\n";
        cout << CYAN << " [7]" << RESET << " Display Vehicles\n";
        cout << CYAN << " [8]" << RESET << " Remove Location\n";
        cout << CYAN << " [9]" << RESET << " Load Sample City\n";
        cout << RED  << " [0]" << RESET << " Exit\n";
        cout << YELLOW << "=======================================\n" << RESET;
        cout << "Enter choice: ";
    }

    void loadSampleCity() {
        cout << MAGENTA << "\n  Loading sample city...\n" << RESET;

        int h1 = city.addLocation("City Hospital",       "hospital");
        int s1 = city.addLocation("Fire Station Alpha",  "station");
        int s2 = city.addLocation("Police HQ",           "station");
        int a  = city.addLocation("Main Square",         "landmark");
        int b  = city.addLocation("North Junction",      "intersection");
        int c  = city.addLocation("East Market",         "intersection");
        int d  = city.addLocation("West Park",           "intersection");
        int e  = city.addLocation("South Bridge",        "intersection");
        int f  = city.addLocation("University Road",     "intersection");
        int g  = city.addLocation("Industrial Zone",     "intersection");

        city.addRoad(h1, a,  2.5, "Hospital Rd");
        city.addRoad(h1, b,  3.1, "North Ave");
        city.addRoad(s1, a,  1.8, "Station Blvd");
        city.addRoad(s1, d,  2.0, "Park Lane");
        city.addRoad(s2, e,  1.5, "South St");
        city.addRoad(s2, c,  2.2, "Market Link");
        city.addRoad(a,  b,  1.2, "Central Ave");
        city.addRoad(a,  c,  1.6, "East Rd");
        city.addRoad(a,  d,  1.4, "West Rd");
        city.addRoad(b,  f,  2.0, "Uni Connector");
        city.addRoad(c,  g,  3.0, "Industrial Hwy");
        city.addRoad(d,  e,  2.5, "Bridge Rd");
        city.addRoad(e,  g,  1.8, "South Industrial");
        city.addRoad(f,  g,  2.3, "Ring Road");
        city.addRoad(b,  c,  1.9, "Cross Junction");

        vehicles.push_back(Vehicle("Ambulance A1",  Vehicle::AMBULANCE,  h1, 80.0));
        vehicles.push_back(Vehicle("Fire Truck F1", Vehicle::FIRE_TRUCK, s1, 70.0));
        vehicles.push_back(Vehicle("Police Car P1", Vehicle::POLICE,     s2, 100.0));
        vehicles.push_back(Vehicle("G63 AMG",       Vehicle::PRIVATE,    f,  180.0));

        cout << GREEN  << "  [+] Sample city loaded successfully!\n" << RESET;
        cout << YELLOW << "  [*] G63 AMG (Private) added at University Road\n" << RESET;
    }

    void findShortestPath() {
        cout << "\n" << BOLD << "-- FIND SHORTEST PATH --\n" << RESET;
        city.displayMap();

        if (vehicles.empty()) {
            cout << RED << "  [!] No vehicles registered. Add one first.\n" << RESET;
            return;
        }

        cout << BOLD << "\nSelect Vehicle:\n" << RESET;
        for (int i = 0; i < (int)vehicles.size(); i++) {
            cout << "  [" << i+1 << "] ";
            vehicles[i].display();
            cout << "\n";
        }
        cout << "Enter vehicle number: ";
        int vIdx; cin >> vIdx;
        if (vIdx < 1 || vIdx > (int)vehicles.size()) {
            cout << RED << "  [!] Invalid choice.\n" << RESET;
            return;
        }
        Vehicle& v = vehicles[vIdx - 1];

        cout << "\nEnter DESTINATION Location ID: ";
        int destId; cin >> destId;

        if (!city.locationExists(destId)) {
            cout << RED << "  [!] Destination ID not found.\n" << RESET;
            return;
        }

        int startId = v.getCurrentLocation();
        cout << "\n" << YELLOW << "  Computing path...\n" << RESET;

        PathFinder::Result result = PathFinder::dijkstra(city, startId, destId);

        cout << "\n" << BOLD << RED
             << "+==========================================+\n"
             << "|           DISPATCH REPORT               |\n"
             << "+==========================================+\n" << RESET;

        cout << "  Vehicle : " << v.getIcon() << " " << BOLD << v.getName() << RESET;
        if (v.isPrivate()) cout << YELLOW << " [Private - No Emergency Priority]" << RESET;
        cout << "\n";
        cout << "  From    : " << CYAN << city.getLocationName(startId) << RESET << "\n";
        cout << "  To      : " << CYAN << city.getLocationName(destId)  << RESET << "\n";

        if (!result.found) {
            cout << RED << "  [X] NO PATH FOUND! Roads may be blocked.\n" << RESET;
        } else {
            cout << GREEN << "  [OK] PATH FOUND!\n\n" << RESET;
            cout << "  Route:\n  ";
            for (int i = 0; i < (int)result.pathNames.size(); i++) {
                if (i > 0) cout << YELLOW << " -> " << RESET;
                cout << BOLD << result.pathNames[i] << RESET;
            }
            cout << "\n\n";
            cout << fixed << setprecision(2);
            cout << "  Total Distance : " << GREEN << result.totalDistance << " km\n" << RESET;
            cout << "  Est. Time      : " << GREEN << v.estimateTime(result.totalDistance) << " min\n" << RESET;
            cout << "  Vehicle Speed  : " << v.getSpeed() << " km/h\n";
            cout << "  Stops          : " << result.path.size() << " locations\n";
        }
        cout << RED << "==========================================\n" << RESET;
    }

    void addLocation() {
        cin.ignore();
        cout << "\nEnter location name: ";
        string name; getline(cin, name);
        cout << "Enter type (intersection/hospital/station/landmark): ";
        string type; getline(cin, type);
        if (type.empty()) type = "intersection";
        city.addLocation(name, type);
    }

    void addRoad() {
        cout << "\nEnter FROM Location ID: ";
        int from; cin >> from;
        cout << "Enter TO Location ID: ";
        int to; cin >> to;
        cout << "Enter distance (km): ";
        double dist; cin >> dist;
        cin.ignore();
        cout << "Enter road name (optional): ";
        string name; getline(cin, name);
        city.addRoad(from, to, dist, name);
    }

    void blockUnblockRoad() {
        cout << "\nEnter FROM Location ID: ";
        int from; cin >> from;
        cout << "Enter TO Location ID: ";
        int to; cin >> to;
        cout << "Block (1) or Unblock (0): ";
        int choice; cin >> choice;
        city.setRoadBlocked(from, to, choice == 1);
    }

    void addVehicle() {
        cin.ignore();
        cout << "\nEnter vehicle name: ";
        string name; getline(cin, name);
        cout << "Type (1=Ambulance, 2=FireTruck, 3=Police, 4=Private): ";
        int t; cin >> t;
        Vehicle::Type type;
        if      (t == 2) type = Vehicle::FIRE_TRUCK;
        else if (t == 3) type = Vehicle::POLICE;
        else if (t == 4) type = Vehicle::PRIVATE;
        else             type = Vehicle::AMBULANCE;

        cout << "Enter starting Location ID: ";
        int locId; cin >> locId;
        if (!city.locationExists(locId)) {
            cout << RED << "  [!] Location not found.\n" << RESET;
            return;
        }
        double speed;
        if      (type == Vehicle::FIRE_TRUCK) speed = 70.0;
        else if (type == Vehicle::POLICE)     speed = 100.0;
        else if (type == Vehicle::PRIVATE)    speed = 180.0;
        else                                  speed = 80.0;

        vehicles.push_back(Vehicle(name, type, locId, speed));
        cout << GREEN << "  [+] Vehicle added!\n" << RESET;
    }

    void displayVehicles() {
        cout << "\n" << BOLD << "-- REGISTERED VEHICLES --\n" << RESET;
        if (vehicles.empty()) {
            cout << "  No vehicles registered.\n";
            return;
        }
        for (int i = 0; i < (int)vehicles.size(); i++) {
            cout << "  ";
            vehicles[i].display();
            cout << " | At: " << CYAN << city.getLocationName(vehicles[i].getCurrentLocation()) << RESET << "\n";
        }
    }

    void removeLocation() {
        cout << "\nEnter Location ID to remove: ";
        int id; cin >> id;
        city.removeLocation(id);
    }

public:
    void run() {
        printBanner();
        cout << "\n" << YELLOW << "  TIP: Start with option [9] to load a sample city!\n" << RESET;

        int choice = -1;
        while (choice != 0) {
            printMenu();
            if (!(cin >> choice)) {
                cin.clear();
                cin.ignore(1000, '\n');
                continue;
            }
            switch(choice) {
                case 1: findShortestPath(); break;
                case 2: city.displayMap();  break;
                case 3: addLocation();      break;
                case 4: addRoad();          break;
                case 5: blockUnblockRoad(); break;
                case 6: addVehicle();       break;
                case 7: displayVehicles();  break;
                case 8: removeLocation();   break;
                case 9: loadSampleCity();   break;
                case 0:
                    cout << BOLD << RED << "\n  *** Emergency System Shutting Down. Stay safe! ***\n" << RESET;
                    break;
                default:
                    cout << RED << "  [!] Invalid option. Try again.\n" << RESET;
            }
        }
    }
};

// ─────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────
int main() {
    EmergencySystem system;
    system.run();
    return 0;
}
//PS C:\oops project> 
//PS C:\oops project> .\emergency.exe
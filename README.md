# 🚨 Emergency Vehicle Shortest Path Finder

A fully Object-Oriented C++ project that finds the **shortest path** for emergency
vehicles navigating through a city road network using **Dijkstra's Algorithm**.

## 🚗 Vehicles
- 🚑 Ambulance A1 — 80 km/h — City Hospital
- 🚒 Fire Truck F1 — 70 km/h — Fire Station Alpha
- 🚓 Police Car P1 — 100 km/h — Police HQ
- 🚗 G63 AMG (Private) — 180 km/h — University Road

## 🧠 OOP Classes
| Class | Purpose |
|---|---|
| `Location` | City node (hospital, station, intersection, landmark) |
| `Road` | Weighted bidirectional road edge |
| `CityGraph` | Graph management + adjacency list |
| `PathFinder` | Dijkstra's algorithm engine |
| `Vehicle` | Emergency + private vehicle types |
| `EmergencySystem` | Interactive controller |

## ⚙️ Features
- Dijkstra's shortest path algorithm
- Dynamic road blocking and unblocking
- Add/remove locations and roads at runtime
- ETA calculation based on vehicle speed
- Console-based interactive menu
- Live graphical web interface (HTML5 Canvas)
- C++ HTTP server using WinSock

## 🖥️ How to Run

### Console Version
```bash
g++ -o emergency shortest_path_finder.cpp -std=c++11
.\emergency.exe
```
Press `9` to load sample city, then `1` to find shortest paths.

### Graphical Web Interface
```bash
g++ -o server server.cpp -std=c++11 -lws2_32
.\server.exe
```
Open browser at: **http://localhost:8080**

## 📊 Complexity
- Time: O((V + E) log V)
- Space: O(V + E)

## 🛠️ Tech Stack
- C++ (OOP, STL, WinSock2)
- HTML5 Canvas + JavaScript
- Dijkstra's Algorithm
- Custom HTTP Server (no external libraries)

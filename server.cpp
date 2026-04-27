/*
============================================================
  EMERGENCY VEHICLE SHORTEST PATH FINDER
  Pure WinSock HTTP Server - No external libraries needed
 
  COMPILE:
    g++ -o server server.cpp -std=c++11 -lws2_32
 
  RUN:
    .\server.exe
 
  OPEN BROWSER:
    http://localhost:8080
============================================================
*/
 
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>
#include <iomanip>
 
#pragma comment(lib, "ws2_32.lib")
using namespace std;
 
#define RESET  "\033[0m"
#define RED    "\033[31m"
#define GREEN  "\033[32m"
#define YELLOW "\033[33m"
#define CYAN   "\033[36m"
#define BOLD   "\033[1m"
 
// ─────────────────────────────────────────────
//  CLASS: Location
// ─────────────────────────────────────────────
class Location {
public:
    int id;
    string name, type;
    double x, y;
    Location() : id(-1), name(""), type("intersection"), x(0), y(0) {}
    Location(int id, const string& n, const string& t, double x, double y)
        : id(id), name(n), type(t), x(x), y(y) {}
};
 
// ─────────────────────────────────────────────
//  CLASS: Road
// ─────────────────────────────────────────────
class Road {
public:
    int from, to;
    double dist;
    string name;
    bool blocked;
    Road(int f, int t, double d, const string& n)
        : from(f), to(t), dist(d), name(n), blocked(false) {}
};
 
// ─────────────────────────────────────────────
//  CLASS: Vehicle
// ─────────────────────────────────────────────
class Vehicle {
public:
    int id;
    string name, type;
    int locId;
    double speed;
    bool isPrivate;
    Vehicle(int id, const string& n, const string& t, int loc, double sp)
        : id(id), name(n), type(t), locId(loc), speed(sp), isPrivate(t == "private") {}
};
 
// ─────────────────────────────────────────────
//  CLASS: CityGraph
// ─────────────────────────────────────────────
class CityGraph {
public:
    map<int, Location> locations;
    vector<Road> roads;
    vector<Vehicle> vehicles;
    int nextLocId, nextVehId;
 
    CityGraph() : nextLocId(1), nextVehId(1) {}
 
    int addLocation(const string& name, const string& type, double x = 0, double y = 0) {
        int id = nextLocId++;
        locations[id] = Location(id, name, type, x, y);
        cout << GREEN << "  [+] Location: " << name << " (ID:" << id << ")\n" << RESET;
        return id;
    }
 
    bool removeLocation(int id) {
        if (locations.find(id) == locations.end()) return false;
        locations.erase(id);
        vector<Road> kept;
        for (int i = 0; i < (int)roads.size(); i++)
            if (roads[i].from != id && roads[i].to != id)
                kept.push_back(roads[i]);
        roads = kept;
        cout << RED << "  [-] Location removed: " << id << "\n" << RESET;
        return true;
    }
 
    bool addRoad(int from, int to, double dist, const string& name) {
        if (locations.find(from) == locations.end() || locations.find(to) == locations.end())
            return false;
        roads.push_back(Road(from, to, dist, name));
        cout << GREEN << "  [+] Road: " << locations[from].name << " <-> " << locations[to].name
             << " (" << dist << "km)\n" << RESET;
        return true;
    }
 
    bool blockRoad(int from, int to, bool block) {
        for (int i = 0; i < (int)roads.size(); i++) {
            if ((roads[i].from == from && roads[i].to == to) ||
                (roads[i].from == to   && roads[i].to == from)) {
                roads[i].blocked = block;
                cout << YELLOW << "  [~] Road " << from << "<->" << to
                     << (block ? " BLOCKED" : " UNBLOCKED") << "\n" << RESET;
                return true;
            }
        }
        return false;
    }
 
    int addVehicle(const string& name, const string& type, int locId, double speed) {
        int id = nextVehId++;
        vehicles.push_back(Vehicle(id, name, type, locId, speed));
        cout << GREEN << "  [+] Vehicle: " << name << " [" << type << "] at "
             << locations[locId].name << "\n" << RESET;
        return id;
    }
 
    struct PathResult {
        bool found;
        double dist;
        vector<int> path;
    };
 
    PathResult dijkstra(int startId, int endId) {
        PathResult res; res.found = false; res.dist = 0;
        map<int, double> dist;
        map<int, int> prev;
        map<int, vector<pair<int, double> > > adj;
 
        for (map<int,Location>::iterator it = locations.begin(); it != locations.end(); ++it) {
            dist[it->first] = numeric_limits<double>::infinity();
            prev[it->first] = -1;
            adj[it->first]  = vector<pair<int,double> >();
        }
        for (int i = 0; i < (int)roads.size(); i++) {
            if (!roads[i].blocked) {
                adj[roads[i].from].push_back(make_pair(roads[i].to,   roads[i].dist));
                adj[roads[i].to  ].push_back(make_pair(roads[i].from, roads[i].dist));
            }
        }
        dist[startId] = 0.0;
        priority_queue<pair<double,int>, vector<pair<double,int> >, greater<pair<double,int> > > pq;
        pq.push(make_pair(0.0, startId));
 
        while (!pq.empty()) {
            double d = pq.top().first;
            int u    = pq.top().second;
            pq.pop();
            if (d > dist[u]) continue;
            if (u == endId) break;
            vector<pair<int,double> >& nbrs = adj[u];
            for (int i = 0; i < (int)nbrs.size(); i++) {
                int v    = nbrs[i].first;
                double w = nbrs[i].second;
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    pq.push(make_pair(dist[v], v));
                }
            }
        }
        if (dist[endId] == numeric_limits<double>::infinity()) return res;
        vector<int> path;
        for (int at = endId; at != -1; at = prev[at]) path.push_back(at);
        reverse(path.begin(), path.end());
        res.found = true; res.dist = dist[endId]; res.path = path;
        return res;
    }
 
    void loadSample() {
        locations.clear(); roads.clear(); vehicles.clear();
        nextLocId = 1; nextVehId = 1;
 
        int h1 = addLocation("City Hospital",      "hospital",     400, 80 );
        int s1 = addLocation("Fire Station Alpha",  "station",      120, 200);
        int s2 = addLocation("Police HQ",           "station",      680, 200);
        int a  = addLocation("Main Square",         "landmark",     400, 280);
        int b  = addLocation("North Junction",      "intersection", 240, 190);
        int c  = addLocation("East Market",         "intersection", 570, 340);
        int d  = addLocation("West Park",           "intersection", 180, 340);
        int e  = addLocation("South Bridge",        "intersection", 400, 460);
        int f  = addLocation("University Road",     "intersection", 230, 410);
        int g  = addLocation("Industrial Zone",     "intersection", 560, 460);
 
        addRoad(h1, a, 2.5, "Hospital Rd");
        addRoad(h1, b, 3.1, "North Ave");
        addRoad(s1, a, 1.8, "Station Blvd");
        addRoad(s1, d, 2.0, "Park Lane");
        addRoad(s2, e, 1.5, "South St");
        addRoad(s2, c, 2.2, "Market Link");
        addRoad(a,  b, 1.2, "Central Ave");
        addRoad(a,  c, 1.6, "East Rd");
        addRoad(a,  d, 1.4, "West Rd");
        addRoad(b,  f, 2.0, "Uni Connector");
        addRoad(c,  g, 3.0, "Industrial Hwy");
        addRoad(d,  e, 2.5, "Bridge Rd");
        addRoad(e,  g, 1.8, "South Industrial");
        addRoad(f,  g, 2.3, "Ring Road");
        addRoad(b,  c, 1.9, "Cross Junction");
 
        addVehicle("Ambulance A1",  "ambulance", h1, 80.0 );
        addVehicle("Fire Truck F1", "firetruck", s1, 70.0 );
        addVehicle("Police Car P1", "police",    s2, 100.0);
        addVehicle("G63 AMG",       "private",   f,  180.0);
 
        cout << BOLD << GREEN << "\n  [OK] Sample city loaded!\n" << RESET;
    }
};
 
// ─────────────────────────────────────────────
//  JSON helpers
// ─────────────────────────────────────────────
string esc(const string& s) {
    string r;
    for (int i = 0; i < (int)s.size(); i++) {
        if (s[i] == '"') r += "\\\"";
        else r += s[i];
    }
    return r;
}
 
string cityToJson(CityGraph& g) {
    ostringstream o;
    o << "{\"locations\":[";
    bool first = true;
    for (map<int,Location>::iterator it = g.locations.begin(); it != g.locations.end(); ++it) {
        Location& l = it->second;
        if (!first) o << ",";
        o << "{\"id\":" << l.id
          << ",\"name\":\"" << esc(l.name) << "\""
          << ",\"type\":\"" << l.type << "\""
          << ",\"x\":" << l.x << ",\"y\":" << l.y << "}";
        first = false;
    }
    o << "],\"roads\":[";
    for (int i = 0; i < (int)g.roads.size(); i++) {
        if (i) o << ",";
        o << "{\"from\":" << g.roads[i].from
          << ",\"to\":" << g.roads[i].to
          << ",\"dist\":" << g.roads[i].dist
          << ",\"name\":\"" << esc(g.roads[i].name) << "\""
          << ",\"blocked\":" << (g.roads[i].blocked ? "true" : "false") << "}";
    }
    o << "],\"vehicles\":[";
    for (int i = 0; i < (int)g.vehicles.size(); i++) {
        if (i) o << ",";
        o << "{\"id\":" << g.vehicles[i].id
          << ",\"name\":\"" << esc(g.vehicles[i].name) << "\""
          << ",\"type\":\"" << g.vehicles[i].type << "\""
          << ",\"locId\":" << g.vehicles[i].locId
          << ",\"speed\":" << g.vehicles[i].speed
          << ",\"isPrivate\":" << (g.vehicles[i].isPrivate ? "true" : "false") << "}";
    }
    o << "]}";
    return o.str();
}
 
// ─────────────────────────────────────────────
//  URL decode & query parser
// ─────────────────────────────────────────────
string urlDecode(const string& s) {
    string r;
    for (int i = 0; i < (int)s.size(); i++) {
        if (s[i] == '%' && i + 2 < (int)s.size()) {
            int v = 0;
            sscanf(s.substr(i+1,2).c_str(), "%x", &v);
            r += (char)v; i += 2;
        } else if (s[i] == '+') r += ' ';
        else r += s[i];
    }
    return r;
}
 
map<string,string> parseQuery(const string& q) {
    map<string,string> m;
    string key, val;
    bool inVal = false;
    for (int i = 0; i <= (int)q.size(); i++) {
        char c = (i < (int)q.size()) ? q[i] : '&';
        if (c == '=') { inVal = true; }
        else if (c == '&') {
            if (!key.empty()) m[urlDecode(key)] = urlDecode(val);
            key.clear(); val.clear(); inVal = false;
        } else { if (inVal) val += c; else key += c; }
    }
    return m;
}
 
string getP(map<string,string>& p, const string& k, const string& def = "") {
    map<string,string>::iterator it = p.find(k);
    return (it != p.end()) ? it->second : def;
}
 
// ─────────────────────────────────────────────
//  HTML page (embedded in C++)
// ─────────────────────────────────────────────
const string HTML_PAGE = "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"UTF-8\">\n<title>Emergency Path Finder</title>\n<style>\n*{box-sizing:border-box;margin:0;padding:0}\nbody{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;display:flex;flex-direction:column;height:100vh}\n.topbar{background:#1a1d27;border-bottom:1px solid #2a2d3a;padding:10px 18px;display:flex;align-items:center;gap:10px}\n.siren{width:12px;height:12px;border-radius:50%;animation:pulse 1s infinite}\n.sr{background:#e24b4a}.sb{background:#378add;animation-delay:.5s}\n@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}\n.topbar h1{font-size:15px;font-weight:600;color:#fff}\n.sub{font-size:11px;color:#555}\n.layout{display:grid;grid-template-columns:270px 1fr;flex:1;overflow:hidden}\n.sidebar{background:#1a1d27;border-right:1px solid #2a2d3a;display:flex;flex-direction:column;overflow:hidden}\n.tabs{display:flex;border-bottom:1px solid #2a2d3a}\n.tab{flex:1;padding:9px 2px;font-size:11px;text-align:center;color:#555;cursor:pointer;border-bottom:2px solid transparent;transition:all .15s}\n.tab.active{color:#fff;border-bottom-color:#e24b4a}\n.panel{display:none;flex-direction:column;flex:1;overflow-y:auto;padding:10px}\n.panel.active{display:flex}\n.lbl{font-size:10px;text-transform:uppercase;letter-spacing:.07em;color:#555;margin-bottom:5px;margin-top:10px}\n.lbl:first-child{margin-top:0}\ninput,select{width:100%;background:#0f1117;border:1px solid #2a2d3a;color:#e0e0e0;border-radius:5px;padding:6px 9px;font-size:12px;outline:none;margin-bottom:5px}\ninput:focus,select:focus{border-color:#e24b4a}\n.r2{display:grid;grid-template-columns:1fr 1fr;gap:5px}\nbutton{width:100%;padding:7px;border-radius:5px;border:none;font-size:12px;cursor:pointer;font-weight:500;margin-bottom:5px;transition:opacity .15s}\nbutton:hover{opacity:.85}\n.bred{background:#e24b4a;color:#fff}.bblue{background:#378add;color:#fff}\n.bgreen{background:#1d9e75;color:#fff}.bgray{background:#2a2d3a;color:#ccc}\n.log{font-family:Consolas,monospace;font-size:11px;background:#0f1117;border:1px solid #2a2d3a;border-radius:5px;padding:8px;max-height:130px;overflow-y:auto;margin-bottom:5px}\n.lok{color:#1d9e75}.lerr{color:#e24b4a}.linfo{color:#378add}.lwarn{color:#ef9f27}\n.vcard{background:#0f1117;border:1px solid #2a2d3a;border-radius:7px;padding:8px 10px;margin-bottom:5px;cursor:pointer;transition:border-color .15s}\n.vcard:hover{border-color:#444}.vcard.sel{border-color:#e24b4a}.vcard.prv{border-color:#ef9f2766}\n.vn{font-size:13px;font-weight:600;color:#fff}.vs{font-size:11px;color:#666;margin-top:2px}\n.badge{display:inline-block;font-size:10px;padding:1px 6px;border-radius:99px;margin-left:5px}\n.ba{background:#e24b4a22;color:#e24b4a;border:1px solid #e24b4a44}\n.bf{background:#d85a3022;color:#d85a30;border:1px solid #d85a3044}\n.bp{background:#378add22;color:#378add;border:1px solid #378add44}\n.bpr{background:#ef9f2722;color:#ef9f27;border:1px solid #ef9f2744}\n.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin-bottom:8px}\n.metric{background:#0f1117;border-radius:5px;padding:8px;text-align:center}\n.mval{font-size:18px;font-weight:600;color:#fff}.mlbl{font-size:10px;color:#555;margin-top:1px}\n.rbox{background:#1a1d27;border:1px solid #2a2d3a;border-radius:7px;padding:10px;margin-bottom:6px}\n.rbox.found{border-color:#1d9e75}.rbox.nf{border-color:#e24b4a}\n.rtitle{font-size:10px;text-transform:uppercase;letter-spacing:.07em;color:#555;margin-bottom:7px}\n.route{display:flex;flex-wrap:wrap;gap:3px;align-items:center}\n.rnode{font-size:11px;padding:2px 8px;border-radius:99px;background:#378add22;color:#378add;border:1px solid #378add44}\n.rnode.p{background:#ef9f2722;color:#ef9f27;border-color:#ef9f2744}\n.rarr{color:#444;font-size:12px}\n.liitem{background:#0f1117;border:1px solid #2a2d3a;border-radius:5px;padding:6px 9px;margin-bottom:4px;display:flex;align-items:center;gap:7px;font-size:12px}\n.ldot{width:9px;height:9px;border-radius:50%;flex-shrink:0}\n.lname{flex:1;color:#ddd}.lid{color:#555;font-size:10px}\n.lrm{background:none;border:none;color:#e24b4a;cursor:pointer;font-size:13px;width:auto;padding:0 3px;margin:0}\n.ritem{background:#0f1117;border:1px solid #2a2d3a;border-radius:5px;padding:6px 9px;margin-bottom:4px;font-size:11px;display:flex;align-items:center;gap:5px}\n.ritem.blk{opacity:.4;text-decoration:line-through}\n.rnm{flex:1;color:#bbb;font-weight:500}.rkm{color:#555}\n.rbtn{background:#2a2d3a;border:none;color:#aaa;cursor:pointer;font-size:10px;padding:2px 6px;border-radius:3px;width:auto;margin:0}\n.main{position:relative;overflow:hidden;background:#0f1117}\ncanvas{display:block}\n.legend{position:absolute;bottom:14px;left:14px;background:#1a1d27cc;border:1px solid #2a2d3a;border-radius:7px;padding:9px 12px;font-size:11px}\n.lr{display:flex;align-items:center;gap:6px;margin-bottom:4px}.lr:last-child{margin-bottom:0}\n.ldott{width:9px;height:9px;border-radius:50%}\n.mctrl{position:absolute;top:14px;right:14px;display:flex;gap:5px}\n.mbtn{background:#1a1d27cc;border:1px solid #2a2d3a;color:#aaa;border-radius:5px;padding:5px 11px;font-size:12px;cursor:pointer;width:auto;margin:0}\n.mbtn:hover{color:#fff;border-color:#444}\n#tip{position:absolute;background:#1a1d27;border:1px solid #2a2d3a;border-radius:7px;padding:7px 11px;font-size:12px;pointer-events:none;display:none;z-index:10;color:#e0e0e0;white-space:nowrap}\n::-webkit-scrollbar{width:3px}::-webkit-scrollbar-thumb{background:#2a2d3a;border-radius:2px}\n</style>\n</head>\n<body>\n<div class=\"topbar\"><div class=\"siren sr\"></div><div class=\"siren sb\"></div><h1>Emergency Vehicle Shortest Path Finder</h1><span class=\"sub\">&nbsp;&mdash;&nbsp;C++ OOP &mdash; Dijkstra &mdash; Live C++ Backend</span></div>\n<div class=\"layout\">\n<div class=\"sidebar\">\n<div class=\"tabs\"><div class=\"tab active\" onclick=\"switchTab('dispatch')\">Dispatch</div><div class=\"tab\" onclick=\"switchTab('locs')\">Locations</div><div class=\"tab\" onclick=\"switchTab('roads')\">Roads</div><div class=\"tab\" onclick=\"switchTab('vehs')\">Vehicles</div></div>\n<div id=\"p-dispatch\" class=\"panel active\">\n<div class=\"lbl\">Select Vehicle</div><div id=\"vcards\"></div>\n<div class=\"lbl\">Destination</div><select id=\"sdest\"></select>\n<button class=\"bred\" onclick=\"findPath()\">&#9654; Find Shortest Path</button>\n<button class=\"bgray\" onclick=\"apiLoadSample()\">&#8635; Reload Sample City</button>\n<div class=\"lbl\">Result</div><div id=\"result\"></div>\n<div class=\"lbl\">Terminal Log (mirrored from C++)</div><div id=\"log\" class=\"log\"></div>\n</div>\n<div id=\"p-locs\" class=\"panel\">\n<div class=\"lbl\">Add Location</div>\n<input id=\"lname\" placeholder=\"Location name\">\n<select id=\"ltype\"><option value=\"intersection\">Intersection</option><option value=\"hospital\">Hospital</option><option value=\"station\">Station</option><option value=\"landmark\">Landmark</option></select>\n<button class=\"bgreen\" onclick=\"apiAddLoc()\">+ Add Location</button>\n<div class=\"lbl\">All Locations</div><div id=\"llist\"></div>\n</div>\n<div id=\"p-roads\" class=\"panel\">\n<div class=\"lbl\">Add Road</div>\n<div class=\"r2\"><select id=\"rfrom\"></select><select id=\"rto\"></select></div>\n<div class=\"r2\"><input id=\"rdist\" type=\"number\" value=\"2.0\" min=\"0.1\" step=\"0.1\" placeholder=\"km\"><input id=\"rname\" placeholder=\"Road name\"></div>\n<button class=\"bblue\" onclick=\"apiAddRoad()\">+ Add Road</button>\n<div class=\"lbl\">All Roads</div><div id=\"rlist\"></div>\n</div>\n<div id=\"p-vehs\" class=\"panel\">\n<div class=\"lbl\">Add Vehicle</div>\n<input id=\"vname\" placeholder=\"Vehicle name\">\n<div class=\"r2\"><select id=\"vtype\"><option value=\"ambulance\">Ambulance</option><option value=\"firetruck\">Fire Truck</option><option value=\"police\">Police</option><option value=\"private\">Private</option></select><select id=\"vloc\"></select></div>\n<button class=\"bgreen\" onclick=\"apiAddVeh()\">+ Add Vehicle</button>\n<div class=\"lbl\">All Vehicles</div><div id=\"avlist\"></div>\n</div>\n</div>\n<div class=\"main\" id=\"main\">\n<canvas id=\"c\"></canvas>\n<div class=\"mctrl\"><button class=\"mbtn\" onclick=\"zoomIn()\">+</button><button class=\"mbtn\" onclick=\"zoomOut()\">&#8722;</button><button class=\"mbtn\" onclick=\"resetView()\">Reset</button></div>\n<div class=\"legend\"><div class=\"lr\"><div class=\"ldott\" style=\"background:#e24b4a\"></div>Hospital</div><div class=\"lr\"><div class=\"ldott\" style=\"background:#ef9f27\"></div>Station</div><div class=\"lr\"><div class=\"ldott\" style=\"background:#1d9e75\"></div>Landmark</div><div class=\"lr\"><div class=\"ldott\" style=\"background:#378add\"></div>Intersection</div><div class=\"lr\" style=\"color:#e24b4a;font-size:10px\">&#9135;&#9135; Shortest Path</div><div class=\"lr\" style=\"color:#555;font-size:10px\">- - - Blocked</div><div style=\"color:#555;font-size:10px;margin-top:5px\">Drag nodes &bull; Scroll to zoom</div></div>\n<div id=\"tip\"></div>\n</div>\n</div>\n<script>\nvar state={locations:[],roads:[],vehicles:[]};var positions={};var hlPath=[];var selVeh=-1;\nvar zoom=1,panX=0,panY=0,dragNode=null,dragCanvas=false,lastMX=0,lastMY=0;\nvar canvas=document.getElementById('c');var ctx=canvas.getContext('2d');\nfunction W(){return canvas.width;}function H(){return canvas.height;}\nfunction resizeCanvas(){var m=document.getElementById('main');canvas.width=m.offsetWidth;canvas.height=m.offsetHeight;draw();}\nwindow.addEventListener('resize',resizeCanvas);\nfunction nodeColor(t){var c={hospital:'#e24b4a',station:'#ef9f27',landmark:'#1d9e75',intersection:'#378add'};return c[t]||'#888';}\nfunction s2w(x,y){return{x:(x-panX)/zoom,y:(y-panY)/zoom};}\nfunction ensurePos(){for(var i=0;i<state.locations.length;i++){var l=state.locations[i];if(!positions[l.id]){if(l.x&&l.y){positions[l.id]={x:l.x,y:l.y};}else{var cx=W()/2,cy=H()/2,r=Math.min(W(),H())*0.35;var angle=(2*Math.PI*i/state.locations.length)-Math.PI/2;positions[l.id]={x:(cx+r*Math.cos(angle)-panX)/zoom,y:(cy+r*Math.sin(angle)-panY)/zoom};}}}}\nfunction draw(){ctx.clearRect(0,0,W(),H());ensurePos();ctx.save();ctx.translate(panX,panY);ctx.scale(zoom,zoom);for(var i=0;i<state.roads.length;i++){var r=state.roads[i];var p1=positions[r.from],p2=positions[r.to];if(!p1||!p2)continue;var onP=false;for(var k=0;k<hlPath.length-1;k++){if((hlPath[k]==r.from&&hlPath[k+1]==r.to)||(hlPath[k]==r.to&&hlPath[k+1]==r.from)){onP=true;break;}}ctx.beginPath();ctx.moveTo(p1.x,p1.y);ctx.lineTo(p2.x,p2.y);if(r.blocked){ctx.setLineDash([6,5]);ctx.strokeStyle='#2a1a1a';ctx.lineWidth=2/zoom;}else if(onP){ctx.setLineDash([]);ctx.strokeStyle='#e24b4a';ctx.lineWidth=4.5/zoom;}else{ctx.setLineDash([]);ctx.strokeStyle='#2a2d3a';ctx.lineWidth=2/zoom;}ctx.stroke();ctx.setLineDash([]);var mx=(p1.x+p2.x)/2,my=(p1.y+p2.y)/2;ctx.fillStyle='#444';ctx.font=(10/zoom)+'px Segoe UI';ctx.textAlign='center';ctx.fillText(r.dist.toFixed(1)+'km',mx,my-5/zoom);}for(var i=0;i<state.locations.length;i++){var l=state.locations[i];var pos=positions[l.id];if(!pos)continue;var onP=hlPath.indexOf(l.id)>=0||hlPath.indexOf(String(l.id))>=0;var rad=(onP?17:12)/zoom;if(onP){ctx.beginPath();ctx.arc(pos.x,pos.y,rad+5/zoom,0,Math.PI*2);ctx.fillStyle=nodeColor(l.type)+'33';ctx.fill();}ctx.beginPath();ctx.arc(pos.x,pos.y,rad,0,Math.PI*2);ctx.fillStyle=nodeColor(l.type);ctx.fill();if(onP){ctx.strokeStyle='#fff';ctx.lineWidth=2/zoom;ctx.stroke();}ctx.fillStyle='#fff';ctx.font='bold '+(9/zoom)+'px Segoe UI';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(l.id,pos.x,pos.y);ctx.fillStyle='#bbb';ctx.font=(10/zoom)+'px Segoe UI';ctx.textBaseline='alphabetic';var lab=l.name.length>15?l.name.slice(0,14)+'...':l.name;ctx.fillText(lab,pos.x,pos.y+rad+12/zoom);}var vc={ambulance:'#e24b4a',firetruck:'#d85a30',police:'#378add',private:'#ef9f27'};var vi={ambulance:'A',firetruck:'F',police:'P',private:'G'};for(var i=0;i<state.vehicles.length;i++){var v=state.vehicles[i];var pos=positions[v.locId];if(!pos)continue;var vr=8/zoom;ctx.beginPath();ctx.arc(pos.x-16/zoom,pos.y-16/zoom,vr,0,Math.PI*2);ctx.fillStyle=vc[v.type]||'#888';ctx.fill();ctx.fillStyle='#fff';ctx.font='bold '+(7/zoom)+'px Segoe UI';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(vi[v.type]||'V',pos.x-16/zoom,pos.y-16/zoom);}ctx.restore();}\nfunction nodeAt(mx,my){var w=s2w(mx,my);for(var id in positions){var p=positions[id],dx=w.x-p.x,dy=w.y-p.y;if(Math.sqrt(dx*dx+dy*dy)<18/zoom)return id;}return null;}\ncanvas.addEventListener('mousedown',function(e){var r=canvas.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;var n=nodeAt(mx,my);if(n){dragNode=n;}else{dragCanvas=true;}lastMX=mx;lastMY=my;});\ncanvas.addEventListener('mousemove',function(e){var r=canvas.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;if(dragNode){var w=s2w(mx,my);positions[dragNode]={x:w.x,y:w.y};draw();}else if(dragCanvas){panX+=mx-lastMX;panY+=my-lastMY;draw();}else{var n=nodeAt(mx,my),tip=document.getElementById('tip');if(n){var l=null;for(var i=0;i<state.locations.length;i++){if(state.locations[i].id==n){l=state.locations[i];break;}}if(l){tip.style.display='block';tip.style.left=(mx+12)+'px';tip.style.top=(my-10)+'px';tip.innerHTML='<b>'+l.name+'</b><br>ID:'+l.id+' &middot; '+l.type;}}else{tip.style.display='none';}}lastMX=mx;lastMY=my;});\ncanvas.addEventListener('mouseup',function(){dragNode=null;dragCanvas=false;});\ncanvas.addEventListener('mouseleave',function(){dragNode=null;dragCanvas=false;document.getElementById('tip').style.display='none';});\ncanvas.addEventListener('wheel',function(e){e.preventDefault();var r=canvas.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;var f=e.deltaY<0?1.1:0.9;panX=mx-(mx-panX)*f;panY=my-(my-panY)*f;zoom=Math.max(0.3,Math.min(3,zoom*f));draw();},{passive:false});\nfunction zoomIn(){zoom=Math.min(3,zoom*1.2);draw();}function zoomOut(){zoom=Math.max(0.3,zoom/1.2);draw();}function resetView(){zoom=1;panX=0;panY=0;draw();}\nfunction switchTab(t){var tabs=['dispatch','locs','roads','vehs'];for(var i=0;i<tabs.length;i++){document.getElementById('p-'+tabs[i]).classList.toggle('active',tabs[i]===t);document.querySelectorAll('.tab')[i].classList.toggle('active',tabs[i]===t);}refreshUI();}\nfunction log(msg,cls){var el=document.getElementById('log');var d=document.createElement('div');d.className=cls||'';d.textContent=new Date().toLocaleTimeString()+'  '+msg;el.appendChild(d);el.scrollTop=el.scrollHeight;}\nfunction api(url,cb){var x=new XMLHttpRequest();x.open('GET',url);x.onload=function(){try{var j=JSON.parse(x.responseText);cb(j);}catch(e){log('Parse error: '+e,'lerr');}};x.onerror=function(){log('Cannot reach C++ server. Is server.exe running?','lerr');};x.send();}\nfunction applyState(j){state=j;refreshUI();draw();}\nfunction apiLoadSample(){log('Loading sample city...','linfo');api('/api/load-sample',function(j){applyState(j);positions={};hlPath=[];selVeh=-1;log('Sample city loaded!','lok');});}\nfunction apiAddLoc(){var n=document.getElementById('lname').value.trim();var t=document.getElementById('ltype').value;if(!n)return alert('Enter a name');api('/api/add-location?name='+encodeURIComponent(n)+'&type='+t,function(j){applyState(j);document.getElementById('lname').value='';});}\nfunction apiRemoveLoc(id){api('/api/remove-location?id='+id,function(j){delete positions[id];applyState(j);});}\nfunction apiAddRoad(){var fr=document.getElementById('rfrom').value;var to=document.getElementById('rto').value;var dist=document.getElementById('rdist').value;var name=document.getElementById('rname').value||'Road';if(fr===to)return alert('Select two different locations');api('/api/add-road?from='+fr+'&to='+to+'&dist='+dist+'&name='+encodeURIComponent(name),function(j){applyState(j);document.getElementById('rname').value='';});}\nfunction apiToggleRoad(idx){var r=state.roads[idx];api('/api/block-road?from='+r.from+'&to='+r.to+'&block='+(!r.blocked),function(j){hlPath=[];applyState(j);});}\nfunction apiAddVeh(){var n=document.getElementById('vname').value.trim();var t=document.getElementById('vtype').value;var loc=document.getElementById('vloc').value;if(!n)return alert('Enter a name');var speeds={ambulance:80,firetruck:70,police:100,private:180};api('/api/add-vehicle?name='+encodeURIComponent(n)+'&type='+t+'&loc='+loc+'&speed='+speeds[t],function(j){applyState(j);document.getElementById('vname').value='';});}\nfunction findPath(){if(selVeh<0||selVeh>=state.vehicles.length)return alert('Select a vehicle first');var dest=document.getElementById('sdest').value;var v=state.vehicles[selVeh];log('Dispatching '+v.name,'linfo');api('/api/path?from='+v.locId+'&to='+dest,function(j){var el=document.getElementById('result');if(!j.found){hlPath=[];draw();el.innerHTML='<div class=\"rbox nf\"><div class=\"rtitle\">Result</div><span style=\"color:#e24b4a;font-weight:600\">No path found.</span></div>';log('No path found!','lerr');return;}hlPath=j.path;draw();var mins=(j.dist/v.speed*60).toFixed(1);var names=[];for(var i=0;i<j.path.length;i++){var found=null;for(var k=0;k<state.locations.length;k++){if(state.locations[k].id==j.path[i]){found=state.locations[k].name;break;}}names.push(found||j.path[i]);}var isP=v.isPrivate;var route='';for(var i=0;i<names.length;i++){if(i)route+='<span class=\"rarr\">&#8594;</span>';route+='<span class=\"rnode'+(isP?' p':'')+'\">'+names[i]+'</span>';}el.innerHTML='<div class=\"rbox found\"><div class=\"rtitle\">Dispatch &mdash; '+v.name+'</div><div class=\"metrics\"><div class=\"metric\"><div class=\"mval\">'+j.dist.toFixed(2)+'</div><div class=\"mlbl\">km</div></div><div class=\"metric\"><div class=\"mval\">'+mins+'</div><div class=\"mlbl\">min ETA</div></div><div class=\"metric\"><div class=\"mval\">'+j.path.length+'</div><div class=\"mlbl\">stops</div></div></div><div class=\"route\">'+route+'</div></div>';log('Path: '+names.join(' -> ')+' ('+j.dist.toFixed(2)+'km, '+mins+'min)','lok');});}\nfunction refreshUI(){var bc={ambulance:'ba',firetruck:'bf',police:'bp',private:'bpr'};var bt={ambulance:'Ambulance',firetruck:'Fire Truck',police:'Police',private:'Private'};var html='';for(var i=0;i<state.vehicles.length;i++){var v=state.vehicles[i];var loc='';for(var k=0;k<state.locations.length;k++){if(state.locations[k].id==v.locId){loc=state.locations[k].name;break;}}html+='<div class=\"vcard'+(v.isPrivate?' prv':'')+(i===selVeh?' sel':''+'\"')+' onclick=\"selVeh='+i+';refreshUI()\"><div class=\"vn\">'+v.name+'<span class=\"badge '+bc[v.type]+'\">'+bt[v.type]+'</span></div><div class=\"vs\">'+v.speed+'km/h &middot; At: '+loc+'</div></div>';}document.getElementById('vcards').innerHTML=html||'<div style=\"color:#555;font-size:12px\">Load sample city first.</div>';var opts='';for(var i=0;i<state.locations.length;i++){var l=state.locations[i];opts+='<option value=\"'+l.id+'\">'+l.name+' (#'+l.id+')</option>';}document.getElementById('sdest').innerHTML=opts;var clrs={hospital:'#e24b4a',station:'#ef9f27',landmark:'#1d9e75',intersection:'#378add'};var lhtml='';for(var i=0;i<state.locations.length;i++){var l=state.locations[i];lhtml+='<div class=\"liitem\"><div class=\"ldot\" style=\"background:'+clrs[l.type]+'\"></div><div class=\"lname\">'+l.name+'</div><div class=\"lid\">#'+l.id+'</div><button class=\"lrm\" onclick=\"apiRemoveLoc('+l.id+')\">&#10005;</button></div>';}document.getElementById('llist').innerHTML=lhtml||'<div style=\"color:#555;font-size:12px\">No locations.</div>';document.getElementById('rfrom').innerHTML=opts;document.getElementById('rto').innerHTML=opts;var rhtml='';for(var i=0;i<state.roads.length;i++){var r=state.roads[i];rhtml+='<div class=\"ritem'+(r.blocked?' blk':'')+'\"><div class=\"rnm\">'+r.name+'</div><div class=\"rkm\">'+r.dist.toFixed(1)+'km</div><button class=\"rbtn\" onclick=\"apiToggleRoad('+i+')\">'+(r.blocked?'Open':'Block')+'</button></div>';}document.getElementById('rlist').innerHTML=rhtml||'<div style=\"color:#555;font-size:12px\">No roads.</div>';document.getElementById('vloc').innerHTML=opts;var avhtml='';for(var i=0;i<state.vehicles.length;i++){var v=state.vehicles[i];var loc='';for(var k=0;k<state.locations.length;k++){if(state.locations[k].id==v.locId){loc=state.locations[k].name;break;}}avhtml+='<div class=\"vcard'+(v.isPrivate?' prv':'')+'\"><div class=\"vn\">'+v.name+'<span class=\"badge '+bc[v.type]+'\">'+bt[v.type]+'</span></div><div class=\"vs\">'+v.speed+'km/h &middot; At: '+loc+'</div></div>';}document.getElementById('avlist').innerHTML=avhtml||'<div style=\"color:#555;font-size:12px\">No vehicles.</div>';}\nwindow.addEventListener('load',function(){resizeCanvas();apiLoadSample();});\n</script>\n</body>\n</html>";
 
// ─────────────────────────────────────────────
//  HTTP Response builder
// ─────────────────────────────────────────────
string makeResponse(const string& body, const string& ct = "application/json") {
    ostringstream r;
    r << "HTTP/1.1 200 OK\r\n"
      << "Content-Type: " << ct << "; charset=utf-8\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      << "Access-Control-Allow-Origin: *\r\n"
      << "Connection: close\r\n\r\n"
      << body;
    return r.str();
}
 
// ─────────────────────────────────────────────
//  Handle one HTTP request
// ─────────────────────────────────────────────
void handleClient(SOCKET client, CityGraph& city) {
    char buf[8192] = {};
    recv(client, buf, sizeof(buf)-1, 0);
    string req(buf);
 
    string line = req.substr(0, req.find("\r\n"));
    size_t sp1 = line.find(' ');
    size_t sp2 = line.find(' ', sp1+1);
    string url = (sp1!=string::npos && sp2!=string::npos)
                  ? line.substr(sp1+1, sp2-sp1-1) : "/";
 
    string path, query;
    size_t qpos = url.find('?');
    if (qpos != string::npos) { path=url.substr(0,qpos); query=url.substr(qpos+1); }
    else { path=url; }
 
    map<string,string> p = parseQuery(query);
    string resp;
 
    cout << CYAN << "  [REQ] " << path;
    if (!query.empty()) cout << "?" << query;
    cout << "\n" << RESET;
 
    if (path=="/" || path=="/index.html") {
        resp = makeResponse(HTML_PAGE, "text/html");
 
    } else if (path=="/api/state") {
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/load-sample") {
        city.loadSample();
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/add-location") {
        city.addLocation(getP(p,"name","Location"), getP(p,"type","intersection"));
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/remove-location") {
        city.removeLocation(atoi(getP(p,"id","0").c_str()));
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/add-road") {
        city.addRoad(atoi(getP(p,"from","0").c_str()), atoi(getP(p,"to","0").c_str()),
                     atof(getP(p,"dist","1").c_str()), getP(p,"name","Road"));
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/block-road") {
        city.blockRoad(atoi(getP(p,"from","0").c_str()), atoi(getP(p,"to","0").c_str()),
                       getP(p,"block","true")=="true");
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/add-vehicle") {
        city.addVehicle(getP(p,"name","Vehicle"), getP(p,"type","ambulance"),
                        atoi(getP(p,"loc","1").c_str()), atof(getP(p,"speed","80").c_str()));
        resp = makeResponse(cityToJson(city));
 
    } else if (path=="/api/path") {
        int from = atoi(getP(p,"from","1").c_str());
        int to   = atoi(getP(p,"to","2").c_str());
 
        cout << BOLD << CYAN << "\n  [DISPATCH] "
             << city.locations[from].name << " -> " << city.locations[to].name << "\n" << RESET;
 
        CityGraph::PathResult r = city.dijkstra(from, to);
        ostringstream o;
        if (!r.found) {
            cout << RED << "  [X] No path found!\n" << RESET;
            o << "{\"found\":false}";
        } else {
            cout << GREEN << "  [OK] Distance: " << r.dist << " km | Route: ";
            for (int i=0; i<(int)r.path.size(); i++) {
                if (i) cout << " -> ";
                cout << city.locations[r.path[i]].name;
            }
            cout << "\n" << RESET;
            o << "{\"found\":true,\"dist\":" << r.dist << ",\"path\":[";
            for (int i=0; i<(int)r.path.size(); i++) { if(i) o<<","; o<<r.path[i]; }
            o << "]}";
        }
        resp = makeResponse(o.str());
 
    } else {
        string body="{\"error\":\"not found\"}";
        resp="HTTP/1.1 404 Not Found\r\nContent-Length: "+to_string(body.size())+"\r\n\r\n"+body;
    }
 
    send(client, resp.c_str(), (int)resp.size(), 0);
    closesocket(client);
}
 
// ─────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────
int main() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        cout << RED << "WSAStartup failed\n" << RESET; return 1;
    }
    SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == INVALID_SOCKET) {
        cout << RED << "Socket failed\n" << RESET; return 1;
    }
    int opt=1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
 
    sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(8080);
 
    if (bind(server,(sockaddr*)&addr,sizeof(addr))==SOCKET_ERROR) {
        cout << RED << "Bind failed. Port 8080 may already be in use.\n" << RESET; return 1;
    }
    listen(server, 10);
 
    CityGraph city;
    city.loadSample();
 
    cout << BOLD << RED;
    cout << "\n+================================================+\n";
    cout << "|   EMERGENCY PATH FINDER  -  C++ SERVER        |\n";
    cout << "+================================================+\n" << RESET;
    cout << GREEN << "  Server: " << RESET << BOLD << "http://localhost:8080\n" << RESET;
    cout << CYAN  << "  Open that URL in Chrome / Edge / Firefox\n" << RESET;
    cout << YELLOW<< "  Press Ctrl+C to stop.\n\n" << RESET;
 
    while (true) {
        sockaddr_in ca; int cl=sizeof(ca);
        SOCKET client = accept(server,(sockaddr*)&ca,&cl);
        if (client==INVALID_SOCKET) continue;
        handleClient(client, city);
    }
    closesocket(server);
    WSACleanup();
    return 0;
}
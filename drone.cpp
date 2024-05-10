
#include <algorithm>
#include <cassert>
#include <deque>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <math.h>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <iomanip>
#include <getopt.h>

using namespace std;

// Create enum for locations for MST
enum class Location {
    main,
    border,
    medical
};

// Make a point for when location matters (MST)
struct PointA {
    int x;
    int y;
    Location location;
};

// Make a point for when location does not matter
struct Point {
    int x;
    int y;
};

// Struct for Prim's Algorithm
struct Prim {
    double dv = numeric_limits<double>::infinity();
    size_t pv;
    bool kv = false;
};




class Drone {

private:
    // Distance matrix for optimal TSP
    vector<vector<double>> distances;

    // Storing points for various reasons
    vector<PointA> pointsA;
    vector<Prim> primStuff;
    vector<Point> points;
    vector<Point> pointsToCheck;

    // Keeping track of final paths
    vector<size_t> fastPath;
    vector<size_t> bestPath;
    double bestTour = 0;
    double currTour = 0;
    double tourLength = 0;
    double promisingLength;

    size_t forMst = 0;

    char mode;


public:

    //Take in arguments
    void readArgs(int argc, char** argv) {
        int option_index = 0, option = 0;
    
        opterr = false;

        struct option longOpts[] = {{ "help", no_argument, nullptr, 'h' },
                                { "mode", required_argument, nullptr, 'm'}};

        while 
        ((option =
        getopt_long(argc, argv, "hm:", longOpts, &option_index)) != -1) {
            switch (option) {
                //Asking for help
                case 'h':
                    cout << 
                    "Must give a valid mode to find optimal tsp, fast tsp, or mst"
                     << endl;
                    exit(0);
                //Selecting the mode
                case 'm':
                    if (optarg && optarg[0] != '\0') {
                        mode = optarg[0];
                    } else {
                        cerr << "Invalid argument for mode option.\n";
                        exit(1);
                    }
                    break;

            }
        }
    }

    //Take in the input for each mode
    void takeInput() {
        size_t amount;
        cin >> amount;
        int x;
        int y;

        if (mode == 'M') {
            pointsA.reserve(amount);
            primStuff.resize(amount);
            while (amount != 0) {
            cin >> x >> y;
            if (x > 0 || y > 0)
                pointsA.push_back({x, y, Location::main});
            else if (x == 0 || y == 0)
                pointsA.push_back({x, y, Location::border});
            else 
                pointsA.push_back({x, y, Location::medical});

            amount--;
            }
        }

        else {
            points.reserve(amount);
            while (amount != 0) {
                cin >> x >> y;
                points.push_back({x, y});
                amount--;
            }
        }
    }
    //Calculating Euclidean Distance Squared for MST mode
    double distanceSquaredA(const PointA &a, const PointA &b) {
        //Drone has to travel through the border
        if (a.location != b.location && a.location != Location::border &&
            b.location != Location::border) {
                return numeric_limits<double>::infinity();
        }

        double xMinus = static_cast<double>(a.x - b.x);
        double yMinus = static_cast<double>(a.y - b.y);

        return (xMinus * xMinus) + (yMinus * yMinus);

    }

    //Calculating Euclidean Distance Squared for TSP
    double distanceSquared(const Point &a, const Point &b) {
        double xMinus = static_cast<double>(a.x - b.x);
        double yMinus = static_cast<double>(a.y - b.y);

        return (xMinus * xMinus) + (yMinus * yMinus);
    }

    double distance(const Point &a, const Point &b) {
        return sqrt(distanceSquared(a,b));
    }

    void runAlgorithm() {
        if (mode == 'M') mst();
        else if (mode == 'F') fasttsp();
        else opttsp();
    }

    //Using Prim's for MST
    void mst() {
        double totalDistance = 0.0;
        primStuff[0].dv = 0;
        size_t n = 0;
        if (mode == 'M') n = pointsA.size();
        else n = fastPath.size() - forMst;
        for (size_t i = 0; i < n; i++) {
            double min = numeric_limits<double>::infinity();
            size_t minIndex = 0;
            for (size_t j = 0; j < n; j++) {
                if (!primStuff[j].kv) {
                    if (primStuff[j].dv < min) {
                        min = primStuff[j].dv;
                        minIndex = j;
                    }
                }
            }
            if (min == numeric_limits<double>::infinity()) {
                cerr << "Cannot construct MST\n";
                exit(1);
            }
            primStuff[minIndex].kv = true;
            totalDistance += sqrt(primStuff[minIndex].dv);
            
            for (size_t j = 0; j < n; j++) {
                if (!primStuff[j].kv) {
                    double distance;
                    if (mode == 'M')
                        distance = distanceSquaredA(pointsA[minIndex], pointsA[j]);
                    else distance = 
                    distances[fastPath[forMst + minIndex]][fastPath[forMst + j]];    
                    if (distance < primStuff[j].dv) {
                        primStuff[j].dv = distance;
                        primStuff[j].pv = minIndex;
                    }
                }
            }
        }

        if (mode != 'M') {
            promisingLength = totalDistance + currTour;
        }
        else {
            cout << totalDistance << '\n';

            for (size_t i = 1; i < primStuff.size(); i++) {
                if (i < primStuff[i].pv) {
                    cout << i << " " << primStuff[i].pv;
                }
                else {
                    cout << primStuff[i].pv << " " << i;
                }
                cout << '\n';
            }
        }
    }

    //Using arbitrary insertion heuristic
    void fasttsp() {
        size_t n = points.size();
        fastPath.reserve(n);
        double minCost = numeric_limits<double>::infinity();
        size_t minIndex = 0;
        fastPath.push_back(0);
        fastPath.push_back(1);
        tourLength = 2 * distance(points[0], points[1]);

        for (size_t i = 2; i < n; i++) {
            for (size_t j = 0; j < fastPath.size() - 1; j++) {
                double cost =
                distance(points[i], points[fastPath[j]]) 
                + distance(points[i], points[fastPath[j + 1]])
                - distance(points[fastPath[j]], points[fastPath[j + 1]]);
                if (cost < minCost) {
                    minCost = cost;
                    minIndex = j;
                }
            }

            fastPath.insert(fastPath.begin() + int(minIndex) + 1, i);
            tourLength += minCost;
            minCost = numeric_limits<double>::infinity();
        }
        twoOpt();
    }

    //Quick 2-opt for further optimization
    void twoOpt() {

        size_t times = 2;
        if (mode == 'O') times = 4;

        for (size_t h = 0; h < times; h++) {
            for (size_t i = 0; i < points.size() - 2; i++) {
                // Don't actually check every point
                for (size_t j = i + 2; j < points.size() && j < i + 30; j++) {
                    double difference = distance(points[fastPath[i]],
                    points[fastPath[j]]) + 
                    distance(points[fastPath[i + 1]],
                    points[fastPath[(j + 1) % points.size()]]) -
                    distance(points[fastPath[i]], points[fastPath[i + 1]]) -
                    distance(points[fastPath[j]],
                    points[fastPath[(j + 1) % points.size()]]);
                
                    if (difference < 0) {
                        tourLength += difference;
                        reverse(fastPath.begin() + static_cast<int>(i + 1),
                        fastPath.begin() + static_cast<int>(j + 1));
                    }
                }
            }
        }

        size_t zeroIndex = 0;

        for (size_t i = 0; i < points.size(); i++) {
            if (fastPath[i] == 0) {
                zeroIndex = i;
                break;
            }
        }
        if (mode == 'F') cout << tourLength << '\n';
        else bestTour = tourLength;

        for (size_t i = 0; i < points.size(); i++) {
            if (mode == 'F') cout << fastPath[zeroIndex] % points.size() << ' ';
            else bestPath.push_back(fastPath[zeroIndex] % points.size());
            zeroIndex++;
        }
        if (mode == 'F') cout << '\n';
    }

    //Optimal TSP
    void opttsp() {
        distances.resize(points.size());
        // Store Adjacency Matrix
        for (size_t i = 0; i < points.size(); i++) {
            distances[i].resize(points.size());
            for (size_t j = 0; j < points.size(); j++) {
                distances[i][j] = distanceSquared(points[i], points[j]);
            }
        }
        bestPath.reserve(points.size());
        // Use Heuristic to find upper bound
        fasttsp();
        genPerms(1);
        cout << bestTour << '\n';
        for (size_t i = 0; i < bestPath.size(); i++) {
            cout << bestPath[i] << ' ';
        }
        cout << '\n';
    }

    // Check if a branch is promising by looking at an mst of remaining vertices
    bool isPromising(size_t permLength) {
        primStuff.resize(fastPath.size() - permLength);
        forMst = permLength;
        mst();
        primStuff.clear();

        double min1 = numeric_limits<double>::infinity();
        double min2 = numeric_limits<double>::infinity();
        for (size_t i = 0; i < fastPath.size() - permLength; i++) {
            double d1 = distances[0][fastPath[forMst + i]];
            if (d1 < min1) {
                min1 = d1;
            }
            double d2 = distances[fastPath[permLength - 1]][fastPath[forMst + i]];
            if (d2 < min2) {
                min2 = d2;
            }
        }

        return promisingLength + sqrt(min1) + sqrt(min2) < bestTour;
    }

    // Rerursive Branch and Bound Algorithm
    void genPerms(size_t permLength) {
        if (permLength == fastPath.size()) {
            double closeEdge = sqrt(distances[fastPath[permLength - 1]][0]);
            currTour += closeEdge;
            if (currTour < bestTour) {
                bestTour = currTour;
                bestPath = fastPath;
            }
            currTour -= closeEdge;
            return;
        }
        if (!(isPromising(permLength))) {
            return;
        }

        for (size_t i = permLength; i < fastPath.size(); i++) {
            swap(fastPath[permLength], fastPath[i]);
            double newEdge = 
            sqrt(distances[fastPath[permLength]][fastPath[permLength-1]]);
            currTour += newEdge;
            genPerms(permLength + 1);
            currTour -= newEdge;
            swap(fastPath[permLength], fastPath[i]);
        }
    }

};



int main(int argc, char** argv) {
    ios_base::sync_with_stdio(false);
    cout << std::setprecision(2);
    cout << std::fixed;

    Drone uav;

    uav.readArgs(argc, argv);

    uav.takeInput();

    uav.runAlgorithm();

    return 0;
}

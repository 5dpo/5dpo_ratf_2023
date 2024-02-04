//
// Created by jpc00 on 22/02/2023.
//

#ifndef INC_5DPO_GRAPH_H
#define INC_5DPO_GRAPH_H
#include <iostream>
#include <vector>
#include <queue>
#include <iterator>
#include <deque>
#include <list>
#include <limits>
#include <cmath>
#include "MutablePriorityQueue.h"
#include "Location.h"


using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

#define INF std::numeric_limits<double>::max()


/****************** Provided structures  ********************/

template <class T>
class Vertex {
    T info;                // contents
    vector<Edge<T> > adj;  // list of outgoing edges
    bool visited;          // auxiliary field used by dfs and bfs
    bool processing;       // auxiliary field used by isDAG
    int indegree;          // auxiliary field used by topsort
    Vertex * path;
    double dist;

    void addEdge(Vertex<T> *dest, double w);
    bool removeEdgeTo(Vertex<T> *d);
public:
    int queueIndex;
    T getInfo() const;
    double getDist();
    Vertex *getPath() const;
    Vertex(T in);
    friend class Graph<T>;
};

template <class T>
bool operator<(Vertex<T> v, Vertex<T> v2);

template <class T>
bool operator!=(T v, T v2);

template <class T>
class Edge {
    Vertex<T> * dest;      // destination vertex
    double weight;         // edge weight
public:
    Edge(Vertex<T> *d, double w);
    friend class Graph<T>;
    friend class Vertex<T>;
};

template <class T>
class Graph {
    vector<vector<double>> minDist;
    vector<vector<Vertex<T>*>> next;


    void dfsVisit(Vertex<T> *v,  vector<T> & res) const;
    Vertex<T> *findVertex(const T &in) const;
    bool dfsIsDAG(Vertex<T> *v) const;
public:
    vector<Vertex<T> *> vertexSet;
    vector<vector<Vertex<T>*>> getnext() const;
    vector<int> Ids;
    int getNumVertex() const;
    bool addVertex(const T &in);
    bool removeVertex(const T &in);
    bool addEdge(const T &sourc, const T &dest);
    bool removeEdge(const T &sourc, const T &dest);
    vector<T> dfs() const;
    vector<T> bfs(const T &source) const;
    vector<T> topsort() const;
    int maxNewChildren(const T &source, T &inf) const;
    bool isDAG() const;
    vector<T> dijkstraShortestPath(const T &origin, const T &dest);
    void unweightedShortestPath(const T &orig);
    vector<T> getPath(const T &origin, const T &dest) const;
    void floydWarshallShortestPath();
    vector<T> getfloydWarshallPath(const T &orig, const T &dest) const;
    vector<T> getSingleDeliveryPath(const T & origin, const T & dest, vector<T> deliveryPoints);
    Vertex<T> *findVertexById(const int id) const;
    T findClosestLocation(const T &rob);
};

/****************** Provided constructors and functions ********************/

template <class T>
Vertex<T>::Vertex(T in): info(in), path(NULL), dist(0) {}

template <class T>
Edge<T>::Edge(Vertex<T> *d, double w): dest(d), weight(w) {}


template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
vector<vector<Vertex<T>*>> Graph<T>::getnext() const
{
    return next;
}


/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet)
        if (v->info == in)
            return v;
    return NULL;
}
template <class T>
Vertex<T> * Graph<T>::findVertexById(const int id) const {
    for (auto v : vertexSet)
        if (v->info.getID()==id) {
            return v;
        }

    return NULL;
}


template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
Vertex<T> *Vertex<T>::getPath() const {
    return this->path;
}


/****************** 1a) addVertex ********************/

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if ( findVertex(in) != NULL)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    Ids.push_back(in.getID());
    return true;
}

/****************** 1b) addEdge ********************/

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL) {

        return false;
    }

    v1->addEdge(v2, getDistBetweenLocations(v1->info,v2->info));
    return true;
}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
void Vertex<T>::addEdge(Vertex<T> *d, double w) {
    adj.push_back(Edge<T>(d, w));
}


/****************** 1c) removeEdge ********************/

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    return v1->removeEdgeTo(v2);
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Vertex<T>::removeEdgeTo(Vertex<T> *d) {
    for (auto it = adj.begin(); it != adj.end(); it++)
        if (it->dest  == d) {
            adj.erase(it);
            return true;
        }
    return false;
}


/****************** 1d) removeVertex ********************/

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++)
        if ((*it)->info  == in) {
            auto v = *it;
            vertexSet.erase(it);
            for (auto u : vertexSet)
                u->removeEdgeTo(v);
            delete v;
            return true;
        }
    return false;
}

template<class T>
double Vertex<T>::getDist() {
    return this->dist;
}


template <class T>
bool operator<(Vertex<T> v, Vertex<T> v2) {
    return v.getDist() < v2.getDist();
}

template <class T>
bool operator!=(T v, T v2) {
    return v.getID() == v2.getID(); //might need to review
}

template <class T>
T Graph<T>::findClosestLocation(const T &rob){
    T res =Location(0,1,0);
    double min_dist= 10000000;
    for (auto v : vertexSet){
        double dist=getDistBetweenLocations(rob,v->info);
        if (dist < min_dist){
            min_dist=dist;
            res=v->info;
        }
    }  
    return res;
}


/****************** 2a) dfs ********************/

/*
 * Performs a depth-first search (dfs) in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T>
vector<T> Graph<T>::dfs() const {
    vector<T> res;
    for (auto v : vertexSet)
        v->visited = false;
    for (auto v : vertexSet)
        if (! v->visited)
            dfsVisit(v, res);
    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, vector<T> & res) const {
    v->visited = true;
    res.push_back(v->info);
    for (auto & e : v->adj) {
        auto w = e.dest;
        if ( ! w->visited)
            dfsVisit(w, res);
    }
}


template<class T>
vector<T> Graph<T>::dijkstraShortestPath(const T &origin, const T &dest) {
    for(Vertex<T>* v: this->vertexSet){
        v->visited = false;
        v->dist = INF;
        v->path = NULL;
    }
    Vertex<T>* v = this->findVertex(origin);
    v->dist = 0;
    MutablePriorityQueue<Vertex<T>> queue;
    queue.insert(v);

    while (!queue.empty()){
        Vertex<T>* v= queue.extractMin();

        for(Edge<T> edge: v->adj){
            if(edge.dest->dist > v->getDist() + edge.weight){
                edge.dest->dist = v->getDist() + edge.weight;
                edge.dest->path = v;
                if(queue.find(edge.dest)){
                    queue.decreaseKey(edge.dest);
                } else{
                    queue.insert(edge.dest);
                }
            }
        }

        if(v == this->findVertex(dest)) {
            return getPath(origin,dest);
            
        }

    }

    queue.elements();
    return getPath(origin,dest); //Might need to review this.
}

template<class T>
vector<T> Graph<T>::getPath(const T &origin, const T &dest) const{
    vector<T> res;
    Vertex<T> * v = this->findVertex(dest);
    //cout << v->getInfo().getID() << "\n";
   
    while (true){
        if (v->getInfo().getID() == origin.getID()) {
            break;
        }
        else {
            res.push_back(v->getInfo());
            v = v->getPath();
        }
    }
     
    res.push_back(v->getInfo());
    reverse(res.begin(), res.end());
    return res;
}

#endif //INC_5DPO_GRAPH_H

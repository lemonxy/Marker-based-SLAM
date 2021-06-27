//! \file graphs.h
//! \author mjmarin
//! \date  Jun/2016


#ifndef GRAPHS_H
#define GRAPHS_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include <utility>
#include <set>
#include <cstdint>
#include <mutex>
using namespace std;

namespace ucoslam
{


class CovisGraph
{
public:
    CovisGraph();
    CovisGraph(      CovisGraph &G);
    CovisGraph & operator=(      CovisGraph &G);
    std::set<uint32_t> getNeighbors(uint32_t idx,bool includeIdx=false,float minWeight=0)  ;
    std::vector<uint32_t> getNeighborsV(uint32_t idx,bool includeIdx=false)  ;
    //returns the neighbors and weights sorted if required
    std::vector<pair<uint32_t, float> > getNeighborsWeights(uint32_t idx, bool sorted)  ;

    void removeNode(uint32_t idx);



    std::set<uint32_t> getNodes(void)const {return _nodes;}
    //Returns the connection between the nodes if created (if not, throws exception). If created=true, the connection is created and returns a reference to the weight
    //If created, the weight is initialized to 0
    float  getEdge(uint32_t idx1, uint32_t idx2  );
    bool  isEdge(uint32_t idx1, uint32_t idx2  );
    void createIncreaseEdge(uint32_t idx1, uint32_t idx2,float increaseValue=1);
    bool decreaseRemoveEdge(uint32_t idx1, uint32_t idx2);


    float getWeight(uint32_t a ,uint32_t b)const{
        assert(_mweights.count(join(a,b)));
        return _mweights.find(join(a,b))->second;
    }

    //! \brief Computes the spanning tree or essential graph
    void getEG(CovisGraph & );

    //! Number of edges
    uint32_t size(void){return _nodes.size();}

    // I/O
    void toStream(ostream &str) const;
    void fromStream(istream &str)throw(std::exception);

    const std::map<uint64_t,float> &getEdgeWeightMap()const{return _mweights;}

    //divides a 64bit edge into its components
    static pair<uint32_t,uint32_t> separe(uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);}

    bool isNode(uint32_t node){return _nodes.count(node);}
    void clear(){
        _nodes.clear();
        _mgraph.clear();
        _mweights.clear();
    }



    std::vector<uint32_t> getNeighbors_safe(uint32_t idx,bool includeIdx=false)  ;

    //returns the connection matrix
    cv::Mat getConnectionMatrix(bool useWeights=true)const;

    //returns the shortest path between the two nodes using a Breadth-first search from the org node. If returns empty, there is no path
    //excluded_links: can be used to avoid using some of the connections
    vector<uint32_t> getShortestPath(uint32_t org, uint32_t end     );




    static  uint64_t join(uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }
private:
    void addEdge(uint32_t idx1, uint32_t idx2, float w=1.0);

    std::set<uint32_t> _nodes;
    std::map<uint32_t,std::set<uint32_t> > _mgraph;//set of nodes links for each node.
    std::map<uint64_t,float> _mweights;//map of edge-weights
    std::mutex lock_mutex;
public:
    friend ostream& operator<<(ostream& os, const CovisGraph & cvg)
    {
        for (auto e : cvg._mweights)
        {
            auto edge= cvg.separe(e.first);
            os << edge.first << "--" << edge.second << " (" << e.second << ")" ;
        }
        return os;
    }

};

}

#endif // GRAPHS_H

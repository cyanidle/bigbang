#ifndef PLANER_NODE_GRAPH_H
#define PLANER_NODE_GRAPH_H

#include <QList>
#include <vector>
#include <boost/pool/pool_alloc.hpp>
template <typename T>
class CachedGraph
{
public:
    CachedGraph(quint32 size) : 
        m_graph(), m_batchSize(size) 
    {
        m_graph.reserve(m_batchSize);
    }
    T &at(quint32 index) {
        return m_graph[index];
    }
    const T &at(quint32 index) const {
        return m_graph.at(index);
    }
    void updateBatchSize(quint32 batchSize) {
        m_batchSize = batchSize;
        m_graph.reserve(m_batchSize);
    }
    template <typename U>
    quint32 append(U&& node) {
        m_graph.push_back(std::forward<U>(node));
        return m_graph.size() - 1;
    }
    void reset() {m_graph.clear(); m_graph.reserve(m_batchSize);}
    quint32 size() const {return m_graph.size();}
private:
    std::vector<T> m_graph;
    quint32 m_batchSize;
};

#endif

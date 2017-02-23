/**
Graph Class (currently Tree). The code is in the HEADER,
because of the template usage. If we don't do that, we
have some trouble instantiating things like this :

Graph<std::vector<float>, int> graph;

2 solutions for that :
Add lines explicitly instantiating all the relevant
templates, such as

template class Graph<std::vector<float>, int>;
template class Graph<std::vector<int>, int>;

But we have to define every possible combination,
and it kinda ruins the whole template thing here.
Or we can put the code into the header file, which
doesn't require us to specifically instantiate all
relevant templates. One drawback is that it can
slows the compilation time a little.

Note that you HAVE to overload the = operator for
your node type, if it's a custom one.

Graph.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_GRAPH_H__
#define __MLA_GRAPH_H__

#include "MLACommonInclude.h"



template<typename T>
class MLANode;


template<typename T>
class MLANode {

	public:
		MLANode();
		MLANode(const unsigned int& parent, const T value, const std::string& name="");
		MLANode(const MLANode<T>& node);
		MLANode& operator=(const MLANode<T>& node);
		const bool operator==(const MLANode<T>& node) const;
		const bool operator!=(const MLANode<T>& node) const;


	public:
		int parent;
		std::vector<unsigned int> child;
		T nodeValue;
		std::string name;
};

template<typename T>
class MLAGraph;


template<typename T>
class MLAGraph{
	public:

		MLAGraph();
		MLAGraph(const MLAGraph<T>& graph);
		virtual ~MLAGraph();

		MLAGraph& operator=(const MLAGraph<T>& graph);
		const bool operator==(const MLAGraph<T>& graph) const;
		const bool operator!=(const MLAGraph<T>& graph) const;

		MLANode<T>* getNode(const unsigned int& idx);
		MLANode<T>* getNode(const std::string& idx);

		T* getNodeValue(const unsigned int& idx);
		T* getNodeValue(const std::string& idx);
		const unsigned int getSizeGraph() const;
		void setChild(const int& parent_idx, const unsigned int& child_idx);
		void addRoot(const T& node);
		void addChild(const int& parent, const T& node);
		void addChild(const std::string& parent, const T& node);


	private:
		std::vector<MLANode<T> > m_nodes;
		std::map<std::string, unsigned int> m_names;
};

#include "MLAGraph.inl"

#endif //__MLA_GRAPH_H__

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
class MlaNode {

	public:
		MlaNode();
		MlaNode(const unsigned int& parent, const T value);
		MlaNode(const MlaNode<T>& node);
		MlaNode& operator=(const MlaNode<T>& node);

	public:
		int parent;
		std::vector<unsigned int> child;
		T nodeValue;
};

template<typename T>
class MlaGraph{
	public:

		MlaGraph();
		virtual ~MlaGraph();

		const MlaNode<T>& getNode(const unsigned int& idx) const;

		T& getNodeValues(const unsigned int& idx);

		const unsigned int getSizeNode() const;

		void setChild(const int& parent_idx, const unsigned int& child_idx);

		void addRoot(const T& node);

		void addChild(const int& parent, const T& node);



	private:
		std::vector<MlaNode<T> > m_nodes;
};

#include "MlaGraph.inl"

#endif //__MLA_GRAPH_H__

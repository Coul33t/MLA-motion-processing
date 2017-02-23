#ifndef __MLA_TREE_H__
#define __MLA_TREE_H__

#include "MLAGraph/MLAGraph.h"

template<typename Node>
class MLATree;

template<typename Node>
class MLATree {

public:
	MLATree();
	~MLATree();
	MLATree(const MLATree<Node>& tree);

	MLATree& operator=(const MLATree<Node>& tree);
	
	const bool operator==(const MLATree<Node>& tree) const;
	const bool operator!=(const MLATree<Node>& tree) const;

	Node* getNode(const unsigned int& idx) const;
	Node* getNode(const std::string& name) const;

	void insertNode(Node* node, const unsigned int& parent_idx);
	void insertNode(Node* node, const std::string& parent_name);

	const unsigned int NodeNumber(void) const;

private:
	MLAGraph<Node> m_graph;
};

#include "MLATree.inl"



#endif //__MLA_TREE_H__